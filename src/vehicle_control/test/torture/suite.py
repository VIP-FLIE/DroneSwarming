"""Safety contracts for real nodes exposed to bounded adversarial PX4 message traces."""

from __future__ import annotations

import json
import math
import signal
import unittest
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from swarm_interfaces.action import (
    ExecuteFlightPlan,
    ExecuteWaypointMission,
    FollowTrajectory,
    Land,
    NavigateTo,
    OrbitPoint,
    Takeoff,
)
from swarm_interfaces.msg import TrajectoryPoint

from peer import LinkFault, OBSERVATION_SEC, Peer

FAULT_REACTION_SEC: float = 1.2
MAXIMUM_RSS_GROWTH_KIB: int = 32 * 1024
CHURN_CYCLES: int = 20
COMMAND_STORM_SIZE: int = 24


class TortureTests(unittest.TestCase):
    """Run one case per supervised process; failures remain ordinary unittest failures."""

    artifact_directory: Path
    seed: int

    def setUp(self) -> None:
        rclpy.init(args=['--ros-args', '--disable-stdout-logs', '--enable-external-lib-logs'])
        self.addCleanup(rclpy.shutdown)
        self.peer = Peer(self.artifact_directory, self.seed)
        self.addCleanup(self.peer.close)

    def navigation(self, timeout_sec: float = 3.0) -> NavigateTo.Goal:
        """Place the target five meters east of the synthetic vehicle."""
        goal = NavigateTo.Goal()
        goal.frame_id = 'local_enu'
        goal.target_position = Point(x=12.0, y=5.0, z=3.0)
        goal.timeout_sec = timeout_sec
        return goal

    def navigate(self) -> ClientGoalHandle:
        handle: ClientGoalHandle = self.peer.send('navigate_to', NavigateTo, self.navigation())
        self.peer.check(handle.accepted, 'valid navigation admitted', handle.accepted, True)
        self.peer.wait(
            lambda: bool(self.peer.setpoints) and self.peer.setpoints[-1].position[1] == 12.0,
            'requested navigation setpoint',
            1.0,
        )
        return handle

    def trajectory(self) -> FollowTrajectory.Goal:
        """Build a feasible two-second route with a distinct intermediate target."""
        goal = FollowTrajectory.Goal()
        goal.timeout_sec = 4.0
        for elapsed, east in ((0.0, 7.0), (1.0, 8.0), (2.0, 7.0)):
            point = TrajectoryPoint()
            point.position = Point(x=east, y=5.0, z=3.0)
            point.time_from_start_sec = elapsed
            goal.points.append(point)
        return goal

    def takeoff(self) -> ClientGoalHandle:
        goal = Takeoff.Goal()
        goal.altitude_m = 3.0
        handle: ClientGoalHandle = self.peer.send('takeoff', Takeoff, goal)
        self.peer.check(handle.accepted, 'valid takeoff admitted', handle.accepted, True)
        return handle

    def no_arming(self) -> None:
        arming: int = sum(
            command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            and command.param1 > 0.0
            for command in self.peer.commands
        )
        self.peer.check(arming == 0, 'fault never bypasses arming checks', arming, 0)

    def test_control_navigation(self) -> None:
        """Control: valid telemetry and a reached destination produce success."""
        self.peer.start()
        handle: ClientGoalHandle = self.navigate()
        self.peer.position.y = 12.0
        response = self.peer.outcome(handle)
        self.peer.check(
            response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success,
            'healthy navigation succeeds',
            response.status,
            GoalStatus.STATUS_SUCCEEDED,
        )

    def test_control_takeoff_land(self) -> None:
        """Control: command ACKs and synthetic motion complete takeoff and terminal landing."""
        self.peer.ground()
        self.peer.follow_setpoints = True
        self.peer.start()
        response = self.peer.outcome(self.takeoff())
        self.peer.check(
            response.result.success, 'healthy takeoff succeeds', response.result.message, 'success'
        )
        self.peer.pump(0.15)
        goal = Land.Goal()
        goal.timeout_sec = 4.0
        handle: ClientGoalHandle = self.peer.send('land', Land, goal)
        self.peer.check(handle.accepted, 'landing admitted', handle.accepted, True)
        response = self.peer.outcome(handle)
        self.peer.check(
            response.result.success, 'healthy landing succeeds', response.result.message, 'success'
        )

    def test_disabled_node_is_passive(self) -> None:
        """ISS-01: disabled nodes send neither heartbeat, setpoints, nor commands."""
        self.peer.start(enabled=False)
        self.peer.pump(OBSERVATION_SEC)
        count: int = sum(self.peer.counts[name] for name in ('command', 'heartbeat', 'setpoint'))
        self.peer.check(count == 0, 'disabled node is passive', count, 0)

    def test_restart_does_not_adopt_offboard(self) -> None:
        """ISS-01: a newly enabled node cannot adopt another controller's Offboard flight."""
        self.peer.start()
        self.peer.pump(OBSERVATION_SEC)
        self.peer.check(
            self.peer.counts['setpoint'] == 0,
            'unowned Offboard flight receives no target',
            self.peer.counts['setpoint'],
            0,
        )

    def test_position_dropout_aborts_navigation(self) -> None:
        """ISS-02/03: sustained position-link loss aborts active navigation."""
        self.peer.start()
        handle: ClientGoalHandle = self.navigate()
        self.peer.fault('position', LinkFault(paused=True))
        response = self.peer.outcome(handle, FAULT_REACTION_SEC)
        self.peer.check(
            not response.result.success,
            'position loss cannot report success',
            response.status,
            'unsuccessful',
        )
        self.no_arming()

    def test_status_dropout_aborts_navigation(self) -> None:
        """ISS-03/14: fresh positions cannot conceal loss of control-status telemetry."""
        self.peer.start()
        handle: ClientGoalHandle = self.navigate()
        self.peer.fault('status', LinkFault(paused=True))
        response = self.peer.outcome(handle, FAULT_REACTION_SEC)
        self.peer.check(
            not response.result.success,
            'status loss cannot report success',
            response.status,
            'unsuccessful',
        )

    def test_stale_telemetry_does_not_sustain_heartbeat(self) -> None:
        """ISS-02: an expired control snapshot cannot indefinitely advertise healthy control."""
        self.peer.start()
        self.navigate()
        for stream in ('position', 'status', 'land'):
            self.peer.fault(stream, LinkFault(paused=True))
        self.peer.pump(FAULT_REACTION_SEC)
        before: int = self.peer.counts['heartbeat']
        self.peer.pump(0.4)
        additional: int = self.peer.counts['heartbeat'] - before
        self.peer.check(
            additional == 0,
            'expired telemetry releases heartbeat after recovery budget',
            additional,
            0,
        )

    def test_frozen_source_timestamp_expires(self) -> None:
        """ISS-14: repeated packets with a frozen source timestamp become stale."""
        self.peer.start()
        self.peer.fault('position', LinkFault(freeze_timestamp=True))
        self.peer.pump(FAULT_REACTION_SEC)
        valid: bool = self.peer.latest_state is not None and self.peer.latest_state.position_valid
        self.peer.check(not valid, 'replayed position is invalidated', valid, False)

    def test_out_of_order_position_is_ignored(self) -> None:
        """ISS-14: an old source sample cannot replace a newer vehicle position."""
        self.peer.start()
        self.peer.fault('position', LinkFault(timestamp_offset_us=-5_000_000))
        self.peer.position.y = 400.0
        self.peer.pump(0.2)
        accepted: bool = (
            self.peer.latest_state is not None
            and self.peer.latest_state.position_valid
            and self.peer.latest_state.position.x == 400.0
        )
        self.peer.check(
            not accepted, 'older source sample cannot move valid state', accepted, False
        )

    def test_nonfinite_position_aborts_without_invalid_outputs(self) -> None:
        """ISS-08/14: NaN position aborts navigation while outgoing targets stay finite."""
        self.peer.start()
        handle: ClientGoalHandle = self.navigate()
        self.peer.event('phase', name='inject_nan_position')
        self.peer.position.x = math.nan
        response = self.peer.outcome(handle, FAULT_REACTION_SEC)
        self.peer.check(
            not response.result.success,
            'invalid position aborts navigation',
            response.status,
            'unsuccessful',
        )
        finite: bool = all(
            math.isfinite(value) for point in self.peer.setpoints for value in point.position
        )
        self.peer.check(finite, 'position setpoint fields remain finite', finite, True)

    def test_invalid_yaw_aborts_heading_trajectory(self) -> None:
        """ISS-03: heading control terminates when the yaw estimate becomes invalid."""
        self.peer.start()
        goal: FollowTrajectory.Goal = self.trajectory()
        goal.control_yaw = True
        handle: ClientGoalHandle = self.peer.send('follow_trajectory', FollowTrajectory, goal)
        self.peer.check(handle.accepted, 'heading trajectory admitted', handle.accepted, True)
        self.peer.position.heading_good_for_control = False
        self.peer.position.heading = math.inf
        response = self.peer.outcome(handle, FAULT_REACTION_SEC)
        self.peer.check(
            not response.result.success,
            'invalid yaw cannot report trajectory success',
            response.status,
            'unsuccessful',
        )

    def test_frame_reset_aborts_navigation(self) -> None:
        """ISS-04: an estimator frame reset invalidates an active absolute target."""
        self.peer.start()
        handle: ClientGoalHandle = self.navigate()
        self.peer.event('phase', name='xy_reset_20_m')
        self.peer.position.xy_reset_counter += 1
        self.peer.position.y += 20.0
        response = self.peer.outcome(handle, FAULT_REACTION_SEC)
        self.peer.check(
            not response.result.success,
            'reset navigation fails closed',
            response.status,
            'unsuccessful',
        )

    def test_frame_reset_aborts_trajectory(self) -> None:
        """ISS-04: timed routes cannot continue using targets from an earlier local frame."""
        self.peer.start()
        handle: ClientGoalHandle = self.peer.send(
            'follow_trajectory', FollowTrajectory, self.trajectory()
        )
        self.peer.check(handle.accepted, 'trajectory admitted', handle.accepted, True)
        self.peer.position.z_reset_counter += 1
        self.peer.position.z -= 20.0
        response = self.peer.outcome(handle, FAULT_REACTION_SEC)
        self.peer.check(
            not response.result.success,
            'reset trajectory fails closed',
            response.status,
            'unsuccessful',
        )

    def takeover(self, mode: int) -> None:
        """Require ownership loss before an externally reached point can imply success."""
        self.peer.start()
        handle: ClientGoalHandle = self.navigate()
        self.peer.event('phase', name='external_takeover', navigation_state=mode)
        self.peer.status.nav_state = mode
        self.peer.battery.remaining = 0.03
        self.peer.position.y = 12.0
        response = self.peer.outcome(handle, FAULT_REACTION_SEC)
        self.peer.check(
            not response.result.success,
            'takeover cannot complete application navigation',
            response.status,
            'unsuccessful',
        )
        self.peer.pump(0.2)
        self.peer.check(
            len(self.peer.commands) == 0, 'takeover is never overridden', len(self.peer.commands), 0
        )

    def test_manual_takeover_remains_authoritative(self) -> None:
        """ISS-03: manual takeover terminates application control without false success."""
        self.takeover(VehicleStatus.NAVIGATION_STATE_MANUAL)

    def test_low_battery_land_remains_authoritative(self) -> None:
        """ISS-03/06: a simulated low-battery PX4 Land takeover remains authoritative."""
        self.takeover(VehicleStatus.NAVIGATION_STATE_AUTO_LAND)

    def test_orbit_recovery_cannot_undo_takeover(self) -> None:
        """ISS-06: takeover during Orbit cancellation prestream cannot trigger Offboard recovery."""
        self.peer.start()
        goal = OrbitPoint.Goal()
        goal.frame_id = 'local_enu'
        goal.center_position = Point(x=10.0, y=5.0, z=3.0)
        goal.radius_m, goal.tangential_speed_mps = 3.0, 1.0
        goal.revolutions, goal.timeout_sec = 2.0, 10.0
        handle: ClientGoalHandle = self.peer.send('orbit_point', OrbitPoint, goal)
        self.peer.check(handle.accepted, 'orbit admitted', handle.accepted, True)
        self.peer.wait(
            lambda: self.peer.status.nav_state == VehicleStatus.NAVIGATION_STATE_ORBIT,
            'native Orbit command',
            1.0,
        )
        self.peer.pump(0.2)
        self.peer.resolve(handle.cancel_goal_async(), 'orbit cancel response')
        self.peer.pump(0.2)
        self.peer.status.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.peer.event('phase', name='manual_takeover_during_recovery')
        self.peer.pump(1.5)
        restores: int = sum(
            command.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE and command.param2 == 6.0
            for command in self.peer.commands
        )
        self.peer.check(restores == 0, 'Orbit recovery respects takeover', restores, 0)

    def test_runtime_disable_is_truthful(self) -> None:
        """ISS-10: a successful disable parameter update actually prevents flight commands."""
        self.peer.ground()
        self.peer.start()
        client = self.peer.create_client(SetParameters, 'vehicle_control/set_parameters')
        self.peer.wait(client.service_is_ready, 'parameter service', 2.0)
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name='enable_flight_commands',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False),
            )
        ]
        response = self.peer.resolve(client.call_async(request), 'disable parameter response')
        self.peer.event(
            'parameter_response',
            successful=response.results[0].successful,
            reason=response.results[0].reason,
        )
        if not response.results[0].successful:
            self.peer.check(
                bool(response.results[0].reason),
                'immutable parameter explains refusal',
                response.results[0].reason,
                'nonempty diagnostic',
            )
            return
        goal = Takeoff.Goal()
        goal.altitude_m = 3.0
        handle: ClientGoalHandle = self.peer.send('takeoff', Takeoff, goal, internal=True)
        self.peer.check(
            not handle.accepted, 'successful disable prevents takeoff', handle.accepted, False
        )

    def test_finite_overflow_position_is_rejected(self) -> None:
        """ISS-08: finite float64 values that cannot fit PX4 float32 are rejected."""
        self.peer.start()
        goal: NavigateTo.Goal = self.navigation()
        goal.target_position.x = 1e300
        handle: ClientGoalHandle = self.peer.send('navigate_to', NavigateTo, goal, internal=True)
        self.peer.check(
            not handle.accepted,
            'unrepresentable position rejected before publication',
            handle.accepted,
            False,
        )

    def test_unrepresentable_timeout_is_rejected(self) -> None:
        """ISS-11: a finite duration outside steady-clock range is rejected before conversion."""
        self.peer.start()
        goal: NavigateTo.Goal = self.navigation(timeout_sec=3e38)
        handle: ClientGoalHandle = self.peer.send('navigate_to', NavigateTo, goal, internal=True)
        self.peer.check(
            not handle.accepted, 'unrepresentable timeout rejected', handle.accepted, False
        )

    def test_impossible_trajectory_is_rejected(self) -> None:
        """ISS-08/16: excessive segment speed never becomes an active trajectory."""
        self.peer.start()
        goal: FollowTrajectory.Goal = self.trajectory()
        goal.points[1].position.x = 25.0
        handle: ClientGoalHandle = self.peer.send(
            'follow_trajectory', FollowTrajectory, goal, internal=True
        )
        self.peer.check(
            not handle.accepted, 'unflyable trajectory rejected', handle.accepted, False
        )

    def test_missed_route_cannot_succeed_at_final_point(self) -> None:
        """ISS-17: remaining at a route's final/start position does not prove path execution."""
        self.peer.start()
        handle: ClientGoalHandle = self.peer.send(
            'follow_trajectory', FollowTrajectory, self.trajectory()
        )
        self.peer.check(handle.accepted, 'feasible trajectory admitted', handle.accepted, True)
        response = self.peer.outcome(handle, 4.0)
        self.peer.check(
            not response.result.success,
            'missed intermediate route cannot succeed',
            response.status,
            'unsuccessful',
        )

    def test_invalid_later_mission_stage_never_arms(self) -> None:
        """ISS-09: invalid Orbit geometry is rejected before a mission's takeoff."""
        self.peer.ground()
        self.peer.follow_setpoints = True
        self.peer.start()
        goal = ExecuteWaypointMission.Goal()
        goal.takeoff_altitude_m = 3.0
        goal.orbit_after_waypoints = True
        goal.orbit_center_offset = Point(x=4.0, y=0.0, z=3.0)
        goal.orbit_radius_m, goal.orbit_tangential_speed_mps = 21.0, 1.0
        goal.orbit_revolutions, goal.orbit_timeout_sec = 1.0, 10.0
        handle: ClientGoalHandle = self.peer.send(
            'execute_waypoint_mission', ExecuteWaypointMission, goal
        )
        if handle.accepted:
            self.peer.outcome(handle, 4.0)
        self.no_arming()

    def acknowledgement_failure(self, wrong_destination: bool = False) -> None:
        self.peer.ground()
        self.peer.apply_commands = False
        if wrong_destination:
            self.peer.ack_target_system = 42
        else:
            self.peer.fault('ack', LinkFault(paused=True))
        self.peer.start()
        response = self.peer.outcome(self.takeoff(), 4.0)
        self.peer.check(
            not response.result.success,
            'unconfirmed transition cannot succeed',
            response.status,
            'unsuccessful',
        )
        self.no_arming()
        self.peer.check(
            len(self.peer.commands) <= 3,
            'command retries are bounded',
            len(self.peer.commands),
            '<= 3',
        )
        self.peer.check(
            'acknowledge' in response.result.message,
            'ACK failure identifies failed stage',
            response.result.message,
            'acknowledgement diagnostic',
        )

    def test_complete_ack_loss_is_bounded(self) -> None:
        """ISS-15/19: complete ACK loss produces bounded failure without arming."""
        self.acknowledgement_failure()

    def test_ack_for_another_system_is_ignored(self) -> None:
        """ISS-15: an ACK addressed to another requester cannot authorize arming."""
        self.acknowledgement_failure(wrong_destination=True)

    def test_unrelated_ack_noise_preserves_expected_ack(self) -> None:
        """ISS-15: unrelated ACK traffic cannot erase an already received transition ACK."""
        self.peer.ground()
        self.peer.follow_setpoints = True
        self.peer.ack_noise = True
        self.peer.start()
        response = self.peer.outcome(self.takeoff(), 4.0)
        self.peer.check(
            response.result.success,
            'valid ACK survives unrelated ACK traffic',
            response.result.message,
            'successful confirmed takeoff',
        )

    def test_preflight_denial_never_arms(self) -> None:
        """PX4 preflight: command denial is propagated without force-arming or bypass."""
        self.peer.ground()
        self.peer.command_results[VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM] = (
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED
        )
        self.peer.status.pre_flight_checks_pass = False
        self.peer.start()
        response = self.peer.outcome(self.takeoff(), 4.0)
        self.peer.check(
            not response.result.success,
            'denied command cannot succeed',
            response.status,
            'unsuccessful',
        )
        self.peer.check(
            self.peer.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED,
            'preflight denial leaves the vehicle disarmed',
            self.peer.status.arming_state,
            VehicleStatus.ARMING_STATE_DISARMED,
        )
        force_requests: int = sum(
            command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            and command.param2 != 0.0
            for command in self.peer.commands
        )
        self.peer.check(
            force_requests == 0,
            'preflight rejection never triggers force arming',
            force_requests,
            0,
        )
        self.peer.check(
            'denied' in response.result.message,
            'PX4 denial survives error propagation',
            response.result.message,
            'denied',
        )

    def test_cancel_during_prestream_never_arms(self) -> None:
        """ISS-05/19: cancellation before Offboard entry never proceeds to arming."""
        self.peer.ground()
        self.peer.start()
        handle: ClientGoalHandle = self.takeoff()
        self.peer.wait(lambda: self.peer.counts['heartbeat'] > 0, 'takeoff prestream begins', 0.7)
        response = self.peer.cancel(handle)
        self.peer.check(
            response.status == GoalStatus.STATUS_CANCELED,
            'prestream cancellation terminates',
            response.status,
            GoalStatus.STATUS_CANCELED,
        )
        self.peer.pump(0.3)
        self.no_arming()

    def test_cancel_and_land_race_serializes(self) -> None:
        """ISS-05/12: simultaneous navigation cancellation and landing end in terminal landing."""
        self.peer.start()
        navigation: ClientGoalHandle = self.navigate()
        cancel_future = navigation.cancel_goal_async()
        landing_future = self.peer.client('land', Land).send_goal_async(Land.Goal())
        self.peer.resolve(cancel_future, 'racing cancellation')
        landing: ClientGoalHandle = self.peer.resolve(landing_future, 'racing landing acceptance')
        self.peer.check(landing.accepted, 'priority landing admitted', landing.accepted, True)
        response = self.peer.outcome(landing, 5.0)
        self.peer.check(
            response.result.success,
            'priority landing completes',
            response.result.message,
            'success',
        )
        response = self.peer.outcome(navigation)
        self.peer.check(
            not response.result.success,
            'interrupted navigation never succeeds',
            response.status,
            'unsuccessful',
        )

    def test_concurrent_command_storm_admits_one(self) -> None:
        """ISS-20/26: a simultaneous command storm admits at most one ordinary operation."""
        self.peer.start()
        client = self.peer.client('navigate_to', NavigateTo)
        futures: list[Future[ClientGoalHandle]] = []
        for index in range(COMMAND_STORM_SIZE):
            goal: NavigateTo.Goal = self.navigation()
            self.peer.message('storm_goal', goal)
            futures.append(client.send_goal_async(goal))
            # Drain our own DDS response queue while all long-running goals still overlap.
            self.peer.pump(0.01)
        handles: list[ClientGoalHandle] = [
            self.peer.resolve(future, 'storm goal response') for future in futures
        ]
        accepted: list[ClientGoalHandle] = [handle for handle in handles if handle.accepted]
        self.peer.check(len(accepted) == 1, 'one ordinary authority under storm', len(accepted), 1)
        self.peer.cancel(accepted[0])

    def test_owner_crash_releases_heartbeat(self) -> None:
        """ISS-02: loss of mission authority cannot leave an indefinite healthy heartbeat."""
        self.peer.start()
        self.navigate()
        self.peer.stop_process('mission_execution')
        self.peer.pump(2.0)
        before: int = self.peer.counts['heartbeat']
        self.peer.pump(0.4)
        additional: int = self.peer.counts['heartbeat'] - before
        self.peer.check(additional == 0, 'orphaned owner releases heartbeat', additional, 0)

    def test_dead_backend_landing_reports_unknown(self) -> None:
        """ISS-05/21: backend death is bounded and never falsely reports a successful landing."""
        self.peer.start()
        navigation: ClientGoalHandle = self.navigate()
        self.peer.stop_process('vehicle_control')
        landing: ClientGoalHandle = self.peer.send('land', Land, Land.Goal())
        self.peer.check(landing.accepted, 'recovery intent admitted', landing.accepted, True)
        response = self.peer.outcome(landing, 19.0)
        self.peer.check(
            not response.result.success and bool(response.result.message),
            'unknown landing has an unsuccessful diagnostic',
            response.result.message,
            'failure with reason',
        )
        response = self.peer.outcome(navigation)
        self.peer.check(
            not response.result.success,
            'dead backend cannot finish navigation',
            response.status,
            'unsuccessful',
        )

    def test_frozen_backend_is_bounded(self) -> None:
        """ISS-05/21: an unresponsive live process cannot deadlock recovery indefinitely."""
        self.peer.start()
        self.navigate()
        self.peer.stop_process('vehicle_control', signal.SIGSTOP)
        landing: ClientGoalHandle = self.peer.send('land', Land, Land.Goal())
        self.peer.check(
            landing.accepted,
            'landing intent admitted against frozen backend',
            landing.accepted,
            True,
        )
        response = self.peer.outcome(landing, 19.0)
        self.peer.check(
            not response.result.success,
            'frozen backend reports failure',
            response.status,
            'unsuccessful',
        )

    def test_invalid_json_never_arms(self) -> None:
        """ISS-09/20: malformed, oversized, recursive, and non-finite plans never dispatch takeoff."""
        self.peer.ground()
        self.peer.start()
        documents: tuple[str, ...] = (
            '{',
            '[' * 1500,
            'x' * 65_537,
            '{"version":NaN}',
            '{"version":2,"version":2}',
        )
        for document in documents:
            with self.subTest(document_size=len(document)):
                goal = ExecuteFlightPlan.Goal()
                goal.plan_json = document
                handle: ClientGoalHandle = self.peer.send(
                    'execute_flight_plan', ExecuteFlightPlan, goal
                )
                if handle.accepted:
                    response = self.peer.outcome(handle, 2.0)
                    self.peer.check(
                        not response.result.success,
                        'invalid plan fails',
                        response.result.message,
                        'unsuccessful',
                    )
                self.peer.pump(0.1)
                self.no_arming()

    def test_missing_planner_never_arms(self) -> None:
        """ISS-09/24: a missing planner times out without any flight side effect."""
        self.peer.ground()
        self.peer.start(planner=False)
        goal = ExecuteFlightPlan.Goal()
        goal.plan_json = json.dumps({'version': 2, 'missions': []})
        handle: ClientGoalHandle = self.peer.send('execute_flight_plan', ExecuteFlightPlan, goal)
        self.peer.check(handle.accepted, 'plan intent admitted', handle.accepted, True)
        response = self.peer.outcome(handle, 2.0)
        self.peer.check(
            not response.result.success and 'planner' in response.result.message.lower(),
            'missing planner has bounded diagnostic',
            response.result.message,
            'planner failure',
        )
        self.no_arming()

    def test_seeded_loss_jitter_and_disturbance(self) -> None:
        """ISS-14/20: combined loss, delay, reordering, and bounded disturbance cannot fabricate arrival."""
        self.peer.start()
        handle: ClientGoalHandle = self.navigate()
        self.peer.fault(
            'position', LinkFault(drop_probability=0.55, delay_sec=0.08, jitter_sec=0.08)
        )
        self.peer.fault('status', LinkFault(drop_probability=0.25, delay_sec=0.04, jitter_sec=0.04))
        for index in range(30):
            self.peer.position.y = 7.0 + self.peer.random.uniform(-0.3, 0.3)
            self.peer.position.vy = self.peer.random.uniform(-8.0, 8.0)
            self.peer.pump(0.05)
        response = self.peer.outcome(handle, 3.0)
        self.peer.check(
            not response.result.success,
            'unreached destination cannot succeed under compound faults',
            response.status,
            'unsuccessful',
        )
        finite: bool = all(
            math.isfinite(value) for point in self.peer.setpoints for value in point.position
        )
        self.peer.check(finite, 'compound faults preserve finite position targets', finite, True)

    def test_repeated_cancel_cycles_remain_bounded(self) -> None:
        """ISS-12/21/39: repeated action churn completes within deadlines and a coarse memory budget."""
        self.peer.start()
        baseline: dict[str, int] = dict(self.peer.peak_rss_kib)
        for index in range(CHURN_CYCLES):
            self.peer.event('phase', name='churn_cycle', index=index)
            handle: ClientGoalHandle = self.navigate()
            response = self.peer.cancel(handle)
            self.peer.check(
                response.status == GoalStatus.STATUS_CANCELED,
                'navigation cancellation terminates',
                response.status,
                GoalStatus.STATUS_CANCELED,
            )
            self.peer.pump(0.1)
        for name, peak_kib in self.peer.peak_rss_kib.items():
            growth_kib: int = peak_kib - baseline.get(name, peak_kib)
            self.peer.check(
                growth_kib <= MAXIMUM_RSS_GROWTH_KIB,
                name + ' RSS growth stays bounded',
                growth_kib,
                MAXIMUM_RSS_GROWTH_KIB,
            )
