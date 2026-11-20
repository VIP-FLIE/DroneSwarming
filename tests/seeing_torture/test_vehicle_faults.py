# Copyright 2026 DroneSwarming maintainers.
"""Attack implementation-specific telemetry, numeric conversion, and control-state paths."""

from __future__ import annotations

import math
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from harness import Harness, navigation_goal, resolve, resolve_batch, trajectory_goal, wait_for
from px4_msgs.msg import VehicleCommand, VehicleStatus

from swarm_interfaces.action import FollowTrajectory, NavigateTo, OrbitPoint, Takeoff


class VehicleFaults(unittest.TestCase):
    """Exercise the real vehicle-control process using synthetic PX4 streams."""

    def setUp(self) -> None:
        """Create a fresh ROS injector and register bounded process cleanup."""
        self.harness = Harness()
        self.addCleanup(self.harness.close)

    def test_navigation_positive_control(self) -> None:
        """A finite nearby target must reach the correct NED wire coordinates."""
        source = self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertTrue(handle.accepted)
        wait_for(
            lambda: any(abs(point.position[1] - 3.0) < 0.01 for point in self.harness.setpoints)
        )
        with self.harness.lock:
            source.local.y = 3.0
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.result.success)
        self.harness.assert_alive()

    def test_disabled_commands_emit_no_setpoints(self) -> None:
        """The default command gate must also suppress unsolicited Offboard output."""
        self.harness.vehicle_control(enabled=False)
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertFalse(handle.accepted)
        time.sleep(0.2)
        self.assertEqual(
            len(self.harness.setpoints),
            0,
            'publish_control_setpoint bypassed enable_flight_commands',
        )

    def test_restart_in_offboard_does_not_command_origin(self) -> None:
        """An Offboard process restart must not send a zero-initialized target."""
        self.harness.vehicle_control()
        wait_for(lambda: bool(self.harness.setpoints))
        positions: list[list[float]] = [list(point.position) for point in self.harness.setpoints]
        self.assertTrue(
            all(math.dist(point, [0.0, 0.0, -3.0]) <= 0.01 for point in positions),
            f'Uncommanded startup targets instead of a current-position hold: {positions}',
        )

    def test_finite_double_target_cannot_become_infinite_px4_float(self) -> None:
        """Finite Point doubles must be rejected before float32 setpoint overflow."""
        self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal(east_m=1e300))
        if handle.accepted:
            self.harness.result(handle)
        self.assertTrue(
            all(
                math.isfinite(value) for point in self.harness.setpoints for value in point.position
            ),
            'Finite 1e300 goal produced a non-finite position command',
        )
        self.assertFalse(handle.accepted, 'Unrepresentable float32 navigation target was accepted')

    def test_local_horizontal_target_obeys_geographic_envelope(self) -> None:
        """Selecting local_enu must not bypass the 1000-meter navigation envelope."""
        self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal(east_m=1001.0))
        self.assertFalse(
            handle.accepted, 'local_navigation_goal returned local input without bounds'
        )

    def test_local_vertical_target_obeys_geographic_envelope(self) -> None:
        """Selecting local_enu must not bypass the 120-meter height-change envelope."""
        self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal(up_m=124.0))
        self.assertFalse(handle.accepted, '121-meter local height change was accepted')

    def test_navigation_aborts_on_mode_exit(self) -> None:
        """External mode selection must end navigation without reporting arrival success."""
        source = self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertTrue(handle.accepted)
        wait_for(lambda: any(point.position[1] == 3.0 for point in self.harness.setpoints))
        with self.harness.lock:
            source.status.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        time.sleep(0.2)
        with self.harness.lock:
            source.local.y = 3.0
        result = self.harness.result(handle)
        self.assertFalse(result.result.success, 'Navigation succeeded after PX4 left Offboard')
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)

    def test_navigation_aborts_on_inflight_failsafe(self) -> None:
        """Failsafe activation after acceptance must prevent navigation success."""
        source = self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertTrue(handle.accepted)
        wait_for(lambda: any(point.position[1] == 3.0 for point in self.harness.setpoints))
        with self.harness.lock:
            source.status.failsafe = True
        time.sleep(0.15)
        with self.harness.lock:
            source.local.y = 3.0
        result = self.harness.result(handle)
        self.assertFalse(result.result.success, 'Active navigation ignored the PX4 failsafe flag')

    def test_frame_reset_aborts_trajectory(self) -> None:
        """A local estimator reset must invalidate an active estimator-local trajectory."""
        source = self.harness.vehicle_control()
        client = self.harness.client(FollowTrajectory, 'vehicle_control/follow_trajectory')
        handle = self.harness.send(client, trajectory_goal())
        self.assertTrue(handle.accepted)
        time.sleep(0.15)
        old_revision: int = self.harness.states[-1].local_frame_revision
        with self.harness.lock:
            source.local.xy_reset_counter += 1
            source.local.y = 10.0
        wait_for(lambda: self.harness.states[-1].local_frame_revision > old_revision)
        future = handle.get_result_async()
        wait_for(future.done, 0.7)
        result = resolve(future)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)

    def test_frame_reset_aborts_navigation_and_recaptures_hold(self) -> None:
        """Navigation must invalidate its target and hold the post-reset coordinates."""
        source = self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertTrue(handle.accepted)
        wait_for(lambda: any(point.position[1] == 3.0 for point in self.harness.setpoints))
        with self.harness.lock:
            source.local.xy_reset_counter += 1
            source.local.y = 10.0
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        wait_for(lambda: self.harness.setpoints[-1].position[1] == 10.0)

    def test_position_dropout_aborts_and_releases_authority(self) -> None:
        """Position stream loss must terminate navigation and permit a recovered command."""
        source = self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertTrue(handle.accepted)
        with self.harness.lock:
            source.publish_position = False
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        wait_for(lambda: not self.harness.states[-1].position_valid)
        source.publish_position = True
        wait_for(lambda: self.harness.states[-1].position_valid)
        recovered = self.harness.send(client, navigation_goal(east_m=0.0))
        self.assertTrue(recovered.accepted)
        self.assertTrue(self.harness.result(recovered).result.success)

    def test_status_dropout_aborts_navigation(self) -> None:
        """Fresh position alone must not conceal a missing PX4 status stream."""
        source = self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertTrue(handle.accepted)
        source.publish_status = False
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)

    def test_initial_failsafe_rejects_navigation(self) -> None:
        """The admission check must reject navigation while PX4 already reports failsafe."""
        source = self.harness.vehicle_control()
        source.status.failsafe = True
        time.sleep(0.1)
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        self.assertFalse(self.harness.send(client, navigation_goal()).accepted)

    def test_nonfinite_goal_flood_preserves_valid_recovery(self) -> None:
        """Invalid IEEE-754 requests must neither reserve authority nor kill the node."""
        self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        futures = [
            client.send_goal_async(navigation_goal(east_m=value))
            for value in [math.nan, math.inf, -math.inf] * 16
        ]
        self.assertTrue(all(not handle.accepted for handle in resolve_batch(futures)))
        self.harness.assert_alive()
        handle = self.harness.send(client, navigation_goal(east_m=0.0))
        self.assertTrue(handle.accepted)
        self.assertTrue(self.harness.result(handle).result.success)

    def test_takeoff_missing_acknowledgements_are_bounded(self) -> None:
        """A blackholed ACK stream must use at most three transition attempts."""
        source = self.harness.vehicle_control()
        with self.harness.lock:
            source.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
            source.status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            source.land.landed = True
            source.local.z = 0.0
            source.acknowledge_commands = False
        wait_for(lambda: not self.harness.states[-1].armed and self.harness.states[-1].landed)
        client = self.harness.client(Takeoff, 'vehicle_control/takeoff')
        handle = self.harness.send(client, Takeoff.Goal(altitude_m=3.0))
        self.assertTrue(handle.accepted)
        result = self.harness.result(handle, 4.0)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            len(
                [
                    command
                    for command in self.harness.commands
                    if command.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                ]
            ),
            3,
        )
        self.assertFalse(
            any(
                command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                for command in self.harness.commands
            )
        )

    def test_takeoff_rejects_foreign_acknowledgements(self) -> None:
        """ACKs addressed to another companion must not authorize arming."""
        source = self.harness.vehicle_control()
        with self.harness.lock:
            source.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
            source.status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            source.land.landed = True
            source.local.z = 0.0
            source.acknowledgement_target = 254
        wait_for(lambda: not self.harness.states[-1].armed and self.harness.states[-1].landed)
        client = self.harness.client(Takeoff, 'vehicle_control/takeoff')
        handle = self.harness.send(client, Takeoff.Goal(altitude_m=3.0))
        self.assertTrue(handle.accepted)
        result = self.harness.result(handle, 4.0)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(
            any(
                command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                for command in self.harness.commands
            )
        )

    def test_nonfinite_telemetry_with_valid_flags_aborts_navigation(self) -> None:
        """PX4 validity flags cannot authorize a NaN position during navigation."""
        source = self.harness.vehicle_control()
        client = self.harness.client(NavigateTo, 'vehicle_control/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        self.assertTrue(handle.accepted)
        with self.harness.lock:
            source.local.x = math.nan
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertTrue(
            all(
                math.isfinite(value) for point in self.harness.setpoints for value in point.position
            )
        )

    def test_heading_validity_loss_aborts_yaw_trajectory(self) -> None:
        """Heading loss must abort an action that owns yaw control."""
        source = self.harness.vehicle_control()
        client = self.harness.client(FollowTrajectory, 'vehicle_control/follow_trajectory')
        goal = trajectory_goal()
        goal.control_yaw = True
        for point in goal.points:
            point.yaw_rad = math.pi / 2.0
        handle = self.harness.send(client, goal)
        self.assertTrue(handle.accepted)
        time.sleep(0.1)
        source.local.heading_good_for_control = False
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)

    def test_orbit_ack_blackhole_does_not_repeat_command(self) -> None:
        """A missing Orbit ACK must not cause repeated native orbit commands."""
        source = self.harness.vehicle_control()
        source.acknowledge_commands = False
        client = self.harness.client(OrbitPoint, 'vehicle_control/orbit_point')
        goal = OrbitPoint.Goal(
            frame_id='local_enu',
            center_position=Point(z=3.0),
            radius_m=2.0,
            tangential_speed_mps=1.0,
            revolutions=1.0,
            timeout_sec=3.0,
        )
        handle = self.harness.send(client, goal)
        self.assertTrue(handle.accepted)
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            sum(
                command.command == VehicleCommand.VEHICLE_CMD_DO_ORBIT
                for command in self.harness.commands
            ),
            1,
        )

    def test_orbit_frame_reset_aborts_and_recovers_hold(self) -> None:
        """Native Orbit must invalidate a changed estimator reference before progress succeeds."""
        source = self.harness.vehicle_control()
        client = self.harness.client(OrbitPoint, 'vehicle_control/orbit_point')
        goal = OrbitPoint.Goal(
            frame_id='local_enu',
            center_position=Point(z=3.0),
            radius_m=2.0,
            tangential_speed_mps=1.0,
            revolutions=1.0,
            timeout_sec=3.0,
        )
        handle = self.harness.send(client, goal)
        self.assertTrue(handle.accepted)
        wait_for(lambda: source.status.nav_state == VehicleStatus.NAVIGATION_STATE_ORBIT)
        time.sleep(0.7)
        source.local.xy_reset_counter += 1
        result = self.harness.result(handle, 4.0)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)
        self.assertEqual(source.status.nav_state, VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def test_orbit_subthreshold_teleports_cannot_forge_revolution(self) -> None:
        """Sub-90-degree estimator jumps cannot prove a physical orbit at zero reported velocity."""
        source = self.harness.vehicle_control()
        client = self.harness.client(OrbitPoint, 'vehicle_control/orbit_point')
        goal = OrbitPoint.Goal(
            frame_id='local_enu',
            center_position=Point(z=3.0),
            radius_m=2.0,
            tangential_speed_mps=1.0,
            revolutions=1.0,
            timeout_sec=4.0,
        )
        handle = self.harness.send(client, goal)
        self.assertTrue(handle.accepted)
        wait_for(lambda: source.status.nav_state == VehicleStatus.NAVIGATION_STATE_ORBIT)
        time.sleep(0.7)
        for index in range(10):
            angle_rad: float = index * math.pi / 4.0
            with self.harness.lock:
                source.local.y = 2.0 * math.cos(angle_rad)
                source.local.x = 2.0 * math.sin(angle_rad)
            time.sleep(0.1)
        result = self.harness.result(handle, 5.0)
        self.assertFalse(
            result.result.success,
            'OrbitProgress accepted 45-degree teleports with zero measured velocity',
        )
