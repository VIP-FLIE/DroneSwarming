"""Command admission and adversarial peer tests through public ROS actions."""

from __future__ import annotations

import copy
import json
import math
import threading

from action_msgs.msg import GoalStatus
from swarm_interfaces.action import (
    ExecuteFlightPlan,
    ExecuteWaypointMission,
    Land,
    NavigateTo,
)
from swarm_interfaces.srv import PlanFlight

from support import (
    PUBLIC_ACTIONS,
    Goal,
    VehiclePeer,
    WireCase,
    fleet_plan,
    geographic,
    point,
    starting_state,
    valid_goals,
    valid_translation,
)


class ExecutorFixture(WireCase):
    def boot(self, *, with_peer: bool = True, airborne: bool = True) -> None:
        self.state = starting_state(airborne=airborne)
        if with_peer:
            self.peer = VehiclePeer(self)
        self.launch('mission_execution', vehicle_id='uav_1', action_server_timeout_sec=1.0)
        self.until(lambda: self.state_publisher.get_subscription_count() > 0, 12.0)
        self.pause()

    def navigation(self) -> NavigateTo.Goal:
        return valid_goals()['navigate_to']

    def begin_navigation(self) -> None:
        self.peer.mode = 'hold'
        self.handle = self.send('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.assertTrue(self.handle.accepted)
        self.until(lambda: len(self.peer.received) == 1)


class ExecutorTorture(ExecutorFixture):
    def numeric_goal_probes(self) -> list[tuple[str, Goal]]:
        """Generate invalid deadline and climb probes independently of state."""
        cases: list[tuple[str, Goal]] = []
        for value in [math.nan, math.inf, -math.inf, -1.0, 3601.0]:
            for operation in [
                'navigate_to',
                'follow_trajectory',
                'orbit_point',
                'land',
            ]:
                goal = valid_goals()[operation]
                goal.timeout_sec = value
                cases.append((operation, goal))
        for value in [math.nan, math.inf, -math.inf, 0.0, -1.0, 121.0]:
            goal = valid_goals()['takeoff']
            goal.altitude_m = value
            cases.append(('takeoff', goal))
        return cases

    def test_malformed_goals_never_reach_children(self) -> None:
        self.boot()
        cases = self.numeric_goal_probes()
        for axis in ['x', 'y', 'z']:
            for value in [math.nan, math.inf, -math.inf]:
                goal = valid_goals()['navigate_to']
                setattr(goal.target_position, axis, value)
                cases.append(('navigate_to', goal))
        for frame_id in ['ned', '../../fmu', 'LOCAL_ENU', '🚁']:
            goal = valid_goals()['navigate_to']
            goal.frame_id = frame_id
            cases.append(('navigate_to', goal))
        for points in [[], valid_goals()['follow_trajectory'].points[:1]]:
            goal = valid_goals()['follow_trajectory']
            goal.points = points
            cases.append(('follow_trajectory', goal))
        for first_sec, last_sec in [
            (0.1, 1.0),
            (0.0, 0.0),
            (0.0, -1.0),
            (0.0, math.nan),
            (0.0, math.inf),
        ]:
            goal = valid_goals()['follow_trajectory']
            goal.points[0].time_from_start_sec = first_sec
            goal.points[1].time_from_start_sec = last_sec
            cases.append(('follow_trajectory', goal))
        for field, values in {
            'radius_m': [0.0, 0.9, -1.0, 21.0, math.nan, math.inf],
            'tangential_speed_mps': [0.0, -1.0, 3.1, math.nan, math.inf],
            'revolutions': [0.0, -1.0, math.nan, math.inf],
        }.items():
            for value in values:
                goal = valid_goals()['orbit_point']
                setattr(goal, field, value)
                cases.append(('orbit_point', goal))
        for index, (operation, goal) in enumerate(cases):
            self.state = starting_state(airborne=operation != 'takeoff')
            self.pause(0.04)
            dispatch_count = len(self.peer.received)
            with self.subTest(index=index, operation=operation):
                self.unsuccessful(
                    f'mission_execution/{operation}',
                    PUBLIC_ACTIONS[f'mission_execution/{operation}'],
                    goal,
                )
                self.assertEqual(
                    len(self.peer.received),
                    dispatch_count,
                    'Invalid goal crossed the command boundary',
                )
        self.state = starting_state(airborne=True)
        self.pause()
        handle = self.send('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.assertTrue(handle.accepted)
        status, result = self.terminal(handle)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.success)

    def test_invalid_state_missing_heading_and_stale_stream(self) -> None:
        self.boot()
        for field, value in [
            ('position_valid', False),
            ('armed', False),
            ('landed', True),
            ('offboard', False),
        ]:
            self.state = starting_state(airborne=True)
            setattr(self.state, field, value)
            self.pause()
            with self.subTest(field=field):
                self.unsuccessful('mission_execution/navigate_to', NavigateTo, self.navigation())
                self.assertEqual(self.peer.received, [])
        for value in [math.nan, math.inf, -math.inf]:
            self.state = starting_state(airborne=True)
            self.state.position.x = value
            self.pause()
            self.unsuccessful('mission_execution/navigate_to', NavigateTo, self.navigation())
            self.assertEqual(self.peer.received, [])
        self.state = starting_state(airborne=True)
        self.state.yaw_valid = False
        self.pause()
        goal = self.navigation()
        goal.frame_id = 'relative_body'
        self.unsuccessful('mission_execution/navigate_to', NavigateTo, goal)
        self.assertEqual(self.peer.received, [])
        self.state = starting_state(airborne=True)
        self.pause()
        self.publish_state = False
        self.pause(1.4)
        self.unsuccessful('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.assertEqual(self.peer.received, [])
        self.publish_state = True
        self.pause()
        handle = self.send('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.assertTrue(handle.accepted)
        self.assertTrue(self.terminal(handle)[1].success)

    def test_concurrent_goal_flood_admits_one(self) -> None:
        self.boot()
        self.peer.mode = 'hold'
        client = self.action_client('mission_execution/navigate_to', NavigateTo)
        futures = []
        for index in range(32):
            goal = self.navigation()
            self.trace('goal_flood_request', index=index, goal=goal)
            futures.append(client.send_goal_async(goal))
        handles = []
        for index, future in enumerate(futures):
            handle = self.resolve(future)
            handles.append(handle)
            self.trace('goal_flood_response', index=index, accepted=handle.accepted)
        accepted = [handle for handle in handles if handle.accepted]
        self.trace(
            'admission_flood',
            accepted=len(accepted),
            rejected=len(handles) - len(accepted),
        )
        self.assertEqual(len(accepted), 1)
        self.until(lambda: len(self.peer.received) == 1)
        self.resolve(accepted[0].cancel_goal_async())
        self.assertEqual(self.terminal(accepted[0])[0], GoalStatus.STATUS_CANCELED)
        self.assertEqual(len(self.peer.received), 1)

    def test_cancel_propagates_and_waits_for_child_terminal(self) -> None:
        self.boot()
        self.begin_navigation()
        self.peer.cancel_release.clear()
        result_future = self.handle.get_result_async()
        cancel_response = self.resolve(self.handle.cancel_goal_async())
        self.assertEqual(len(cancel_response.goals_canceling), 1)
        self.until(lambda: ('child', 'cancel_requested') in self.peer.events)
        self.pause()
        self.assertFalse(
            result_future.done(), 'Parent ended before the child confirmed cancellation'
        )
        self.peer.cancel_release.set()
        response = self.resolve(result_future)
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(response.result.success)

    def test_cancellation_failure_retains_child_diagnostic(self) -> None:
        self.boot()
        self.begin_navigation()
        self.peer.mode = 'cancel_abort'
        self.resolve(self.handle.cancel_goal_async())
        status, result = self.terminal(self.handle)
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.success)
        self.assertIn('synthetic child hold failure', result.message)

    def test_landing_preemption_drains_child_and_rejects_duplicates(self) -> None:
        self.boot()
        self.begin_navigation()
        self.peer.cancel_release.clear()
        landing = self.send('mission_execution/land', Land, Land.Goal(timeout_sec=5.0))
        self.assertTrue(landing.accepted)
        self.until(lambda: ('child', 'cancel_requested') in self.peer.events)
        duplicate = self.send('mission_execution/land', Land, Land.Goal(timeout_sec=5.0))
        self.assertFalse(duplicate.accepted)
        self.assertEqual(self.resolve(landing.cancel_goal_async()).goals_canceling, [])
        self.pause()
        self.assertEqual([operation for operation, _ in self.peer.received], ['navigate_to'])
        self.peer.cancel_release.set()
        self.assertEqual(self.terminal(self.handle)[0], GoalStatus.STATUS_ABORTED)
        self.assertTrue(self.terminal(landing)[1].success)
        self.assertLess(
            self.peer.events.index(('navigate_to', 'terminal')),
            self.peer.events.index(('land', 'received')),
        )

    def test_invalid_landing_cannot_interrupt(self) -> None:
        self.boot()
        self.begin_navigation()
        for timeout_sec in [math.nan, math.inf, -1.0, 3601.0]:
            self.unsuccessful('mission_execution/land', Land, Land.Goal(timeout_sec=timeout_sec))
        self.assertNotIn(('child', 'cancel_requested'), self.peer.events)
        self.assertEqual(len(self.peer.received), 1)
        self.resolve(self.handle.cancel_goal_async())
        self.assertEqual(self.terminal(self.handle)[0], GoalStatus.STATUS_CANCELED)

    def test_unavailable_child_is_bounded(self) -> None:
        self.boot(with_peer=False)
        self.unsuccessful('mission_execution/navigate_to', NavigateTo, self.navigation())

    def test_child_rejection_and_failure_do_not_report_success(self) -> None:
        self.boot()
        for mode in ['reject', 'abort']:
            self.peer.mode = mode
            self.unsuccessful('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.peer.mode = 'succeed'
        handle = self.send('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.assertTrue(handle.accepted)
        self.assertTrue(self.terminal(handle)[1].success)

    def test_unconfirmed_child_blocks_further_dispatch(self) -> None:
        self.boot()
        self.begin_navigation()
        self.peer.mode = 'ignore_cancel'
        self.resolve(self.handle.cancel_goal_async())
        self.assertEqual(self.terminal(self.handle, 20.0)[0], GoalStatus.STATUS_ABORTED)
        self.unsuccessful('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.unsuccessful('mission_execution/land', Land, Land.Goal(timeout_sec=5.0))
        self.assertEqual([operation for operation, _ in self.peer.received], ['navigate_to'])
        self.peer.release.set()
        self.until(lambda: ('navigate_to', 'terminal') in self.peer.events)

    def test_late_child_acceptance_is_canceled(self) -> None:
        self.boot()
        self.peer.mode = 'late_accept'
        handle = self.send('mission_execution/navigate_to', NavigateTo, self.navigation())
        self.assertTrue(handle.accepted)
        self.until(lambda: len(self.peer.received) == 1)
        self.assertEqual(self.terminal(handle, 5.0)[0], GoalStatus.STATUS_ABORTED)
        # Release admission while retaining a running operation for cancellation observation.
        self.peer.mode = 'hold'
        self.peer.admission_release.set()
        self.until(lambda: ('child', 'cancel_requested') in self.peer.events)

    def test_waypoint_frame_mix_and_bounds_never_dispatch(self) -> None:
        self.boot(airborne=False)
        baseline = ExecuteWaypointMission.Goal(
            frame_id='relative_enu',
            takeoff_altitude_m=3.0,
            waypoint_offsets=[point(3.0, 0.0, 3.0)],
            position_tolerance_m=0.5,
            waypoint_timeout_sec=30.0,
            land_after_waypoints=True,
        )
        goals = []
        for field, value in [
            ('frame_id', 'ned'),
            ('takeoff_altitude_m', math.nan),
            ('waypoint_timeout_sec', 601.0),
            ('waypoint_offsets', [point()] * 257),
            ('waypoint_positions', [geographic(0.0, 0.0, 103.0)]),
        ]:
            goal = copy.deepcopy(baseline)
            setattr(goal, field, value)
            goals.append(goal)
        for index, goal in enumerate(goals):
            dispatch_count = len(self.peer.received)
            with self.subTest(index=index):
                self.unsuccessful('execute_waypoint_mission', ExecuteWaypointMission, goal)
                self.assertEqual(len(self.peer.received), dispatch_count)

    def test_empty_waypoint_request_has_bounded_consistent_outcome(self) -> None:
        self.boot(airborne=False)
        goal = ExecuteWaypointMission.Goal(
            frame_id='relative_enu',
            takeoff_altitude_m=3.0,
            position_tolerance_m=0.5,
            waypoint_timeout_sec=30.0,
            land_after_waypoints=True,
        )
        handle = self.send('execute_waypoint_mission', ExecuteWaypointMission, goal)
        if handle.accepted:
            status, result = self.terminal(handle)
            self.assertEqual(result.completed_waypoints, 0)
            self.assertFalse(result.orbit_completed)
            self.assertEqual(result.success, status == GoalStatus.STATUS_SUCCEEDED)
        self.assertLessEqual(len(self.peer.received), 1)

    def test_relative_body_navigation_captures_geographic_target(self) -> None:
        self.boot()
        self.state.yaw_rad = math.pi / 2
        self.pause()
        goal = self.navigation()
        goal.frame_id = 'relative_body'
        goal.target_position = point(3.0, 0.0, 0.0)
        handle = self.send('mission_execution/navigate_to', NavigateTo, goal)
        self.assertTrue(handle.accepted)
        self.assertTrue(self.terminal(handle)[1].success)
        self.assertEqual(len(self.peer.received), 1)
        translated = self.peer.received[0][1]
        self.assertEqual(translated.frame_id, 'wgs84')
        self.assertAlmostEqual(
            math.radians(translated.geographic_target.latitude_deg) * 6_371_000.0,
            3.0,
            delta=0.02,
        )
        self.assertAlmostEqual(translated.geographic_target.longitude_deg, 0.0, places=7)
        self.assertAlmostEqual(translated.geographic_target.altitude_amsl_m, 103.0, places=5)


class PlannerPeerTorture(ExecutorFixture):
    def serve(self) -> None:
        self.translation = valid_translation()
        self.planning_received = threading.Event()
        self.planning_release = threading.Event()
        self.planning_release.set()
        self.addCleanup(self.planning_release.set)
        self.service = self.node.create_service(
            PlanFlight,
            'mission_planner/plan_flight',
            self.respond,
            callback_group=self.callback_group,
        )

    def respond(
        self, request: PlanFlight.Request, response: PlanFlight.Response
    ) -> PlanFlight.Response:
        del response
        self.trace('executor_plan_request', request=request)
        self.planning_received.set()
        self.planning_release.wait(timeout=20.0)
        self.trace('synthetic_plan_response', response=self.translation)
        return self.translation

    def test_untrusted_planner_responses_never_dispatch(self) -> None:
        self.serve()
        self.boot(airborne=False)
        invalid = []
        for field, value in [
            ('success', False),
            ('takeoff_altitude_m', math.nan),
            ('takeoff_altitude_m', -1.0),
            ('takeoff_altitude_m', 121.0),
            ('position_tolerance_m', math.nan),
            ('position_tolerance_m', -1.0),
            ('waypoint_positions', []),
            ('waypoint_positions', [geographic(math.nan, 0.0, 103.0)]),
            ('waypoint_positions', [geographic(91.0, 0.0, 103.0)]),
            ('waypoint_positions', [geographic(0.0, 0.0, math.inf)]),
            ('waypoint_positions', [geographic(0.0, 0.0, 103.0)] * 257),
        ]:
            response = valid_translation()
            setattr(response, field, value)
            invalid.append(response)
        for index, response in enumerate(invalid):
            with self.subTest(index=index):
                self.translation = response
                self.unsuccessful(
                    'mission_execution/execute_flight_plan',
                    ExecuteFlightPlan,
                    ExecuteFlightPlan.Goal(plan_json=json.dumps(fleet_plan())),
                )
                self.assertEqual(self.peer.received, [])

    def test_planning_cancel_discards_late_response(self) -> None:
        self.serve()
        self.planning_release.clear()
        self.boot(airborne=False)
        handle = self.send(
            'mission_execution/execute_flight_plan',
            ExecuteFlightPlan,
            ExecuteFlightPlan.Goal(plan_json=json.dumps(fleet_plan())),
        )
        self.assertTrue(handle.accepted)
        self.until(self.planning_received.is_set)
        self.resolve(handle.cancel_goal_async())
        self.assertEqual(self.terminal(handle)[0], GoalStatus.STATUS_CANCELED)
        self.planning_release.set()
        self.pause(1.5)
        self.assertEqual(self.peer.received, [])

    def test_planning_timeout_discards_late_response(self) -> None:
        self.serve()
        self.planning_release.clear()
        self.boot(airborne=False)
        self.unsuccessful(
            'mission_execution/execute_flight_plan',
            ExecuteFlightPlan,
            ExecuteFlightPlan.Goal(plan_json=json.dumps(fleet_plan())),
        )
        self.planning_release.set()
        self.pause(1.5)
        self.assertEqual(self.peer.received, [])

    def test_real_planner_executor_handoff(self) -> None:
        self.boot(airborne=False)
        self.launch('mission_planner', vehicle_id='uav_1')
        self.planner()
        self.canary()
        handle = self.send(
            'mission_execution/execute_flight_plan',
            ExecuteFlightPlan,
            ExecuteFlightPlan.Goal(plan_json=json.dumps(fleet_plan())),
        )
        self.assertTrue(handle.accepted)
        status, result = self.terminal(handle)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(result.success)
        self.assertEqual(result.completed_waypoints, 2)
        self.assertEqual([operation for operation, _ in self.peer.received], ['fly_mission'])

    def test_invalid_state_at_planning_handoff_prevents_dispatch(self) -> None:
        self.serve()
        self.planning_release.clear()
        self.boot(airborne=False)
        handle = self.send(
            'mission_execution/execute_flight_plan',
            ExecuteFlightPlan,
            ExecuteFlightPlan.Goal(plan_json=json.dumps(fleet_plan())),
        )
        self.assertTrue(handle.accepted)
        self.until(self.planning_received.is_set)
        self.state.global_position_valid = False
        self.pause()
        self.planning_release.set()
        status, result = self.terminal(handle)
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.success)
        self.assertEqual(self.peer.received, [])

    def test_local_revision_change_preserves_geographic_route(self) -> None:
        self.serve()
        self.planning_release.clear()
        self.boot(airborne=False)
        handle = self.send(
            'mission_execution/execute_flight_plan',
            ExecuteFlightPlan,
            ExecuteFlightPlan.Goal(plan_json=json.dumps(fleet_plan())),
        )
        self.assertTrue(handle.accepted)
        self.until(self.planning_received.is_set)
        self.state.local_frame_revision += 1
        self.pause()
        self.planning_release.set()
        self.assertTrue(self.terminal(handle)[1].success)
        self.assertEqual(len(self.peer.received), 1)
        self.assertEqual(
            self.peer.received[0][1].waypoint_positions,
            self.translation.waypoint_positions,
        )
