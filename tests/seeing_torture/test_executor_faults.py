# Copyright 2026 DroneSwarming maintainers.
"""Exploit validation gaps and asynchronous child-action authority transitions."""

from __future__ import annotations

import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from harness import ControlledAction, Harness, navigation_goal, resolve, resolve_batch, wait_for

from swarm_interfaces.action import (
    ExecuteFlightPlan,
    ExecuteWaypointMission,
    FlyMission,
    Land,
    NavigateTo,
    OrbitPoint,
    Takeoff,
)
from swarm_interfaces.msg import GeographicPosition
from swarm_interfaces.srv import PlanFlight


def waypoint_goal() -> ExecuteWaypointMission.Goal:
    """Provide one geographic waypoint within both execution envelopes."""
    return ExecuteWaypointMission.Goal(
        frame_id='wgs84',
        takeoff_altitude_m=3.0,
        position_tolerance_m=0.5,
        waypoint_timeout_sec=1.0,
        waypoint_positions=[
            GeographicPosition(
                latitude_deg=47.0,
                longitude_deg=8.0,
                altitude_amsl_m=503.0,
            )
        ],
    )


class ExecutorFaults(unittest.TestCase):
    """Run the real mission executor against faulty action and planning peers."""

    def setUp(self) -> None:
        """Create independent faulty peers and a fresh executor process per case."""
        self.harness = Harness()
        self.addCleanup(self.harness.close)

    def test_child_success_positive_control(self) -> None:
        """A consistent child terminal result must reach the public caller."""
        backend = ControlledAction(self.harness, Takeoff, 'takeoff')
        self.harness.mission_execution()
        client = self.harness.client(Takeoff, 'mission_execution/takeoff')
        handle = self.harness.send(client, Takeoff.Goal(altitude_m=3.0))
        self.assertTrue(handle.accepted)
        result = self.harness.result(handle)
        self.assertTrue(result.result.success, result.result.message)
        self.assertEqual(len(backend.received), 1)

    def test_contradictory_child_success_is_failure(self) -> None:
        """A SUCCEEDED status with success=false must not become public success."""
        backend = ControlledAction(self.harness, Takeoff, 'takeoff')
        backend.success = False
        self.harness.mission_execution()
        client = self.harness.client(Takeoff, 'mission_execution/takeoff')
        handle = self.harness.send(client, Takeoff.Goal(altitude_m=3.0))
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)

    def test_aborted_child_with_true_success_is_failure(self) -> None:
        """An ABORTED status with success=true must not become public success."""
        backend = ControlledAction(self.harness, Takeoff, 'takeoff')
        backend.status = 'abort'
        self.harness.mission_execution()
        client = self.harness.client(Takeoff, 'mission_execution/takeoff')
        handle = self.harness.send(client, Takeoff.Goal(altitude_m=3.0))
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)

    def test_goal_flood_dispatches_only_one_child(self) -> None:
        """Concurrent public requests must preserve one command-authority reservation."""
        backend = ControlledAction(self.harness, NavigateTo, 'navigate_to')
        backend.hold = True
        self.harness.mission_execution(airborne=True)
        client = self.harness.client(NavigateTo, 'mission_execution/navigate_to')
        futures = [client.send_goal_async(navigation_goal()) for _ in range(48)]
        handles = resolve_batch(futures)
        self.assertEqual(sum(handle.accepted for handle in handles), 1)
        wait_for(backend.started.is_set)
        self.assertEqual(len(backend.received), 1)
        backend.release.set()
        accepted = next(handle for handle in handles if handle.accepted)
        self.assertTrue(self.harness.result(accepted).result.success)

    def test_late_acceptance_is_canceled_before_next_dispatch(self) -> None:
        """Timed-out goal responses must retain ownership until the late child terminates."""
        backend = ControlledAction(self.harness, NavigateTo, 'navigate_to')
        backend.response_delay_sec = 1.2
        backend.hold = True
        self.harness.mission_execution(airborne=True)
        client = self.harness.client(NavigateTo, 'mission_execution/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        second = self.harness.send(client, navigation_goal())
        if second.accepted:
            self.assertFalse(self.harness.result(second).result.success)
        self.assertEqual(len(backend.received), 1)
        wait_for(lambda: backend.cancellations == 1)
        backend.response_delay_sec = 0.0
        backend.hold = False
        time.sleep(0.2)
        recovered = self.harness.send(client, navigation_goal())
        self.assertTrue(recovered.accepted)
        self.assertTrue(self.harness.result(recovered).result.success)

    def test_parent_cancellation_waits_for_child_terminal(self) -> None:
        """A public canceled result requires the active child's canceled terminal status."""
        backend = ControlledAction(self.harness, NavigateTo, 'navigate_to')
        backend.hold = True
        self.harness.mission_execution(airborne=True)
        client = self.harness.client(NavigateTo, 'mission_execution/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        wait_for(backend.started.is_set)
        resolve(handle.cancel_goal_async())
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(backend.cancellations, 1)
        self.assertFalse(result.result.success)

    def test_uncooperative_child_blocks_landing_dispatch(self) -> None:
        """A child that refuses cancellation must not overlap with a landing child."""
        backend = ControlledAction(self.harness, NavigateTo, 'navigate_to')
        backend.hold = True
        backend.ignore_cancel = True
        landing = ControlledAction(self.harness, Land, 'land')
        self.harness.mission_execution(airborne=True)
        client = self.harness.client(NavigateTo, 'mission_execution/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        wait_for(backend.started.is_set)
        land_client = self.harness.client(Land, 'mission_execution/land')
        land_handle = self.harness.send(land_client, Land.Goal(timeout_sec=1.0))
        self.assertTrue(land_handle.accepted)
        result = self.harness.result(handle, 17.0)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        landing_result = self.harness.result(land_handle)
        self.assertFalse(landing_result.result.success)
        self.assertEqual(len(landing.received), 0)
        self.assertGreaterEqual(backend.cancellations, 1)

    def test_telemetry_loss_during_child_prevents_success(self) -> None:
        """The execution boundary must not report success with expired vehicle state."""
        backend = ControlledAction(self.harness, NavigateTo, 'navigate_to')
        backend.hold = True
        source = self.harness.mission_execution(airborne=True)
        client = self.harness.client(NavigateTo, 'mission_execution/navigate_to')
        handle = self.harness.send(client, navigation_goal())
        wait_for(backend.started.is_set)
        source.enabled = False
        time.sleep(0.5)
        backend.release.set()
        result = self.harness.result(handle)
        self.assertFalse(
            result.result.success, 'run_child_action checked state only before dispatch'
        )

    def test_invalid_orbit_is_rejected_before_takeoff_dispatch(self) -> None:
        """A downstream-invalid orbit must fail preflight before uploading a takeoff route."""
        backend = ControlledAction(self.harness, FlyMission, 'fly_mission')
        orbit = ControlledAction(self.harness, OrbitPoint, 'orbit_point')
        orbit.reject = True
        self.harness.mission_execution()
        client = self.harness.client(ExecuteWaypointMission, 'execute_waypoint_mission')
        goal = waypoint_goal()
        goal.orbit_after_waypoints = True
        goal.orbit_center_geographic = goal.waypoint_positions[0]
        goal.orbit_radius_m = 21.0
        goal.orbit_tangential_speed_mps = 1.0
        goal.orbit_revolutions = 1.0
        goal.orbit_timeout_sec = 1.0
        handle = self.harness.send(client, goal)
        if handle.accepted:
            self.harness.result(handle)
        self.assertEqual(
            len(backend.received),
            0,
            'Executor dispatched takeoff before validating the orbit radius limit',
        )

    def test_incomplete_child_route_cannot_report_mission_success(self) -> None:
        """A one-waypoint mission requires one completed waypoint in the child result."""
        backend = ControlledAction(self.harness, FlyMission, 'fly_mission')
        backend.completed_waypoints = 0
        self.harness.mission_execution()
        client = self.harness.client(ExecuteWaypointMission, 'execute_waypoint_mission')
        handle = self.harness.send(client, waypoint_goal())
        result = self.harness.result(handle)
        self.assertFalse(
            result.result.success, 'Succeeded after child reported zero of one waypoints'
        )

    def test_impossible_child_progress_cannot_report_mission_success(self) -> None:
        """A child cannot complete more waypoints than the uploaded route contains."""
        backend = ControlledAction(self.harness, FlyMission, 'fly_mission')
        backend.completed_waypoints = 0xFFFFFFFF
        self.harness.mission_execution()
        client = self.harness.client(ExecuteWaypointMission, 'execute_waypoint_mission')
        handle = self.harness.send(client, waypoint_goal())
        result = self.harness.result(handle)
        self.assertFalse(result.result.success, 'Unvalidated UINT32_MAX progress became success')

    def test_planner_out_of_bounds_response_never_dispatches(self) -> None:
        """A compromised planner cannot bypass the executor's geographic envelope."""
        backend = ControlledAction(self.harness, FlyMission, 'fly_mission')

        def malicious_plan(
            _request: PlanFlight.Request, response: PlanFlight.Response
        ) -> PlanFlight.Response:
            response.success = True
            response.takeoff_altitude_m = 3.0
            response.position_tolerance_m = 0.5
            response.waypoint_positions = [
                GeographicPosition(
                    latitude_deg=-47.0,
                    longitude_deg=-172.0,
                    altitude_amsl_m=503.0,
                )
            ]
            return response

        service = self.harness.node.create_service(
            PlanFlight,
            'mission_planner/plan_flight',
            malicious_plan,
            callback_group=self.harness.group,
        )
        self.addCleanup(self.harness.node.destroy_service, service)
        self.harness.mission_execution()
        client = self.harness.client(ExecuteFlightPlan, 'mission_execution/execute_flight_plan')
        handle = self.harness.send(client, ExecuteFlightPlan.Goal(plan_json='{}'))
        result = self.harness.result(handle)
        self.assertFalse(result.result.success)
        self.assertEqual(len(backend.received), 0)

    def test_relative_body_requires_valid_yaw_before_dispatch(self) -> None:
        """Finite body offsets must not be rotated using an invalid captured heading."""
        backend = ControlledAction(self.harness, NavigateTo, 'navigate_to')
        source = self.harness.mission_execution(airborne=True)
        source.state.yaw_valid = False
        time.sleep(0.1)
        client = self.harness.client(NavigateTo, 'mission_execution/navigate_to')
        goal = navigation_goal()
        goal.frame_id = 'relative_body'
        goal.target_position = Point(x=3.0)
        handle = self.harness.send(client, goal)
        if handle.accepted:
            self.assertFalse(self.harness.result(handle).result.success)
        self.assertEqual(len(backend.received), 0)
