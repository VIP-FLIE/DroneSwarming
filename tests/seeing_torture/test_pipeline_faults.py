# Copyright 2026 DroneSwarming maintainers.
"""Exercise real executor-to-controller actions with faults injected only below the adapter."""

from __future__ import annotations

import math
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from harness import Harness, navigation_goal, resolve, trajectory_goal, wait_for

from swarm_interfaces.action import FollowTrajectory, NavigateTo


class PipelineFaults(unittest.TestCase):
    """Attack the public action path with both C++ nodes running together."""

    def setUp(self) -> None:
        """Connect the real executor to telemetry published by the real PX4 adapter."""
        self.harness = Harness()
        self.addCleanup(self.harness.close)
        self.source = self.harness.vehicle_control()
        self.harness.launch(
            'mission_execution',
            {
                'vehicle_id': 'uav_1',
                'telemetry_timeout_sec': '0.3',
                'starting_position_timeout_sec': '0.5',
                'action_server_timeout_sec': '0.8',
            },
        )
        self.client = self.harness.client(NavigateTo, 'mission_execution/navigate_to')
        time.sleep(0.4)

    def test_relative_body_pipeline_positive_control(self) -> None:
        """A forward command at PX4 north heading must produce a northward NED target."""
        goal = navigation_goal()
        goal.frame_id = 'relative_body'
        goal.target_position = Point(x=2.0)
        handle = self.harness.send(self.client, goal)
        self.assertTrue(handle.accepted)
        wait_for(
            lambda: any(
                abs(point.position[0] - 2.0) < 0.01 and abs(point.position[1]) < 0.01
                for point in self.harness.setpoints
            )
        )
        with self.harness.lock:
            self.source.local.x = 2.0
        result = self.harness.result(handle)
        self.assertTrue(result.result.success, result.result.message)
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)

    def test_public_finite_double_cannot_escape_as_infinite_setpoint(self) -> None:
        """The complete public command path must prevent double-to-float overflow."""
        handle = self.harness.send(self.client, navigation_goal(east_m=1e300))
        if handle.accepted:
            self.harness.result(handle)
        self.assertTrue(
            all(
                math.isfinite(value) for point in self.harness.setpoints for value in point.position
            ),
            'Infinite PX4 position escaped through both real command boundaries',
        )

    def test_raw_position_dropout_propagates_public_failure(self) -> None:
        """PX4 position loss must terminate the real child and public parent action."""
        handle = self.harness.send(self.client, navigation_goal())
        self.assertTrue(handle.accepted)
        wait_for(lambda: any(point.position[1] == 3.0 for point in self.harness.setpoints))
        self.source.publish_position = False
        result = self.harness.result(handle)
        self.assertFalse(result.result.success)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)

    def test_cancellation_waits_for_settled_velocity(self) -> None:
        """Public cancellation must wait for the real controller's settled-hold boundary."""
        with self.harness.lock:
            self.source.local.vx = 2.0
        handle = self.harness.send(self.client, navigation_goal())
        self.assertTrue(handle.accepted)
        wait_for(lambda: any(point.position[1] == 3.0 for point in self.harness.setpoints))
        resolve(handle.cancel_goal_async())
        result_future = handle.get_result_async()
        time.sleep(0.3)
        self.assertFalse(
            result_future.done(), 'Cancellation completed while horizontal speed was 2 m/s'
        )
        with self.harness.lock:
            self.source.local.vx = 0.0
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(result.result.success)

    def test_trajectory_frame_reset_propagates_public_failure(self) -> None:
        """An estimator revision change must abort the public estimator-local trajectory."""
        client = self.harness.client(FollowTrajectory, 'mission_execution/follow_trajectory')
        handle = self.harness.send(client, trajectory_goal())
        self.assertTrue(handle.accepted)
        time.sleep(0.2)
        with self.harness.lock:
            self.source.local.xy_reset_counter += 1
            self.source.local.y = 10.0
        future = handle.get_result_async()
        wait_for(future.done, 0.8)
        result = self.harness.result(handle)
        self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.result.success)
