# Copyright 2026 DroneSwarming maintainers.
"""Execute one PX4 SITL flight case and record independent assertions."""

from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
import sys
import tempfile
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from functools import partial
from itertools import pairwise
from pathlib import Path
from typing import Protocol, TextIO, cast

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from flight_test.metrics import (
    cross_track_distance,
    distance_2d,
    distance_3d,
    percentile,
    require_at_least,
    require_at_most,
    wrapped_angle_error,
)
from swarm_interfaces.action import (
    ExecuteWaypointMission,
    FollowTrajectory,
    Land,
    NavigateTo,
    OrbitPoint,
    Takeoff,
)
from swarm_interfaces.msg import TrajectoryPoint, VehicleState

SAMPLE_GAP_LIMIT_SEC: float = 0.5
STATE_FRESHNESS_LIMIT_SEC: float = 1.0
TAKEOFF_ALTITUDE_M: float = 3.0
DEFAULT_ACTION_TIMEOUT_SEC: float = 180.0
AIRBORNE_YAW_TIMEOUT_SEC: float = 15.0
MAX_RECORDED_SAMPLES: int = 12000
FT02_PATH_TOLERANCE_M: float = 1.5
FT02_ENDPOINT_TOLERANCE_M: float = 0.75
FT02_MAX_HORIZONTAL_SPEED_MPS: float = 5.0


def ros_value_to_json(value: object) -> object:
    """Convert a generated ROS message or scalar to JSON-compatible values."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [ros_value_to_json(item) for item in value]
    fields_method: object = getattr(value, 'get_fields_and_field_types', None)
    if callable(fields_method):
        fields: dict[str, str] = cast(dict[str, str], fields_method())
        return {field_name: ros_value_to_json(getattr(value, field_name)) for field_name in fields}
    return str(value)


class ActionResultProtocol(Protocol):
    """Expose fields shared by application action results."""

    success: bool
    message: str


class WrappedActionResultProtocol(Protocol):
    """Expose the terminal status returned by an action client future."""

    status: int
    result: object


@dataclass(frozen=True)
class FlightSample:
    """Store one VehicleState sample with monotonic receipt time."""

    received_monotonic_sec: float
    header_stamp_sec: float
    frame_id: str
    position_enu: tuple[float, float, float]
    velocity_enu: tuple[float, float, float]
    yaw_rad: float
    position_valid: bool
    velocity_valid: bool
    yaw_valid: bool
    armed: bool
    offboard: bool
    landed: bool
    setpoint_publication_count: int


class FlightTestFailure(RuntimeError):
    """Represent an unsafe precondition, failed action, or assertion."""


class FlightTestRunner(Node):
    """Run one bounded flight scenario and record its vehicle telemetry."""

    def __init__(self, case_id: str, artifact_directory: Path) -> None:
        super().__init__('flight_test_runner')
        self.case_id: str = case_id
        self.artifact_directory: Path = artifact_directory
        self.telemetry_file = (artifact_directory / 'vehicle-state.csv').open(
            'w', newline='', encoding='utf-8'
        )
        self.telemetry_writer = csv.writer(self.telemetry_file)
        self.telemetry_writer.writerow(
            (
                'received_monotonic_sec',
                'header_stamp_sec',
                'frame_id',
                'x_m',
                'y_m',
                'z_m',
                'vx_mps',
                'vy_mps',
                'vz_mps',
                'yaw_rad',
                'position_valid',
                'velocity_valid',
                'yaw_valid',
                'armed',
                'offboard',
                'landed',
                'setpoint_publication_count',
            )
        )
        self.command_file = (artifact_directory / 'commanded-trajectory.csv').open(
            'w', newline='', encoding='utf-8'
        )
        self.command_writer = csv.writer(self.command_file)
        self.command_writer.writerow(
            ('case_id', 'point_index', 'time_from_start_sec', 'x_m', 'y_m', 'z_m', 'yaw_rad')
        )
        self.feedback_file = (artifact_directory / 'action-feedback.jsonl').open(
            'w', encoding='utf-8'
        )
        self.samples: deque[FlightSample] = deque(maxlen=MAX_RECORDED_SAMPLES)
        self.telemetry_fault: str | None = None
        self.assertions: list[dict[str, object]] = []
        self.measurements: dict[str, float] = {}
        self.action_events: list[dict[str, object]] = []
        self.recovery_message: str | None = None
        self.case_start_sample_index: int = 0
        self.last_trajectory_started_monotonic_sec: float = 0.0
        self.expect_airborne: bool = False
        self.expect_offboard: bool = True
        self.state_subscription = self.create_subscription(
            VehicleState, '/vehicle_state', self._record_state, 10
        )
        self.takeoff_client = ActionClient(self, Takeoff, '/takeoff')
        self.navigation_client = ActionClient(self, NavigateTo, '/navigate_to')
        self.land_client = ActionClient(self, Land, '/land')
        self.mission_client = ActionClient(
            self, ExecuteWaypointMission, '/execute_waypoint_mission'
        )
        self.trajectory_client = ActionClient(self, FollowTrajectory, '/follow_trajectory')
        self.orbit_client = ActionClient(self, OrbitPoint, '/orbit_point')
        self.case_started_monotonic_sec: float = 0.0
        self.case_active: bool = False

    def _record_state(self, message: VehicleState) -> None:
        """Stream state to CSV and retain a bounded sample window for metrics."""
        received_monotonic_sec: float = time.monotonic()
        stamp_sec: float = float(message.header.stamp.sec) + (
            float(message.header.stamp.nanosec) / 1_000_000_000.0
        )
        position_enu: tuple[float, float, float] = (
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
        )
        velocity_enu: tuple[float, float, float] = (
            float(message.velocity.x),
            float(message.velocity.y),
            float(message.velocity.z),
        )
        sample: FlightSample = FlightSample(
            received_monotonic_sec=received_monotonic_sec,
            header_stamp_sec=stamp_sec,
            frame_id=str(message.header.frame_id),
            position_enu=position_enu,
            velocity_enu=velocity_enu,
            yaw_rad=float(message.yaw_rad),
            position_valid=bool(message.position_valid)
            and all(math.isfinite(value) for value in position_enu),
            velocity_valid=bool(message.velocity_valid)
            and all(math.isfinite(value) for value in velocity_enu),
            yaw_valid=bool(message.yaw_valid) and math.isfinite(float(message.yaw_rad)),
            armed=bool(message.armed),
            offboard=bool(message.offboard),
            landed=bool(message.landed),
            setpoint_publication_count=int(message.setpoint_publication_count),
        )
        if self.case_active and self.samples:
            previous_sample: FlightSample = self.samples[-1]
            if (
                received_monotonic_sec - previous_sample.received_monotonic_sec
                > SAMPLE_GAP_LIMIT_SEC
            ):
                self.telemetry_fault = 'vehicle_state sample gap exceeded 0.5 seconds'
        if (
            self.case_active
            and self.expect_airborne
            and (
                not message.armed
                or message.landed
                or (self.expect_offboard and not message.offboard)
            )
        ):
            self.telemetry_fault = 'vehicle left its required armed flight mode unexpectedly'
        self.samples.append(sample)
        self.telemetry_writer.writerow(
            (
                sample.received_monotonic_sec,
                sample.header_stamp_sec,
                sample.frame_id,
                *sample.position_enu,
                *sample.velocity_enu,
                sample.yaw_rad,
                sample.position_valid,
                sample.velocity_valid,
                sample.yaw_valid,
                sample.armed,
                sample.offboard,
                sample.landed,
                sample.setpoint_publication_count,
            )
        )
        self.telemetry_file.flush()

    def close(self) -> None:
        """Close the streamed telemetry report."""
        self.telemetry_file.close()
        self.command_file.close()
        self.feedback_file.close()

    def _record_action_feedback(self, action_name: str, feedback_message: object) -> None:
        """Write one application action feedback sample to the run artifact."""
        feedback: object = getattr(feedback_message, 'feedback', feedback_message)
        event: dict[str, object] = {
            'action': action_name,
            'received_monotonic_sec': time.monotonic(),
            'feedback': ros_value_to_json(feedback),
        }
        self.feedback_file.write(json.dumps(event) + '\n')
        self.feedback_file.flush()

    def _spin_until_future(self, future: Future, timeout_sec: float, operation: str) -> None:
        """Spin ROS callbacks until a future completes or its deadline expires."""
        deadline: float = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise FlightTestFailure(f'{operation} timed out after {timeout_sec:.1f} seconds')

    def _wait_for_ready(
        self, timeout_sec: float, required_clients: tuple[ActionClient, ...]
    ) -> None:
        """Require fresh landed telemetry and all vehicle actions used by a case."""
        required_node_names: tuple[str, ...] = (
            ('vehicle_control', 'mission_execution')
            if self.case_id == 'FT-01'
            else ('vehicle_control',)
        )
        deadline: float = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.samples:
                continue
            sample: FlightSample = self.samples[-1]
            has_fresh_state: bool = (
                time.monotonic() - sample.received_monotonic_sec <= STATE_FRESHNESS_LIMIT_SEC
                and sample.position_valid
                and sample.frame_id == 'local_enu'
                and sample.landed
                and not sample.armed
            )
            graph_nodes: list[tuple[str, str]] = self.get_node_names_and_namespaces()
            has_expected_nodes: bool = all(
                (node_name, '/') in graph_nodes for node_name in required_node_names
            ) and all(
                sum(1 for graph_name, _ in graph_nodes if graph_name == node_name) == 1
                for node_name in required_node_names
            )
            has_actions: bool = all(client.server_is_ready() for client in required_clients)
            if has_fresh_state and has_expected_nodes and has_actions:
                return
        raise FlightTestFailure(
            'fresh local_enu landed telemetry, the expected root nodes, or a required '
            'action server is unavailable'
        )

    def _current_sample(self, require_yaw: bool = False) -> FlightSample:
        """Return the newest fresh valid state sample for an active flight."""
        if not self.samples:
            raise FlightTestFailure('vehicle state has not been received')
        sample: FlightSample = self.samples[-1]
        sample_age_sec: float = time.monotonic() - sample.received_monotonic_sec
        if sample_age_sec > STATE_FRESHNESS_LIMIT_SEC or not sample.position_valid:
            raise FlightTestFailure(
                'vehicle position telemetry is stale or invalid '
                f'(age={sample_age_sec:.2f} s, limit={STATE_FRESHNESS_LIMIT_SEC:.2f} s, '
                f'position_valid={sample.position_valid}, position={sample.position_enu})'
            )
        if require_yaw and (not sample.yaw_valid or not math.isfinite(sample.yaw_rad)):
            raise FlightTestFailure(
                'vehicle yaw telemetry is invalid '
                f'(yaw_valid={sample.yaw_valid}, measured_yaw={sample.yaw_rad:.3f} rad, '
                f'position={sample.position_enu}, landed={sample.landed})'
            )
        if self.telemetry_fault is not None:
            raise FlightTestFailure(self.telemetry_fault)
        return sample

    def _require_airborne_sample(self, require_yaw: bool = False) -> FlightSample:
        """Require fresh armed telemetry in the mode required by the active case."""
        sample: FlightSample = self._current_sample(require_yaw=require_yaw)
        if not sample.armed or sample.landed or (self.expect_offboard and not sample.offboard):
            raise FlightTestFailure(
                'vehicle left its required armed flight mode '
                f'(armed={sample.armed}, landed={sample.landed}, '
                f'offboard={sample.offboard}, expected_offboard={self.expect_offboard})'
            )
        return sample

    def _wait_for_airborne_yaw(self) -> None:
        """Wait for PX4 yaw control readiness before commanding a heading."""
        deadline: float = time.monotonic() + AIRBORNE_YAW_TIMEOUT_SEC
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._require_airborne_sample().yaw_valid:
                return
        sample: FlightSample = self._current_sample()
        raise FlightTestFailure(
            f'PX4 yaw remained invalid for {AIRBORNE_YAW_TIMEOUT_SEC:.1f} seconds after takeoff '
            f'(yaw_valid={sample.yaw_valid}, measured_yaw={sample.yaw_rad:.3f} rad, '
            f'position={sample.position_enu})'
        )

    def _wait_for_duration(self, duration_sec: float) -> list[FlightSample]:
        """Collect fresh state for a bounded duration and return new samples."""
        start_monotonic_sec: float = time.monotonic()
        first_sample_index: int = len(self.samples)
        deadline: float = start_monotonic_sec + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._require_airborne_sample()
        return list(self.samples)[first_sample_index:]

    def _execute_action(
        self,
        client: ActionClient,
        goal: object,
        action_name: str,
        timeout_sec: float,
    ) -> ActionResultProtocol:
        """Send one action goal and require a successful terminal result."""
        if not client.wait_for_server(timeout_sec=5.0):
            raise FlightTestFailure(f'{action_name} action server is unavailable')
        goal_future: Future = client.send_goal_async(
            goal, feedback_callback=partial(self._record_action_feedback, action_name)
        )
        self._spin_until_future(goal_future, 10.0, f'{action_name} goal response')
        goal_handle: ClientGoalHandle = cast(ClientGoalHandle, goal_future.result())
        if not goal_handle.accepted:
            self.action_events.append(
                {
                    'action': action_name,
                    'status': 'rejected',
                    'goal': ros_value_to_json(goal),
                }
            )
            raise FlightTestFailure(f'{action_name} goal was rejected')
        action_started_sec: float = time.monotonic()
        result_future: Future = goal_handle.get_result_async()
        self._spin_until_future(result_future, timeout_sec, f'{action_name} result')
        wrapped_result: WrappedActionResultProtocol = cast(
            WrappedActionResultProtocol, result_future.result()
        )
        action_result: ActionResultProtocol = cast(ActionResultProtocol, wrapped_result.result)
        result_fields: dict[str, object] = cast(
            dict[str, object], ros_value_to_json(action_result)
        )
        self.action_events.append(
            {
                'action': action_name,
                'status': int(wrapped_result.status),
                'elapsed_sec': time.monotonic() - action_started_sec,
                'goal': ros_value_to_json(goal),
                'result': result_fields,
            }
        )
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or not action_result.success:
            raise FlightTestFailure(f'{action_name} failed: {action_result.message}')
        return action_result

    def _takeoff(self, altitude_m: float = TAKEOFF_ALTITUDE_M) -> None:
        """Take off to an altitude relative to the accepted start position."""
        self.expect_airborne = False
        self.expect_offboard = True
        goal: Takeoff.Goal = Takeoff.Goal()
        goal.altitude_m = float(altitude_m)
        self._execute_action(self.takeoff_client, goal, 'takeoff', DEFAULT_ACTION_TIMEOUT_SEC)
        self._require_airborne_sample()
        self.expect_airborne = True

    def _land(self) -> None:
        """Land through PX4 and verify landed, disarmed, and outside Offboard."""
        self.expect_airborne = False
        if not self.samples:
            return
        sample: FlightSample = self._current_sample()
        if sample.landed and not sample.armed and not sample.offboard:
            return
        goal: Land.Goal = Land.Goal()
        goal.timeout_sec = 60.0
        self._execute_action(self.land_client, goal, 'land', 70.0)
        deadline: float = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            landed_sample: FlightSample = self._current_sample()
            if landed_sample.landed and not landed_sample.armed and not landed_sample.offboard:
                return
        raise FlightTestFailure(
            'PX4 did not report landed, disarmed, and outside Offboard after landing'
        )

    def _record_assertion(self, name: str, actual: float, limit: float, relation: str) -> None:
        """Record one numeric assertion before raising on a failed threshold."""
        passed: bool = actual <= limit if relation == 'at_most' else actual >= limit
        self.assertions.append(
            {
                'name': name,
                'actual': actual,
                'relation': relation,
                'limit': limit,
                'passed': passed,
            }
        )
        if relation == 'at_most':
            require_at_most(name, actual, limit)
        else:
            require_at_least(name, actual, limit)

    def _samples_since(self, first_sample_index: int) -> list[FlightSample]:
        """Return recorded samples after a scenario boundary."""
        return list(self.samples)[first_sample_index:]

    def _ordered_target_indices(
        self,
        samples: list[FlightSample],
        targets: list[tuple[float, float, float]],
        tolerance_m: float,
    ) -> list[int]:
        """Find the first telemetry sample that reaches each target in order."""
        target_sample_indices: list[int] = []
        next_sample_index: int = 0
        for target_number, target in enumerate(targets, start=1):
            reached_index: int | None = None
            for sample_index in range(next_sample_index, len(samples)):
                if distance_3d(samples[sample_index].position_enu, target) <= tolerance_m:
                    reached_index = sample_index
                    break
            if reached_index is None:
                closest_error_m: float = min(
                    (
                        distance_3d(sample.position_enu, target)
                        for sample in samples[next_sample_index:]
                    ),
                    default=math.inf,
                )
                raise FlightTestFailure(
                    f'target {target_number} was not visited in order; '
                    f'closest remaining error was {closest_error_m:.2f} m'
                )
            target_sample_indices.append(reached_index)
            next_sample_index = reached_index + 1
        return target_sample_indices

    def _position_after_offset(
        self, origin: tuple[float, float, float], offset: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Add one ENU offset to an origin position."""
        return (origin[0] + offset[0], origin[1] + offset[1], origin[2] + offset[2])

    def _assert_case_boundary(
        self, origin: tuple[float, float, float], positions: list[tuple[float, float, float]]
    ) -> None:
        """Reject commanded positions outside the 20-meter SITL test envelope."""
        for position in positions:
            horizontal_distance_m: float = distance_2d(origin, position)
            self._record_assertion(
                'commanded horizontal boundary', horizontal_distance_m, 20.0, 'at_most'
            )
            vertical_offset_m: float = abs(position[2] - origin[2])
            self._record_assertion('commanded vertical boundary', vertical_offset_m, 8.0, 'at_most')

    def _record_trajectory_point(
        self,
        trajectory_goal: FollowTrajectory.Goal,
        position: tuple[float, float, float],
        time_from_start_sec: float,
        yaw_rad: float | None,
    ) -> None:
        """Append one typed ENU setpoint to a trajectory goal."""
        point: TrajectoryPoint = TrajectoryPoint()
        point.position.x = float(position[0])
        point.position.y = float(position[1])
        point.position.z = float(position[2])
        point.time_from_start_sec = float(time_from_start_sec)
        if yaw_rad is not None:
            point.yaw_rad = float(yaw_rad)
        trajectory_goal.points.append(point)
        self.command_writer.writerow(
            (
                self.case_id,
                len(trajectory_goal.points) - 1,
                point.time_from_start_sec,
                point.position.x,
                point.position.y,
                point.position.z,
                point.yaw_rad if yaw_rad is not None else '',
            )
        )
        self.command_file.flush()

    def _run_trajectory(self, trajectory_goal: FollowTrajectory.Goal) -> ActionResultProtocol:
        """Execute one validated application trajectory action."""
        self.last_trajectory_started_monotonic_sec: float = time.monotonic()
        return self._execute_action(
            self.trajectory_client,
            trajectory_goal,
            'follow_trajectory',
            DEFAULT_ACTION_TIMEOUT_SEC,
        )

    def _run_takeoff_hover_land(self) -> None:
        """Exercise takeoff, position hold, and landing behavior."""
        start_index: int = len(self.samples)
        starting_position: tuple[float, float, float] = self._current_sample().position_enu
        target_position: tuple[float, float, float] = (
            starting_position[0],
            starting_position[1],
            starting_position[2] + TAKEOFF_ALTITUDE_M,
        )
        self._takeoff()
        self._wait_for_duration(2.0)
        hold_samples: list[FlightSample] = self._wait_for_duration(5.0)
        if not hold_samples:
            raise FlightTestFailure('hover interval recorded no vehicle state samples')
        vertical_error_m: float = max(
            abs(sample.position_enu[2] - target_position[2]) for sample in hold_samples
        )
        horizontal_drift_m: float = max(
            distance_2d(sample.position_enu, target_position) for sample in hold_samples
        )
        self._record_assertion('takeoff vertical error', vertical_error_m, 0.5, 'at_most')
        self._record_assertion('hover horizontal drift', horizontal_drift_m, 0.5, 'at_most')
        self._land()
        self.assertions.append(
            {
                'name': 'landed, disarmed, and outside Offboard',
                'passed': (
                    self._current_sample().landed
                    and not self._current_sample().armed
                    and not self._current_sample().offboard
                ),
            }
        )
        if start_index >= len(self.samples):
            raise FlightTestFailure('takeoff scenario recorded no vehicle state samples')

    def _run_square_mission(self) -> None:
        """Fly the existing launch-relative four-waypoint square mission."""
        mission_origin: tuple[float, float, float] = self._current_sample().position_enu
        start_index: int = len(self.samples)
        goal: ExecuteWaypointMission.Goal = ExecuteWaypointMission.Goal()
        goal.takeoff_altitude_m = TAKEOFF_ALTITUDE_M
        goal.waypoint_offsets = []
        for east_m, north_m in ((3.0, 0.0), (3.0, 3.0), (0.0, 3.0), (0.0, 0.0)):
            offset: Point = Point()
            offset.x = east_m
            offset.y = north_m
            offset.z = TAKEOFF_ALTITUDE_M
            goal.waypoint_offsets.append(offset)
        goal.position_tolerance_m = 0.5
        goal.waypoint_timeout_sec = 30.0
        goal.land_after_waypoints = True
        goal.orbit_after_waypoints = False
        self._execute_action(self.mission_client, goal, 'execute_waypoint_mission', 180.0)
        mission_samples: list[FlightSample] = self._samples_since(start_index)
        if not mission_samples:
            raise FlightTestFailure('square mission recorded no vehicle state samples')
        waypoint_positions: list[tuple[float, float, float]] = [
            self._position_after_offset(mission_origin, offset)
            for offset in (
                (3.0, 0.0, TAKEOFF_ALTITUDE_M),
                (3.0, 3.0, TAKEOFF_ALTITUDE_M),
                (0.0, 3.0, TAKEOFF_ALTITUDE_M),
                (0.0, 0.0, TAKEOFF_ALTITUDE_M),
            )
        ]
        flight_samples: list[FlightSample] = [
            sample
            for sample in mission_samples
            if sample.position_enu[2] >= mission_origin[2] + 1.5
        ]
        if not flight_samples:
            raise FlightTestFailure('square mission recorded no airborne telemetry samples')
        waypoint_indices: list[int] = self._ordered_target_indices(
            flight_samples, waypoint_positions, 0.5
        )
        for waypoint_number, (waypoint_position, sample_index) in enumerate(
            zip(waypoint_positions, waypoint_indices, strict=True), start=1
        ):
            closest_error_m: float = distance_3d(
                flight_samples[sample_index].position_enu, waypoint_position
            )
            self._record_assertion(
                f'waypoint {waypoint_number} position error', closest_error_m, 0.5, 'at_most'
            )
        path_start: tuple[float, float, float] = self._position_after_offset(
            mission_origin, (0.0, 0.0, TAKEOFF_ALTITUDE_M)
        )
        path_points: list[tuple[float, float, float]] = [path_start, *waypoint_positions]
        path_error_values_m: list[float] = []
        segment_start_index: int = 0
        for segment_index, segment_end_index in enumerate(waypoint_indices, start=1):
            path_error_values_m.extend(
                cross_track_distance(
                    sample.position_enu, path_points[segment_index - 1], path_points[segment_index]
                )
                for sample in flight_samples[segment_start_index : segment_end_index + 1]
            )
            segment_start_index = segment_end_index
        if not path_error_values_m:
            raise FlightTestFailure('square mission path produced no segment samples')
        path_error_m: float = max(path_error_values_m)
        self._record_assertion('square path cross-track error', path_error_m, 1.5, 'at_most')
        final_position_error_m: float = distance_3d(
            flight_samples[waypoint_indices[-1]].position_enu, waypoint_positions[-1]
        )
        self._record_assertion('square return error', final_position_error_m, 0.75, 'at_most')
        mission_result_fields: dict[str, object] = cast(
            dict[str, object], self.action_events[-1]['result']
        )
        completed_waypoints: int = int(mission_result_fields['completed_waypoints'])
        self._record_assertion(
            'completed square waypoints', float(completed_waypoints), 4.0, 'at_least'
        )
        self._record_assertion(
            'completed square waypoint count', float(completed_waypoints), 4.0, 'at_most'
        )

    def _run_fast_stop_and_return(self) -> None:
        """Fly an outbound path, turn, return to the origin, and land."""
        self._takeoff()
        self._wait_for_airborne_yaw()
        start_index: int = len(self.samples)
        starting_sample: FlightSample = self._current_sample(require_yaw=True)
        start_position: tuple[float, float, float] = starting_sample.position_enu
        return_yaw_rad: float = math.atan2(
            math.sin(starting_sample.yaw_rad + math.pi),
            math.cos(starting_sample.yaw_rad + math.pi),
        )
        outbound_position: tuple[float, float, float] = (
            start_position[0] + 12.0,
            start_position[1],
            start_position[2],
        )
        self._assert_case_boundary(start_position, [outbound_position])
        goal: FollowTrajectory.Goal = FollowTrajectory.Goal()
        goal.control_yaw = True
        goal.position_tolerance_m = 0.5
        goal.yaw_tolerance_rad = 0.2
        goal.timeout_sec = 30.0
        self._record_trajectory_point(goal, start_position, 0.0, starting_sample.yaw_rad)
        self._record_trajectory_point(goal, outbound_position, 4.0, starting_sample.yaw_rad)
        self._record_trajectory_point(goal, outbound_position, 6.0, starting_sample.yaw_rad)
        self._record_trajectory_point(goal, outbound_position, 8.0, return_yaw_rad)
        self._record_trajectory_point(goal, start_position, 12.0, return_yaw_rad)
        self._record_trajectory_point(goal, start_position, 14.0, return_yaw_rad)
        self._run_trajectory(goal)
        samples: list[FlightSample] = self._samples_since(start_index)
        if not samples:
            raise FlightTestFailure('stop-and-return trajectory recorded no state samples')
        position_samples: list[FlightSample] = [sample for sample in samples if sample.position_valid]
        velocity_samples: list[FlightSample] = [sample for sample in samples if sample.velocity_valid]
        if not position_samples or not velocity_samples:
            raise FlightTestFailure('stop-and-return trajectory lacks valid position or velocity')
        maximum_path_error_m: float = max(
            math.hypot(
                cross_track_distance(sample.position_enu, start_position, outbound_position),
                sample.position_enu[2] - start_position[2],
            )
            for sample in position_samples
        )
        self._record_assertion(
            'out-and-back path error', maximum_path_error_m, FT02_PATH_TOLERANCE_M, 'at_most'
        )
        maximum_horizontal_speed_mps: float = max(
            math.hypot(sample.velocity_enu[0], sample.velocity_enu[1])
            for sample in velocity_samples
        )
        self._record_assertion(
            'out-and-back horizontal speed',
            maximum_horizontal_speed_mps,
            FT02_MAX_HORIZONTAL_SPEED_MPS,
            'at_most',
        )
        outbound_position_error_m: float = min(
            distance_3d(sample.position_enu, outbound_position) for sample in position_samples
        )
        self._record_assertion(
            'outbound position error',
            outbound_position_error_m,
            FT02_ENDPOINT_TOLERANCE_M,
            'at_most',
        )
        outbound_sample_index: int = next(
            index
            for index, sample in enumerate(position_samples)
            if distance_3d(sample.position_enu, outbound_position) <= FT02_ENDPOINT_TOLERANCE_M
        )
        return_position_error_m: float = min(
            (
                distance_3d(sample.position_enu, start_position)
                for sample in position_samples[outbound_sample_index + 1 :]
            ),
            default=math.inf,
        )
        self._record_assertion(
            'return position error', return_position_error_m, FT02_ENDPOINT_TOLERANCE_M, 'at_most'
        )
        self._land()
        self.measurements['landing_horizontal_offset_m'] = distance_2d(
            self._current_sample().position_enu, start_position
        )

    def _make_orbit_goal(
        self,
        center_position: tuple[float, float, float],
        radius_m: float,
        revolutions: float,
        timeout_sec: float,
    ) -> OrbitPoint.Goal:
        """Build a counterclockwise PX4 orbit goal with yaw facing its center."""
        center: Point = Point()
        center.x, center.y, center.z = center_position
        goal: OrbitPoint.Goal = OrbitPoint.Goal()
        goal.center_position = center
        goal.frame_id = 'local_enu'
        goal.face_center = True
        goal.yaw_tolerance_rad = float(math.radians(30.0))
        goal.radius_m = float(radius_m)
        goal.clockwise = False
        goal.tangential_speed_mps = 1.5
        goal.revolutions = float(revolutions)
        goal.timeout_sec = float(timeout_sec)
        return goal

    def _run_orbit_cancel_and_hold(
        self, center_position: tuple[float, float, float], radius_m: float
    ) -> None:
        """Cancel a moving OrbitPoint goal and verify restored Offboard hold."""
        goal: OrbitPoint.Goal = self._make_orbit_goal(
            center_position, radius_m, revolutions=3.0, timeout_sec=120.0
        )
        self.expect_offboard = False
        goal_future: Future = self.orbit_client.send_goal_async(
            goal, feedback_callback=partial(self._record_action_feedback, 'orbit_point')
        )
        self._spin_until_future(goal_future, 10.0, 'cancelable orbit goal response')
        goal_handle: ClientGoalHandle = cast(ClientGoalHandle, goal_future.result())
        if not goal_handle.accepted:
            self.expect_offboard = True
            self.action_events.append(
                {
                    'action': 'orbit_point',
                    'status': 'rejected',
                    'goal': ros_value_to_json(goal),
                }
            )
            raise FlightTestFailure('cancelable orbit goal was rejected')

        action_started_sec: float = time.monotonic()
        result_future: Future = goal_handle.get_result_async()
        orbit_progress_rad: float = 0.0
        previous_angle_rad: float | None = None
        progress_deadline: float = time.monotonic() + 60.0
        try:
            while time.monotonic() < progress_deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if result_future.done():
                    raise FlightTestFailure(
                        'cancelable orbit completed or failed before measured cancellation progress'
                    )
                sample: FlightSample = self._require_airborne_sample(require_yaw=True)
                radius_error_m: float = abs(
                    distance_2d(sample.position_enu, center_position) - radius_m
                )
                if (
                    sample.velocity_valid
                    and math.hypot(sample.velocity_enu[0], sample.velocity_enu[1]) > 0.2
                    and radius_error_m <= 2.0
                    and abs(sample.position_enu[2] - center_position[2]) <= 1.0
                ):
                    angle_rad: float = math.atan2(
                        sample.position_enu[1] - center_position[1],
                        sample.position_enu[0] - center_position[0],
                    )
                    if previous_angle_rad is not None:
                        orbit_progress_rad += wrapped_angle_error(angle_rad, previous_angle_rad)
                    previous_angle_rad = angle_rad
                if orbit_progress_rad >= math.pi / 2.0:
                    break
            if orbit_progress_rad < math.pi / 2.0:
                raise FlightTestFailure(
                    'cancelable orbit did not reach 90 degrees of in-band angular progress'
                )

            cancellation_future: Future = goal_handle.cancel_goal_async()
            self._spin_until_future(cancellation_future, 10.0, 'orbit cancellation response')
            cancellation_response: object = cancellation_future.result()
            goals_canceling: list[object] = cast(
                list[object], getattr(cancellation_response, 'goals_canceling', [])
            )
            if not goals_canceling:
                raise FlightTestFailure('orbit goal was not accepted for cancellation')
            self._spin_until_future(result_future, 20.0, 'orbit canceled result')
        finally:
            if not result_future.done():
                try:
                    cancellation_future = goal_handle.cancel_goal_async()
                    self._spin_until_future(
                        cancellation_future, 10.0, 'orbit cleanup cancellation response'
                    )
                    self._spin_until_future(result_future, 20.0, 'orbit cleanup result')
                except Exception as exception:  # noqa: BLE001
                    self.recovery_message = (
                        f'orbit cancellation cleanup failed: {type(exception).__name__}: '
                        f'{exception}'
                    )
            self.expect_offboard = True

        wrapped_result: WrappedActionResultProtocol = cast(
            WrappedActionResultProtocol, result_future.result()
        )
        orbit_result: OrbitPoint.Result = cast(OrbitPoint.Result, wrapped_result.result)
        self.action_events.append(
            {
                'action': 'orbit_point',
                'status': int(wrapped_result.status),
                'elapsed_sec': time.monotonic() - action_started_sec,
                'cancelled_after_angular_progress_rad': orbit_progress_rad,
                'goal': ros_value_to_json(goal),
                'result': ros_value_to_json(orbit_result),
            }
        )
        self.measurements['orbit_cancel_progress_rad'] = orbit_progress_rad
        self._record_assertion(
            'orbit cancellation progress', orbit_progress_rad, math.pi / 2.0, 'at_least'
        )
        cancellation_passed: bool = wrapped_result.status == GoalStatus.STATUS_CANCELED
        self.assertions.append(
            {
                'name': 'orbit canceled terminal status',
                'actual': int(wrapped_result.status),
                'expected': GoalStatus.STATUS_CANCELED,
                'passed': cancellation_passed,
            }
        )
        if not cancellation_passed:
            raise FlightTestFailure(
                f'orbit terminal status was {wrapped_result.status}, expected canceled'
            )

        hold_reference: FlightSample = self._require_airborne_sample(require_yaw=True)
        self.assertions.append(
            {
                'name': 'Offboard restored after orbit cancellation',
                'passed': hold_reference.offboard
                and hold_reference.armed
                and not hold_reference.landed,
            }
        )
        first_setpoint_count: int = hold_reference.setpoint_publication_count
        hold_samples: list[FlightSample] = self._wait_for_duration(2.0)
        if not hold_samples:
            raise FlightTestFailure('post-orbit-cancel hold produced no telemetry samples')
        hold_drift_m: float = max(
            distance_3d(sample.position_enu, hold_reference.position_enu) for sample in hold_samples
        )
        self._record_assertion('orbit cancel hold drift', hold_drift_m, 0.75, 'at_most')
        setpoint_increments: int = (
            hold_samples[-1].setpoint_publication_count - first_setpoint_count
        )
        self._record_assertion(
            'orbit cancel hold setpoint publications', float(setpoint_increments), 1.0, 'at_least'
        )

    def _run_orbit(self) -> None:
        """Orbit a local ENU point and record radial and heading tracking."""
        self._takeoff(8.0)
        self._wait_for_airborne_yaw()
        origin: tuple[float, float, float] = self._current_sample().position_enu
        center_position: tuple[float, float, float] = (origin[0] + 12.0, origin[1], origin[2])
        orbit_radius_m: float = 6.0
        self._assert_case_boundary(
            origin,
            [
                (center_position[0] - orbit_radius_m, center_position[1], center_position[2]),
                (center_position[0] + orbit_radius_m, center_position[1], center_position[2]),
                (center_position[0], center_position[1] - orbit_radius_m, center_position[2]),
                (center_position[0], center_position[1] + orbit_radius_m, center_position[2]),
            ],
        )
        self._run_orbit_cancel_and_hold(center_position, orbit_radius_m)
        start_index: int = len(self.samples)
        goal: OrbitPoint.Goal = self._make_orbit_goal(
            center_position, orbit_radius_m, revolutions=1.0, timeout_sec=180.0
        )
        self.expect_offboard = False
        try:
            result: ActionResultProtocol = self._execute_action(
                self.orbit_client, goal, 'orbit_point', 185.0
            )
        finally:
            self.expect_offboard = True
        self._require_airborne_sample()
        completed_revolutions: float = float(cast(OrbitPoint.Result, result).completed_revolutions)
        self._record_assertion(
            'completed orbit revolutions', completed_revolutions, 1.0, 'at_least'
        )
        samples: list[FlightSample] = self._samples_since(start_index)
        orbit_samples: list[FlightSample] = [
            sample
            for sample in samples
            if sample.yaw_valid
            and sample.velocity_valid
            and math.hypot(sample.velocity_enu[0], sample.velocity_enu[1]) > 0.2
            and abs(distance_2d(sample.position_enu, center_position) - orbit_radius_m) <= 2.0
            and abs(sample.position_enu[2] - center_position[2]) <= 1.0
        ]
        if not orbit_samples:
            raise FlightTestFailure('no telemetry samples fell within the requested orbit annulus')
        radial_errors_m: list[float] = [
            abs(distance_2d(sample.position_enu, center_position) - orbit_radius_m)
            for sample in orbit_samples
        ]
        maximum_radial_error_m: float = max(radial_errors_m)
        self.measurements['orbit_maximum_radial_error_m'] = maximum_radial_error_m
        angles_rad: list[float] = [
            math.atan2(
                sample.position_enu[1] - center_position[1],
                sample.position_enu[0] - center_position[0],
            )
            for sample in orbit_samples
        ]
        signed_angular_progress_rad: float = sum(
            wrapped_angle_error(current_angle_rad, previous_angle_rad)
            for previous_angle_rad, current_angle_rad in pairwise(angles_rad)
        )
        self.measurements['orbit_signed_angular_progress_rad'] = signed_angular_progress_rad
        self._record_assertion(
            'counterclockwise angular progress',
            signed_angular_progress_rad,
            2.0 * math.pi * 0.9,
            'at_least',
        )
        heading_errors_rad: list[float] = [
            abs(
                wrapped_angle_error(
                    math.atan2(
                        center_position[1] - sample.position_enu[1],
                        center_position[0] - sample.position_enu[0],
                    ),
                    sample.yaw_rad,
                )
            )
            for sample in orbit_samples
        ]
        heading_p95_rad: float = percentile(heading_errors_rad, 0.95)
        maximum_heading_error_rad: float = max(heading_errors_rad)
        self.measurements['orbit_maximum_heading_error_rad'] = maximum_heading_error_rad
        self.measurements['orbit_radial_error_p95_m'] = percentile(radial_errors_m, 0.95)
        self.measurements['orbit_heading_error_p95_rad'] = heading_p95_rad
        self._land()

    def _run_search_lanes(self) -> None:
        """Fly three timed lawnmower lanes at a fixed altitude."""
        self._takeoff()
        start_index: int = len(self.samples)
        initial_sample: FlightSample = self._current_sample()
        origin: tuple[float, float, float] = initial_sample.position_enu
        targets: list[tuple[float, float, float]] = [
            origin,
            (origin[0] + 12.0, origin[1], origin[2]),
            (origin[0] + 12.0, origin[1] + 4.0, origin[2]),
            (origin[0], origin[1] + 4.0, origin[2]),
            (origin[0], origin[1] + 8.0, origin[2]),
            (origin[0] + 12.0, origin[1] + 8.0, origin[2]),
        ]
        self._assert_case_boundary(origin, targets)
        schedule: tuple[tuple[float, int], ...] = (
            (0.0, 0),
            (4.0, 1),
            (5.0, 1),
            (7.0, 2),
            (8.0, 2),
            (12.0, 3),
            (13.0, 3),
            (15.0, 4),
            (16.0, 4),
            (20.0, 5),
            (22.0, 5),
        )
        goal: FollowTrajectory.Goal = FollowTrajectory.Goal()
        goal.position_tolerance_m = 0.5
        goal.timeout_sec = 45.0
        for time_from_start_sec, target_index in schedule:
            self._record_trajectory_point(
                goal, targets[target_index], time_from_start_sec, None
            )
        self._run_trajectory(goal)
        samples: list[FlightSample] = self._samples_since(start_index)
        if not samples:
            raise FlightTestFailure('search-lane trajectory recorded no state samples')
        endpoint_indices: list[int] = self._ordered_target_indices(samples, targets[1:], 0.75)
        endpoint_errors_m: list[float] = [
            distance_3d(samples[sample_index].position_enu, target)
            for sample_index, target in zip(endpoint_indices, targets[1:], strict=True)
        ]
        for lane_number, endpoint_error_m in enumerate(endpoint_errors_m, start=1):
            self._record_assertion(
                f'lane endpoint {lane_number} error', endpoint_error_m, 0.75, 'at_most'
            )
        line_segments: tuple[tuple[int, int], ...] = ((0, 1), (2, 3), (4, 5))
        target_indices: list[int] = [0, *endpoint_indices]
        straight_leg_samples: list[float] = []
        for start_index_in_targets, end_index_in_targets in line_segments:
            start_sample_index: int = target_indices[start_index_in_targets]
            end_sample_index: int = target_indices[end_index_in_targets]
            straight_leg_samples.extend(
                cross_track_distance(
                    sample.position_enu,
                    targets[start_index_in_targets],
                    targets[end_index_in_targets],
                )
                for sample in samples[start_sample_index : end_sample_index + 1]
                if sample.velocity_valid and math.hypot(*sample.velocity_enu[:2]) > 0.5
            )
        if not straight_leg_samples:
            raise FlightTestFailure('search-lane path contains no moving samples')
        self._record_assertion(
            'search-lane cross-track error', max(straight_leg_samples), 1.5, 'at_most'
        )
        self._land()

    def _run_altitude_steps(self) -> None:
        """Climb, hold, descend, and measure vertical response and lateral drift."""
        self._takeoff()
        start_index: int = len(self.samples)
        starting_sample: FlightSample = self._current_sample()
        start_position: tuple[float, float, float] = starting_sample.position_enu
        climb_position: tuple[float, float, float] = (
            start_position[0],
            start_position[1],
            start_position[2] + 3.0,
        )
        self._assert_case_boundary(start_position, [climb_position])
        goal: FollowTrajectory.Goal = FollowTrajectory.Goal()
        goal.position_tolerance_m = 0.5
        goal.timeout_sec = 30.0
        self._record_trajectory_point(goal, start_position, 0.0, None)
        self._record_trajectory_point(goal, climb_position, 3.0, None)
        self._record_trajectory_point(goal, climb_position, 5.0, None)
        self._record_trajectory_point(goal, start_position, 8.0, None)
        self._record_trajectory_point(goal, start_position, 10.0, None)
        self._run_trajectory(goal)
        samples: list[FlightSample] = self._samples_since(start_index)
        if not samples:
            raise FlightTestFailure('altitude trajectory recorded no vehicle state samples')
        maximum_climb_overshoot_m: float = max(
            sample.position_enu[2] - climb_position[2] for sample in samples
        )
        self._record_assertion('climb overshoot', maximum_climb_overshoot_m, 1.0, 'at_most')
        maximum_lateral_drift_m: float = max(
            distance_2d(sample.position_enu, start_position) for sample in samples
        )
        self._record_assertion(
            'altitude-step lateral drift', maximum_lateral_drift_m, 1.0, 'at_most'
        )
        climb_settle_error_m: float = min(
            abs(sample.position_enu[2] - climb_position[2]) for sample in samples
        )
        self._record_assertion('climb altitude error', climb_settle_error_m, 0.5, 'at_most')
        climb_sample_index: int = next(
            index
            for index, sample in enumerate(samples)
            if abs(sample.position_enu[2] - climb_position[2]) <= 0.5
        )
        descent_samples: list[FlightSample] = samples[climb_sample_index + 1 :]
        return_settle_error_m: float = min(
            (abs(sample.position_enu[2] - start_position[2]) for sample in descent_samples),
            default=math.inf,
        )
        self._record_assertion('descent altitude error', return_settle_error_m, 0.5, 'at_most')

        def elapsed_from_trajectory_start(sample: FlightSample) -> float:
            return sample.received_monotonic_sec - self.last_trajectory_started_monotonic_sec

        climb_hold_samples: list[FlightSample] = [
            sample for sample in samples if 3.0 <= elapsed_from_trajectory_start(sample) <= 5.5
        ]
        climb_settle_duration_sec: float = self._longest_boolean_interval(
            climb_hold_samples,
            lambda sample: abs(sample.position_enu[2] - climb_position[2]) <= 0.5,
        )
        self.measurements['climb_settle_duration_sec'] = climb_settle_duration_sec
        self.measurements['climb_rise_time_sec'] = elapsed_from_trajectory_start(
            samples[climb_sample_index]
        )
        descent_overshoot_m: float = max(
            (start_position[2] - sample.position_enu[2] for sample in descent_samples),
            default=0.0,
        )
        self._record_assertion('descent undershoot', descent_overshoot_m, 1.0, 'at_most')
        descent_settle_duration_sec: float = self._longest_boolean_interval(
            descent_samples,
            lambda sample: abs(sample.position_enu[2] - start_position[2]) <= 0.5,
        )
        self.measurements['descent_settle_duration_sec'] = descent_settle_duration_sec
        self._land()

    def _run_cancel_navigation(self) -> None:
        """Cancel a moving navigation goal and verify Offboard position hold."""
        self._takeoff()
        start_position: tuple[float, float, float] = self._current_sample().position_enu
        goal: NavigateTo.Goal = NavigateTo.Goal()
        goal.target_position.x = start_position[0] + 10.0
        goal.target_position.y = start_position[1]
        goal.target_position.z = start_position[2]
        goal.frame_id = 'local_enu'
        goal.position_tolerance_m = 0.5
        goal.timeout_sec = 60.0
        goal_future: Future = self.navigation_client.send_goal_async(
            goal, feedback_callback=partial(self._record_action_feedback, 'navigate_to')
        )
        self._spin_until_future(goal_future, 10.0, 'navigate_to goal response')
        goal_handle: ClientGoalHandle = cast(ClientGoalHandle, goal_future.result())
        if not goal_handle.accepted:
            self.action_events.append(
                {
                    'action': 'navigate_to',
                    'status': 'rejected',
                    'goal': ros_value_to_json(goal),
                }
            )
            raise FlightTestFailure('navigate_to cancellation goal was rejected')

        action_started_sec: float = time.monotonic()
        result_future: Future = goal_handle.get_result_async()
        movement_deadline: float = time.monotonic() + 30.0
        progress_m: float = 0.0
        while time.monotonic() < movement_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            sample: FlightSample = self._require_airborne_sample()
            progress_m = max(progress_m, distance_2d(sample.position_enu, start_position))
            if progress_m >= 3.0:
                break
            if result_future.done():
                raise FlightTestFailure('navigate_to completed before cancellation progress')
        if progress_m < 3.0:
            raise FlightTestFailure('navigate_to did not make 3 m of progress before timeout')

        cancellation_future: Future = goal_handle.cancel_goal_async()
        self._spin_until_future(cancellation_future, 5.0, 'navigate_to cancellation response')
        cancellation_response: object = cancellation_future.result()
        goals_canceling: list[object] = cast(
            list[object], getattr(cancellation_response, 'goals_canceling', [])
        )
        if not goals_canceling:
            raise FlightTestFailure('navigate_to goal was not accepted for cancellation')
        self._spin_until_future(result_future, 10.0, 'navigate_to canceled result')
        wrapped_result: WrappedActionResultProtocol = cast(
            WrappedActionResultProtocol, result_future.result()
        )
        if wrapped_result.status != GoalStatus.STATUS_CANCELED:
            raise FlightTestFailure(
                f'navigate_to terminal status was {wrapped_result.status}, expected canceled'
            )
        self.action_events.append(
            {
                'action': 'navigate_to',
                'status': int(wrapped_result.status),
                'elapsed_sec': time.monotonic() - action_started_sec,
                'cancelled_after_progress_m': progress_m,
                'goal': ros_value_to_json(goal),
                'result': ros_value_to_json(wrapped_result.result),
            }
        )

        hold_reference: FlightSample = self._require_airborne_sample()
        first_setpoint_count: int = hold_reference.setpoint_publication_count
        hold_samples: list[FlightSample] = self._wait_for_duration(5.0)
        if not hold_samples:
            raise FlightTestFailure('post-cancel hold produced no telemetry samples')
        hold_window_start_sec: float = hold_samples[-1].received_monotonic_sec - 4.0
        final_hold_samples: list[FlightSample] = [
            sample
            for sample in hold_samples
            if sample.received_monotonic_sec >= hold_window_start_sec
        ]
        drift_m: float = max(
            distance_3d(sample.position_enu, hold_reference.position_enu)
            for sample in final_hold_samples
        )
        self._record_assertion('post-cancel hold drift', drift_m, 0.75, 'at_most')
        setpoint_increments: int = (
            hold_samples[-1].setpoint_publication_count - first_setpoint_count
        )
        self._record_assertion(
            'post-cancel setpoint publications', float(setpoint_increments), 1.0, 'at_least'
        )
        offboard_duration_sec: float = self._longest_boolean_interval(
            hold_samples, lambda sample: sample.offboard
        )
        self._record_assertion(
            'post-cancel Offboard duration', offboard_duration_sec, 4.0, 'at_least'
        )
        self._land()

    def _longest_boolean_interval(
        self, samples: list[FlightSample], predicate: Callable[[FlightSample], bool]
    ) -> float:
        """Measure the longest fresh interval for which a predicate remains true."""
        longest_interval_sec: float = 0.0
        interval_start_sec: float | None = None
        previous_sample: FlightSample | None = None
        for sample in samples:
            is_contiguous: bool = (
                previous_sample is not None
                and sample.received_monotonic_sec - previous_sample.received_monotonic_sec
                <= SAMPLE_GAP_LIMIT_SEC
            )
            if predicate(sample) and is_contiguous and previous_sample is not None:
                if interval_start_sec is None:
                    interval_start_sec = previous_sample.received_monotonic_sec
            elif predicate(sample):
                if interval_start_sec is not None and previous_sample is not None:
                    longest_interval_sec = max(
                        longest_interval_sec,
                        previous_sample.received_monotonic_sec - interval_start_sec,
                    )
                interval_start_sec = sample.received_monotonic_sec
            else:
                if interval_start_sec is not None and previous_sample is not None:
                    longest_interval_sec = max(
                        longest_interval_sec,
                        previous_sample.received_monotonic_sec - interval_start_sec,
                    )
                interval_start_sec = None
            previous_sample = sample
        if interval_start_sec is not None and previous_sample is not None:
            longest_interval_sec = max(
                longest_interval_sec,
                previous_sample.received_monotonic_sec - interval_start_sec,
            )
        return longest_interval_sec

    def run_case(self) -> dict[str, object]:
        """Execute the selected flight case, attempt recovery, and write evidence."""
        required_clients_by_case: dict[str, tuple[ActionClient, ...]] = {
            'FT-00': (self.takeoff_client, self.land_client),
            'FT-01': (self.mission_client, self.takeoff_client, self.land_client),
            'FT-02': (self.takeoff_client, self.land_client, self.trajectory_client),
            'FT-03': (self.takeoff_client, self.land_client, self.orbit_client),
            'FT-04': (self.takeoff_client, self.land_client, self.trajectory_client),
            'FT-05': (self.takeoff_client, self.land_client, self.trajectory_client),
            'FT-06': (self.takeoff_client, self.land_client, self.navigation_client),
        }
        started_at_utc: str = datetime.now(UTC).isoformat()
        case_method_by_id: dict[str, Callable[[], None]] = {
            'FT-00': self._run_takeoff_hover_land,
            'FT-01': self._run_square_mission,
            'FT-02': self._run_fast_stop_and_return,
            'FT-03': self._run_orbit,
            'FT-04': self._run_search_lanes,
            'FT-05': self._run_altitude_steps,
            'FT-06': self._run_cancel_navigation,
        }
        status: str = 'passed'
        failure_message: str | None = None
        self.case_started_monotonic_sec = time.monotonic()
        try:
            self._wait_for_ready(60.0, required_clients_by_case[self.case_id])
            self.case_start_sample_index = len(self.samples)
            self.case_active = True
            case_method_by_id[self.case_id]()
            final_sample: FlightSample = self._current_sample()
            if not final_sample.landed or final_sample.armed or final_sample.offboard:
                raise FlightTestFailure(
                    'case ended without landed, disarmed, and outside Offboard state'
                )
            self.assertions.append(
                {'name': 'landed, disarmed, and outside Offboard', 'passed': True}
            )
        except Exception as exception:  # noqa: BLE001
            status = 'failed'
            failure_message = f'{type(exception).__name__}: {exception}'
            try:
                sample: FlightSample = self._current_sample()
                if sample.armed or not sample.landed or sample.offboard:
                    self._land()
                    self.recovery_message = 'landing action completed after case failure'
            except Exception as recovery_exception:  # noqa: BLE001
                self.recovery_message = (
                    f'landing recovery failed: {type(recovery_exception).__name__}: '
                    f'{recovery_exception}'
                )
        finally:
            self.case_active = False

        ended_at_utc: str = datetime.now(UTC).isoformat()
        samples: list[FlightSample] = list(self.samples)[self.case_start_sample_index :]
        sample_gaps_sec: list[float] = [
            later.received_monotonic_sec - earlier.received_monotonic_sec
            for earlier, later in pairwise(samples)
        ]
        result: dict[str, object] = {
            'case_id': self.case_id,
            'status': status,
            'started_at_utc': started_at_utc,
            'ended_at_utc': ended_at_utc,
            'elapsed_sec': (
                time.monotonic() - self.case_started_monotonic_sec
                if self.case_started_monotonic_sec
                else 0.0
            ),
            'failure': failure_message,
            'recovery': self.recovery_message,
            'sample_count': len(samples),
            'maximum_sample_gap_sec': max(sample_gaps_sec, default=0.0),
            'action_events': self.action_events,
            'assertions': self.assertions,
            'measurements': self.measurements,
        }
        if self.case_id == 'FT-03':
            result['orbit_center_type'] = 'virtual local_enu point'
            result['building_fixture_configured'] = False
            result['contact_monitor_configured'] = False
        (self.artifact_directory / 'action-events.json').write_text(
            json.dumps(self.action_events, indent=2) + '\n', encoding='utf-8'
        )
        (self.artifact_directory / 'assertions.json').write_text(
            json.dumps(self.assertions, indent=2) + '\n', encoding='utf-8'
        )
        (self.artifact_directory / 'case-result.json').write_text(
            json.dumps(result, indent=2) + '\n', encoding='utf-8'
        )
        failed_assertions: list[dict[str, object]] = [
            assertion for assertion in self.assertions if assertion.get('passed') is False
        ]
        first_failed_assertion: str = (
            str(failed_assertions[0].get('name')) if failed_assertions else 'none'
        )
        report_lines: list[str] = [
            f'# Flight test {self.case_id}',
            '',
            f'- Status: {status}',
            f'- Failure: {failure_message or "none"}',
            f'- First failed assertion: {first_failed_assertion}',
            f'- Telemetry samples: {len(samples)}',
            f'- Maximum sample gap: {max(sample_gaps_sec, default=0.0):.3f} s',
            f'- Recovery: {self.recovery_message or "not required"}',
            f'- Measurements: `{json.dumps(self.measurements, sort_keys=True)}`',
            '',
            '## Assertions',
            '',
            '| Assertion | Actual | Relation | Limit | Result |',
            '| --- | ---: | --- | ---: | --- |',
        ]
        for assertion in self.assertions:
            report_lines.append(
                '| {name} | {actual} | {relation} | {limit} | {result} |'.format(
                    name=assertion.get('name', 'unrecorded'),
                    actual=assertion.get('actual', ''),
                    relation=assertion.get('relation', ''),
                    limit=assertion.get('limit', ''),
                    result='passed' if assertion.get('passed') else 'failed',
                )
            )
        if self.case_id == 'FT-03':
            report_lines.extend(
                [
                    '',
                    (
                        'FT-03 orbits a virtual `local_enu` center. The Gazebo world has no '
                        'building fixture or contact monitor, so this run makes no collision '
                        'or building-clearance claim.'
                    ),
                ]
            )
        report_lines.extend(
            [
                '',
                'Telemetry: [vehicle-state.csv](vehicle-state.csv)',
                'Commands: [commanded-trajectory.csv](commanded-trajectory.csv)',
                'Actions: [action-events.json](action-events.json)',
                'Feedback: [action-feedback.jsonl](action-feedback.jsonl)',
                '',
            ]
        )
        (self.artifact_directory / 'case-report.md').write_text(
            '\n'.join(report_lines), encoding='utf-8'
        )
        return result


def parse_arguments(arguments: list[str] | None = None) -> argparse.Namespace:
    """Parse one standalone flight and an optional artifact destination."""
    parser = argparse.ArgumentParser(description='Execute one PX4 SITL flight test case.')
    parser.add_argument(
        '--case',
        choices=('FT-00', 'FT-01', 'FT-02', 'FT-03', 'FT-04', 'FT-05', 'FT-06'),
        required=True,
    )
    parser.add_argument('--artifact-dir', type=Path)
    return parser.parse_args(arguments)


def resolve_artifact_directory(case_id: str, requested_directory: Path | None) -> Path:
    """Create a unique artifact directory for one flight invocation."""
    if requested_directory is not None:
        artifact_directory: Path = requested_directory.resolve()
        artifact_directory.mkdir(parents=True, exist_ok=False)
    else:
        run_directory_value: str | None = os.environ.get('DRONE_RUN_DIR')
        if not run_directory_value:
            raise ValueError('DRONE_RUN_DIR is required when --artifact-dir is omitted')
        run_directory: Path = Path(run_directory_value).resolve()
        if not run_directory.is_dir():
            raise ValueError(f'run directory does not exist: {run_directory}')
        artifact_root: Path = run_directory / 'flight-tests'
        artifact_root.mkdir(exist_ok=True)
        timestamp: str = datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ')
        artifact_directory = Path(
            tempfile.mkdtemp(prefix=f'{case_id}-{timestamp}-', dir=artifact_root)
        )
    try:
        artifact_directory.chmod(0o755)
    except OSError:
        artifact_directory.rmdir()
        raise
    return artifact_directory


def main(arguments: list[str] | None = None) -> int:
    """Run one flight and return nonzero unless every case gate passes."""
    options: argparse.Namespace = parse_arguments(arguments)
    try:
        artifact_directory: Path = resolve_artifact_directory(
            str(options.case), options.artifact_dir
        )
    except (OSError, ValueError) as exception:
        print(f'flight artifact setup failed: {exception}', file=sys.stderr)
        return 2
    print(f'Artifacts: {artifact_directory}', flush=True)
    runner: FlightTestRunner | None = None
    lock_stream: TextIO | None = None
    try:
        lock_directory: Path = artifact_directory.parent
        run_directory_value: str | None = os.environ.get('DRONE_RUN_DIR')
        if run_directory_value:
            run_directory: Path = Path(run_directory_value).resolve()
            if run_directory.is_dir():
                lock_directory = run_directory / 'flight-tests'
                lock_directory.mkdir(exist_ok=True)
        lock_stream = (lock_directory / '.flight-case.lock').open('a', encoding='utf-8')
        try:
            fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exception:
            raise FlightTestFailure('another flight case is already running') from exception
        rclpy.init(args=None)
        runner = FlightTestRunner(str(options.case), artifact_directory)
        result: dict[str, object] = runner.run_case()
        print(f'{result["case_id"]}: {result["status"]}')
        return 0 if result['status'] == 'passed' else 1
    except Exception as exception:  # noqa: BLE001
        failure_result: dict[str, object] = {
            'case_id': str(options.case),
            'status': 'failed',
            'failure': f'{type(exception).__name__}: {exception}',
        }
        result_path: Path = artifact_directory / 'case-result.json'
        if result_path.is_file():
            try:
                existing_result: dict[str, object] = cast(
                    dict[str, object], json.loads(result_path.read_text(encoding='utf-8'))
                )
                existing_result.update(failure_result)
                failure_result = existing_result
            except (json.JSONDecodeError, OSError):
                pass
        result_path.write_text(json.dumps(failure_result, indent=2) + '\n', encoding='utf-8')
        print(f'{failure_result["failure"]}')
        return 1
    finally:
        try:
            if runner is not None:
                runner.close()
                runner.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            if lock_stream is not None:
                lock_stream.close()
