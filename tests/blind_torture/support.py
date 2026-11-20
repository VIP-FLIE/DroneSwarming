"""ROS wire peers and process fixtures for the blind node tests."""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import threading
import time
import unittest
from pathlib import Path
from typing import TYPE_CHECKING, TypeVar

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from swarm_interfaces.action import (
    ExecuteFlightPlan,
    ExecuteWaypointMission,
    FlyMission,
    FollowTrajectory,
    Land,
    NavigateTo,
    OrbitPoint,
    Takeoff,
)
from swarm_interfaces.msg import GeographicPosition, TrajectoryPoint, VehicleState
from swarm_interfaces.srv import PlanFlight

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.action.client import ClientGoalHandle
    from rclpy.action.server import ServerGoalHandle
    from rclpy.task import Future

Value = TypeVar('Value')
Goal = (
    Takeoff.Goal
    | NavigateTo.Goal
    | FollowTrajectory.Goal
    | OrbitPoint.Goal
    | Land.Goal
    | FlyMission.Goal
    | ExecuteWaypointMission.Goal
    | ExecuteFlightPlan.Goal
)
ActionType = (
    type[Takeoff]
    | type[NavigateTo]
    | type[FollowTrajectory]
    | type[OrbitPoint]
    | type[Land]
    | type[FlyMission]
    | type[ExecuteWaypointMission]
    | type[ExecuteFlightPlan]
)
Result = (
    Takeoff.Result
    | NavigateTo.Result
    | FollowTrajectory.Result
    | OrbitPoint.Result
    | Land.Result
    | FlyMission.Result
    | ExecuteFlightPlan.Result
    | ExecuteWaypointMission.Result
)
type JsonValue = str | int | float | bool | list[JsonValue] | dict[str, JsonValue] | None
OPERATIONS: dict[str, ActionType] = {
    'takeoff': Takeoff,
    'navigate_to': NavigateTo,
    'follow_trajectory': FollowTrajectory,
    'orbit_point': OrbitPoint,
    'land': Land,
    'fly_mission': FlyMission,
}
PUBLIC_ACTIONS: dict[str, ActionType] = {
    **{
        f'mission_execution/{name}': action
        for name, action in OPERATIONS.items()
        if name != 'fly_mission'
    },
    'execute_waypoint_mission': ExecuteWaypointMission,
    'mission_execution/execute_flight_plan': ExecuteFlightPlan,
}
DISCOVERY_TIMEOUT_SEC = 12.0
REPLY_TIMEOUT_SEC = 5.0
SETTLE_SEC = 0.2
ARTIFACT_DIRECTORY = Path(os.environ['BLIND_ARTIFACT_DIRECTORY'])


def point(east_m: float = 0.0, north_m: float = 0.0, up_m: float = 0.0) -> Point:
    """Construct an application ENU point."""
    return Point(x=float(east_m), y=float(north_m), z=float(up_m))


def geographic(
    latitude_deg: float = 0.0,
    longitude_deg: float = 0.0,
    altitude_amsl_m: float = 100.0,
) -> GeographicPosition:
    """Construct a WGS84/AMSL destination."""
    return GeographicPosition(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_amsl_m=altitude_amsl_m,
    )


def starting_state(*, airborne: bool = False) -> VehicleState:
    """Provide a finite, internally consistent telemetry baseline."""
    state = VehicleState()
    state.header.frame_id = 'local_enu'
    state.position = point(0.0, 0.0, 3.0 if airborne else 0.0)
    state.position_valid = True
    state.velocity_valid = True
    state.yaw_valid = True
    state.yaw_rad = 0.0
    state.armed = airborne
    state.offboard = airborne
    state.landed = not airborne
    state.global_reference_valid = True
    state.reference_altitude_amsl_m = 100.0
    state.global_position_valid = True
    state.geographic_position = geographic(altitude_amsl_m=103.0 if airborne else 100.0)
    state.local_frame_revision = 1
    return state


def fleet_plan() -> dict[str, JsonValue]:
    """Return the documented minimal version 2 schema."""
    return {
        'version': 2,
        'missions': [
            {
                'vehicle_ids': ['uav_1'],
                'frame_id': 'relative_enu',
                'takeoff_altitude_m': 3.0,
                'waypoint_acceptance_radius_m': 0.5,
                'path': {'type': 'waypoints', 'points': [[3.0, 0.0], [0.0, 3.0]]},
            }
        ],
    }


def valid_goals() -> dict[str, Goal]:
    """Provide valid wire shapes before individual mutations."""
    trajectory = FollowTrajectory.Goal(
        points=[
            TrajectoryPoint(position=point(0.0, 0.0, 3.0), time_from_start_sec=0.0),
            TrajectoryPoint(position=point(1.0, 0.0, 3.0), time_from_start_sec=1.0),
        ],
        timeout_sec=5.0,
    )
    return {
        'takeoff': Takeoff.Goal(altitude_m=3.0),
        'navigate_to': NavigateTo.Goal(
            frame_id='local_enu', target_position=point(1.0, 0.0, 3.0), timeout_sec=5.0
        ),
        'follow_trajectory': trajectory,
        'orbit_point': OrbitPoint.Goal(
            frame_id='local_enu',
            center_position=point(0.0, 0.0, 3.0),
            radius_m=2.0,
            tangential_speed_mps=1.0,
            revolutions=1.0,
            timeout_sec=60.0,
        ),
        'land': Land.Goal(timeout_sec=5.0),
        'fly_mission': FlyMission.Goal(
            starting_position=geographic(),
            takeoff_position=geographic(altitude_amsl_m=103.0),
            waypoint_positions=[geographic(0.00001, 0.0, 103.0)],
            position_tolerance_m=0.5,
            waypoint_timeout_sec=30.0,
            land_after_waypoints=True,
        ),
    }


class WireCase(unittest.TestCase):
    """Own one isolated node process and ROS participant per test method."""

    def setUp(self) -> None:
        """Start the probe executor and register bounded cleanup."""
        ARTIFACT_DIRECTORY.mkdir(parents=True, exist_ok=True)
        self.trace_file = (ARTIFACT_DIRECTORY / 'wire.jsonl').open('a', encoding='utf-8')
        self.addCleanup(self.trace_file.close)
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = Node('blind_probe', namespace='/blind')
        self.addCleanup(self.node.destroy_node)
        self.callback_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.addCleanup(self.stop_executor)
        self.processes: list[subprocess.Popen] = []
        self.addCleanup(self.stop_processes)
        self.clients: dict[str, ActionClient] = {}
        self.state = starting_state()
        self.publish_state = True
        self.state_publisher = self.node.create_publisher(VehicleState, 'vehicle_state', 10)
        self.state_timer = self.node.create_timer(
            0.02, self.publish_telemetry, callback_group=self.callback_group
        )

    def stop_executor(self) -> None:
        """Stop callbacks before destroying the ROS context."""
        self.executor.shutdown(timeout_sec=3.0)
        self.spin_thread.join(timeout=3.0)
        if self.spin_thread.is_alive():
            self.fail('ROS probe executor failed to shut down')

    def tearDown(self) -> None:
        """Check node and probe liveness before cleanup terminates processes."""
        self.healthy()

    def stop_processes(self) -> None:
        """Terminate and reap every node owned by this fixture."""
        for process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.0)

    def publish_telemetry(self) -> None:
        """Publish synthetic state until the stream is disabled."""
        if self.publish_state:
            self.state.header.stamp = self.node.get_clock().now().to_msg()
            self.state_publisher.publish(self.state)

    def trace(self, event: str, **fields: object) -> None:
        # Generated ROS messages have no shared Python base type. Conversion is confined here.
        """Flush a timestamped wire event to the scenario artifact."""
        converted: dict[str, object] = {}
        for key, value in fields.items():
            converted[key] = (
                message_to_ordereddict(value)
                if hasattr(value, 'get_fields_and_field_types')
                else value
            )
        self.trace_file.write(
            json.dumps(
                {'event': event, 'monotonic_sec': time.monotonic(), **converted},
                default=str,
            )
            + '\n'
        )
        self.trace_file.flush()

    def launch(self, package: str, **parameters: str | bool | float) -> subprocess.Popen:
        """Discover installed executable metadata without opening source files."""
        ros_cli = shutil.which('ros2')
        if ros_cli is None:
            self.fail('The ROS 2 CLI is not installed or sourced')
        listing = subprocess.run(  # noqa: S603 - Fixed arguments query installed package metadata.
            [ros_cli, 'pkg', 'executables', package],
            check=True,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
        candidates = [
            line.split()[1] for line in listing.stdout.splitlines() if line.split()[0] == package
        ]
        node_candidates = [name for name in candidates if 'node' in name or name == package]
        self.assertEqual(
            len(node_candidates),
            1,
            f'Ambiguous installed node executables: {candidates}',
        )
        executable = Path(get_package_prefix(package)) / 'lib' / package / node_candidates[0]
        command = [
            str(executable),
            '--ros-args',
            '-r',
            '__ns:=/blind',
        ]
        for name, value in parameters.items():
            command.extend(
                [
                    '-p',
                    f'{name}:={str(value).lower() if isinstance(value, bool) else value}',
                ]
            )
        log_file = (ARTIFACT_DIRECTORY / f'{package}-{len(self.processes)}.log').open(
            'w', encoding='utf-8'
        )
        self.addCleanup(log_file.close)
        # The executable comes from installed package metadata. Arguments do not use a shell.
        process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT)  # noqa: S603
        self.processes.append(process)
        self.trace('launch', command=command, pid=process.pid)
        return process

    def healthy(self) -> None:
        """Fail when an owned node exits during a wire assertion."""
        self.assertTrue(self.spin_thread.is_alive(), 'The ROS probe callback executor exited')
        for process in self.processes:
            self.assertIsNone(
                process.poll(),
                f'Node process {process.pid} exited: {process.returncode}',
            )

    def until(self, predicate: Callable[[], bool], timeout_sec: float = REPLY_TIMEOUT_SEC) -> None:
        """Poll one observable condition with a liveness check and deadline."""
        deadline_sec = time.monotonic() + timeout_sec
        while not predicate():
            self.healthy()
            if time.monotonic() >= deadline_sec:
                self.fail(f'Wire operation exceeded {timeout_sec} seconds')
            time.sleep(0.01)

    def pause(self, duration_sec: float = SETTLE_SEC) -> None:
        """Allow callback delivery while checking node liveness."""
        deadline_sec = time.monotonic() + duration_sec
        self.until(lambda: time.monotonic() >= deadline_sec, duration_sec + 1.0)

    def resolve(self, future: Future[Value], timeout_sec: float = REPLY_TIMEOUT_SEC) -> Value:
        """Wait for one ROS reply and surface transport exceptions."""
        self.until(future.done, timeout_sec)
        self.assertIsNone(future.exception())
        return future.result()

    def action_client(self, endpoint: str, action_type: ActionType) -> ActionClient:
        """Discover and retain one typed action client."""
        if endpoint not in self.clients:
            client = ActionClient(
                self.node, action_type, endpoint, callback_group=self.callback_group
            )
            self.until(client.server_is_ready, DISCOVERY_TIMEOUT_SEC)
            self.clients[endpoint] = client
        return self.clients[endpoint]

    def send(self, endpoint: str, action_type: ActionType, goal: Goal) -> ClientGoalHandle:
        """Record a goal and wait for its acceptance response."""
        self.trace('goal', endpoint=endpoint, goal=goal)
        handle = self.resolve(self.action_client(endpoint, action_type).send_goal_async(goal))
        self.trace('goal_response', endpoint=endpoint, accepted=handle.accepted)
        return handle

    def terminal(
        self, handle: ClientGoalHandle, timeout_sec: float = REPLY_TIMEOUT_SEC
    ) -> tuple[int, Result]:
        """Record a terminal result and check its action status."""
        response = self.resolve(handle.get_result_async(), timeout_sec)
        self.trace('result', status=response.status, result=response.result)
        self.assertIn(
            response.status,
            [
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_CANCELED,
            ],
        )
        return response.status, response.result

    def unsuccessful(
        self,
        endpoint: str,
        action_type: ActionType,
        goal: Goal,
        timeout_sec: float = REPLY_TIMEOUT_SEC,
    ) -> None:
        """Require rejection or a diagnostic unsuccessful result."""
        handle = self.send(endpoint, action_type, goal)
        if handle.accepted:
            status, result = self.terminal(handle, timeout_sec)
            self.assertFalse(result.success)
            self.assertNotEqual(status, GoalStatus.STATUS_SUCCEEDED)
            self.assertTrue(result.message, 'Failed accepted goals require a diagnostic')
        self.healthy()

    def planner(self) -> None:
        """Discover the planning service in the isolated namespace."""
        self.plan_client = self.node.create_client(
            PlanFlight,
            'mission_planner/plan_flight',
            callback_group=self.callback_group,
        )
        self.until(self.plan_client.service_is_ready, DISCOVERY_TIMEOUT_SEC)

    def plan(
        self, content: str, state: VehicleState | None = None, vehicle_id: str = 'uav_1'
    ) -> PlanFlight.Response:
        """Record one planning request and its response."""
        request = PlanFlight.Request(
            plan_json=content, vehicle_id=vehicle_id, starting_state=state or self.state
        )
        self.trace('plan_request', request=request)
        response = self.resolve(self.plan_client.call_async(request))
        self.trace('plan_response', response=response)
        return response

    def canary(self) -> PlanFlight.Response:
        """Require a finite two-waypoint route from the documented baseline."""
        response = self.plan(json.dumps(fleet_plan()), starting_state())
        self.assertTrue(response.success, response.message)
        self.assertEqual(len(response.waypoint_positions), 2)
        for destination in response.waypoint_positions:
            self.assertTrue(
                all(
                    math.isfinite(value)
                    for value in [
                        destination.latitude_deg,
                        destination.longitude_deg,
                        destination.altitude_amsl_m,
                    ]
                )
            )
            self.assertAlmostEqual(destination.altitude_amsl_m, 103.0, places=5)
        return response


class VehiclePeer:
    """Script internal action outcomes and expose dispatch/cancellation order."""

    def __init__(self, case: WireCase) -> None:
        """Serve internal operations with controllable outcomes."""
        self.case = case
        self.mode = 'succeed'
        self.release = threading.Event()
        self.admission_release = threading.Event()
        self.cancel_release = threading.Event()
        self.cancel_release.set()
        self.received: list[tuple[str, Goal]] = []
        self.events: list[tuple[str, str]] = []
        self.servers: list[ActionServer] = []
        for operation, action_type in OPERATIONS.items():
            server = ActionServer(
                case.node,
                action_type,
                f'vehicle_control/{operation}',
                execute_callback=self.execution(operation, action_type),
                goal_callback=self.admission(operation),
                cancel_callback=self.cancel,
                callback_group=case.callback_group,
            )
            self.servers.append(server)
        case.addCleanup(self.close)

    def close(self) -> None:
        """Release delayed callbacks and destroy synthetic action servers."""
        self.release.set()
        self.admission_release.set()
        self.cancel_release.set()
        for server in self.servers:
            server.destroy()

    def admission(self, operation: str) -> Callable[[Goal], GoalResponse]:
        """Capture child goals and optionally delay or reject admission."""

        def admit(goal: Goal) -> GoalResponse:
            self.received.append((operation, goal))
            self.events.append((operation, 'received'))
            self.case.trace('child_goal', operation=operation, goal=goal)
            if self.mode == 'late_accept':
                self.admission_release.wait(timeout=20.0)
            return GoalResponse.REJECT if self.mode == 'reject' else GoalResponse.ACCEPT

        return admit

    def cancel(self, handle: ServerGoalHandle) -> CancelResponse:
        """Record child cancellation without completing execution."""
        self.case.trace('child_cancel_request', goal_id=handle.goal_id)
        self.events.append(('child', 'cancel_requested'))
        return CancelResponse.ACCEPT

    def execution(
        self, operation: str, action_type: ActionType
    ) -> Callable[[ServerGoalHandle], Result]:
        """Select a child terminal outcome or bounded stalled execution."""

        def execute(handle: ServerGoalHandle) -> Result:
            result = action_type.Result()
            deadline_sec = time.monotonic() + 45.0
            while (
                operation != 'land'
                and self.mode in {'hold', 'cancel_abort', 'ignore_cancel', 'late_accept'}
                and not self.release.is_set()
            ):
                if (
                    handle.is_cancel_requested
                    and self.mode != 'ignore_cancel'
                    and self.cancel_release.is_set()
                ):
                    result.success = False
                    result.message = (
                        'synthetic child hold failure'
                        if self.mode == 'cancel_abort'
                        else 'synthetic confirmed hold'
                    )
                    if self.mode == 'cancel_abort':
                        handle.abort()
                    else:
                        handle.canceled()
                    self.events.append((operation, 'terminal'))
                    self.case.trace('child_terminal', operation=operation, result=result)
                    return result
                if time.monotonic() >= deadline_sec:
                    handle.abort()
                    result.message = 'synthetic peer deadline'
                    return result
                time.sleep(0.01)
            result.success = self.mode != 'abort'
            result.message = (
                'synthetic child success' if result.success else 'synthetic child failure'
            )
            if isinstance(result, FlyMission.Result):
                result.completed_waypoints = len(handle.request.waypoint_positions)
            if isinstance(result, Land.Result):
                result.landed = result.success
            if result.success:
                handle.succeed()
            else:
                handle.abort()
            self.events.append((operation, 'terminal'))
            self.case.trace('child_terminal', operation=operation, result=result)
            return result

        return execute


def valid_translation() -> PlanFlight.Response:
    """Provide one finite planner response for executor boundary tests."""
    return PlanFlight.Response(
        success=True,
        message='synthetic plan',
        takeoff_altitude_m=3.0,
        waypoint_positions=[geographic(0.00001, 0.0, 103.0)],
        position_tolerance_m=0.5,
    )
