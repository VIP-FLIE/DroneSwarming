# Copyright 2026 DroneSwarming maintainers.
"""Run real nodes against bounded, observable ROS and PX4 fault injectors."""

from __future__ import annotations

import copy
import json
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import IO, TYPE_CHECKING, cast

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
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

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.action.client import ClientGoalHandle
    from rclpy.action.server import ServerGoalHandle
    from rclpy.task import Future

type ActionType = type[
    ExecuteFlightPlan
    | ExecuteWaypointMission
    | FlyMission
    | FollowTrajectory
    | Land
    | NavigateTo
    | OrbitPoint
    | Takeoff
]
type Goal = (
    ExecuteFlightPlan.Goal
    | ExecuteWaypointMission.Goal
    | FlyMission.Goal
    | FollowTrajectory.Goal
    | Land.Goal
    | NavigateTo.Goal
    | OrbitPoint.Goal
    | Takeoff.Goal
)
type Result = (
    ExecuteFlightPlan.Result
    | ExecuteWaypointMission.Result
    | FlyMission.Result
    | FollowTrajectory.Result
    | Land.Result
    | NavigateTo.Result
    | OrbitPoint.Result
    | Takeoff.Result
)
type FeedbackMessage = (
    ExecuteFlightPlan.Impl.FeedbackMessage
    | ExecuteWaypointMission.Impl.FeedbackMessage
    | FlyMission.Impl.FeedbackMessage
    | FollowTrajectory.Impl.FeedbackMessage
    | Land.Impl.FeedbackMessage
    | NavigateTo.Impl.FeedbackMessage
    | OrbitPoint.Impl.FeedbackMessage
    | Takeoff.Impl.FeedbackMessage
)
type EventMessage = (
    VehicleState | TrajectorySetpoint | VehicleCommand | Goal | Result | FeedbackMessage
)
type JsonValue = bool | int | float | str | list[JsonValue] | dict[str, JsonValue] | None
POLL_INTERVAL_SEC = 0.01
DISCOVERY_TIMEOUT_SEC = 8.0
NAMESPACE = '/seeing'
PX4_PREFIX = '/seeing/fmu'
OFFBOARD_CUSTOM_MODE = 6.0
STARTUP_STATE_PUBLICATIONS = 15


@dataclass(frozen=True)
class ActionOutcome:
    """Retain the ROS terminal status alongside its typed application result."""

    status: int
    result: Result


def verify_isolation() -> None:
    """Refuse synthetic flight-enabled tests outside a networkless Docker container."""
    interfaces: set[str] = {path.name for path in Path('/sys/class/net').iterdir()}
    if not Path('/.dockerenv').exists() or interfaces != {'lo'}:
        message = 'Run scripts/test-seeing-torture; tests require Docker --network none'
        raise RuntimeError(message)
    if os.environ.get('ROS_DOMAIN_ID') != '83' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        message = 'Seeing torture requires ROS_DOMAIN_ID=83 and ROS_LOCALHOST_ONLY=1'
        raise RuntimeError(message)


def wait_for(predicate: Callable[[], bool], timeout_sec: float = 3.0) -> None:
    """Fail with a bounded deadline while executor callbacks remain runnable."""
    deadline_sec: float = time.monotonic() + timeout_sec
    while time.monotonic() < deadline_sec:
        if predicate():
            return
        time.sleep(POLL_INTERVAL_SEC)
    if not predicate():
        message: str = f'Condition did not become true within {timeout_sec} seconds'
        raise AssertionError(message)


def resolve[FutureValue](future: Future[FutureValue], timeout_sec: float = 3.0) -> FutureValue:
    """Read a completed ROS future and propagate callback exceptions."""
    wait_for(future.done, timeout_sec)
    # rclpy exposes an untyped result at the generated ROS message boundary.
    return cast('FutureValue', future.result())


def resolve_batch[FutureValue](
    futures: list[Future[FutureValue]],
    timeout_sec: float = 5.0,
) -> list[FutureValue]:
    """Bound the whole overload burst and report how many replies remained unresolved."""
    try:
        wait_for(lambda: all(future.done() for future in futures), timeout_sec)
    except AssertionError as error:
        completed: int = sum(future.done() for future in futures)
        message = f'{completed}/{len(futures)} burst responses arrived within {timeout_sec} seconds'
        raise AssertionError(message) from error
    return [resolve(future) for future in futures]


def starting_state(*, airborne: bool = False) -> VehicleState:
    """Supply one internally consistent geographic and local snapshot."""
    state = VehicleState()
    state.header.frame_id = 'local_enu'
    state.position_valid = True
    state.velocity_valid = True
    state.yaw_valid = True
    state.global_position_valid = True
    state.global_reference_valid = True
    state.local_frame_revision = 1
    state.reference_latitude_deg = 47.0
    state.reference_longitude_deg = 8.0
    state.reference_altitude_amsl_m = 500.0
    state.geographic_position = GeographicPosition(
        latitude_deg=47.0, longitude_deg=8.0, altitude_amsl_m=500.0
    )
    state.armed = airborne
    state.offboard = airborne
    state.landed = not airborne
    return state


def navigation_goal(east_m: float = 3.0, up_m: float = 3.0) -> NavigateTo.Goal:
    """Construct a local navigation operation with a short execution deadline."""
    return NavigateTo.Goal(
        frame_id='local_enu',
        target_position=Point(x=east_m, y=0.0, z=up_m),
        timeout_sec=1.5,
        position_tolerance_m=0.2,
    )


def trajectory_goal() -> FollowTrajectory.Goal:
    """Request a bounded stationary trajectory that remains active during faults."""
    return FollowTrajectory.Goal(
        points=[
            TrajectoryPoint(position=Point(z=3.0), time_from_start_sec=0.0),
            TrajectoryPoint(position=Point(z=3.0), time_from_start_sec=2.0),
        ],
        timeout_sec=3.5,
    )


class Harness:
    """Own real-node processes, a concurrent ROS peer, and per-case evidence."""

    def __init__(self) -> None:
        """Create an executor and retain evidence until all real-node processes exit."""
        verify_isolation()
        rclpy.init()
        self.node = Node('seeing_fault_injector', namespace=NAMESPACE)
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)
        self.directory = Path(os.environ['SEEING_CASE_DIRECTORY'])
        self.events: IO[str] = (self.directory / 'events.jsonl').open('w')
        self.lock = threading.RLock()
        self.processes: list[subprocess.Popen[bytes]] = []
        self.logs: list[IO[bytes]] = []
        self.clients: list[ActionClient] = []
        self.servers: list[ControlledAction] = []
        self.states: list[VehicleState] = []
        self.setpoints: list[TrajectorySetpoint] = []
        self.commands: list[VehicleCommand] = []
        self.stopping = threading.Event()
        self.spin_errors: list[BaseException] = []
        self.spin_thread = threading.Thread(target=self.spin, daemon=True)
        self.spin_thread.start()

    def spin(self) -> None:
        """Keep callback failures visible to assertions and the case supervisor."""
        try:
            self.executor.spin()
        except BaseException as error:
            self.spin_errors.append(error)

    def record(self, kind: str, message: EventMessage) -> None:
        """Persist timestamped wire evidence before a case process can terminate."""
        with self.lock:
            self.events.write(
                json.dumps(
                    {
                        'monotonic_sec': time.monotonic(),
                        'kind': kind,
                        'message': message_to_ordereddict(message),
                    }
                )
                + '\n'
            )
            self.events.flush()

    def launch(self, package: str, parameters: dict[str, str] | None = None) -> None:
        """Start an installed node with all DDS endpoints in the seeing namespace."""
        executable = f'{package}_node'
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        arguments: list[str] = [str(path), '--ros-args', '-r', f'__ns:={NAMESPACE}']
        for name, value in (parameters or {}).items():
            arguments.extend(['-p', f'{name}:={value}'])
        log = (self.directory / f'{package}-{len(self.processes)}.log').open('wb')
        self.logs.append(log)
        self.processes.append(subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT))

    def mission_execution(self, *, airborne: bool = False) -> StateSource:
        """Launch the executor with injectable application telemetry and short response limits."""
        source = StateSource(self, airborne=airborne)
        self.launch(
            'mission_execution',
            {
                'vehicle_id': 'uav_1',
                'telemetry_timeout_sec': '0.3',
                'starting_position_timeout_sec': '0.5',
                'action_server_timeout_sec': '0.5',
            },
        )
        wait_for(lambda: source.publisher.get_subscription_count() > 0, DISCOVERY_TIMEOUT_SEC)
        source.publications = 0
        wait_for(lambda: source.publications >= STARTUP_STATE_PUBLICATIONS)
        return source

    def vehicle_control(self, *, enabled: bool = True) -> Px4Source:
        """Enable commands only against synthetic PX4 inside the networkless container."""
        source = Px4Source(self)
        self.launch(
            'vehicle_control',
            {
                'enable_flight_commands': str(enabled).lower(),
                'px4_topic_prefix': PX4_PREFIX,
                'mission_connection_url': 'udpin://127.0.0.1:14683',
                'telemetry_timeout_sec': '0.3',
                'command_ack_timeout_sec': '0.15',
                'transition_timeout_sec': '0.3',
                'takeoff_timeout_sec': '3.0',
            },
        )
        wait_for(lambda: bool(self.states), DISCOVERY_TIMEOUT_SEC)
        wait_for(lambda: self.states[-1].position_valid, DISCOVERY_TIMEOUT_SEC)
        return source

    def client(self, action_type: ActionType, name: str) -> ActionClient:
        """Discover an action and retain its client until process cleanup."""
        client = ActionClient(self.node, action_type, name, callback_group=self.group)
        self.clients.append(client)
        if not client.wait_for_server(timeout_sec=DISCOVERY_TIMEOUT_SEC):
            self.assert_alive()
            message: str = f'Action server unavailable: {name}'
            raise RuntimeError(message)
        return client

    def send(self, client: ActionClient, goal: Goal) -> ClientGoalHandle:
        """Record ingress and await the server's acceptance decision."""
        self.record('goal', goal)

        def record_feedback(message: FeedbackMessage) -> None:
            self.record('feedback', message)

        return resolve(client.send_goal_async(goal, feedback_callback=record_feedback))

    def result(self, handle: ClientGoalHandle, timeout_sec: float = 3.0) -> ActionOutcome:
        """Preserve the public terminal result and status in the wire-evidence log."""
        response = resolve(handle.get_result_async(), timeout_sec)
        # Generated action clients erase the concrete result type at this boundary.
        result = cast('Result', response.result)
        self.record(f'public_result_status_{response.status}', result)
        self.assert_alive()
        return ActionOutcome(status=response.status, result=result)

    def assert_alive(self) -> None:
        """Detect crashed real nodes and failed injector callbacks."""
        if self.spin_errors:
            message = 'Fault injector executor failed'
            raise RuntimeError(message) from self.spin_errors[0]
        for process in self.processes:
            if process.poll() is not None:
                message = f'Real node exited unexpectedly: {process.returncode}'
                raise AssertionError(message)

    def close(self) -> None:
        """Terminate real nodes before stopping their peers, including faulted actions."""
        self.stopping.set()
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in self.processes:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        for client in self.clients:
            client.destroy()
        for server in self.servers:
            server.server.destroy()
        self.node.destroy_node()
        rclpy.shutdown()
        self.events.close()
        for log in self.logs:
            log.close()


class StateSource:
    """Publish application telemetry with independently controlled validity and delivery."""

    def __init__(self, harness: Harness, *, airborne: bool) -> None:
        """Create a periodic application-state publisher for executor fault cases."""
        self.harness = harness
        self.state = starting_state(airborne=airborne)
        self.enabled = True
        self.publications = 0
        self.publisher = harness.node.create_publisher(VehicleState, 'vehicle_state', 10)
        self.timer = harness.node.create_timer(0.02, self.publish, callback_group=harness.group)

    def publish(self) -> None:
        """Stamp each receipt while preserving deliberately malformed payload fields."""
        with self.harness.lock:
            if self.enabled:
                self.state.header.stamp = self.harness.node.get_clock().now().to_msg()
                self.publisher.publish(self.state)
                self.publications += 1
                self.harness.record('injected_vehicle_state', self.state)


class Px4Source:
    """Inject PX4 topics and observe commands without PX4, Gazebo, or MAVLink peers."""

    def __init__(self, harness: Harness) -> None:
        """Create synthetic telemetry publishers and actual PX4-command observers."""
        self.harness = harness
        self.local = VehicleLocalPosition(
            timestamp=1,
            timestamp_sample=1,
            xy_valid=True,
            z_valid=True,
            v_xy_valid=True,
            v_z_valid=True,
            z=-3.0,
            heading_good_for_control=True,
            xy_global=True,
            z_global=True,
            ref_lat=47.0,
            ref_lon=8.0,
            ref_alt=500.0,
            ref_timestamp=1,
        )
        self.status = VehicleStatus(
            arming_state=VehicleStatus.ARMING_STATE_ARMED,
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            vehicle_type=VehicleStatus.VEHICLE_TYPE_ROTARY_WING,
        )
        self.land = VehicleLandDetected(landed=False)
        self.publish_position = True
        self.publish_status = True
        self.publish_land = True
        self.acknowledge_commands = True
        self.acknowledgement_target = 255
        self.advance_timestamps = True
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.position_publisher = harness.node.create_publisher(
            VehicleLocalPosition, f'{PX4_PREFIX}/out/vehicle_local_position_v1', qos
        )
        self.status_publisher = harness.node.create_publisher(
            VehicleStatus, f'{PX4_PREFIX}/out/vehicle_status_v1', qos
        )
        self.land_publisher = harness.node.create_publisher(
            VehicleLandDetected, f'{PX4_PREFIX}/out/vehicle_land_detected', qos
        )
        self.acknowledgement_publisher = harness.node.create_publisher(
            VehicleCommandAck, f'{PX4_PREFIX}/out/vehicle_command_ack', qos
        )
        self.setpoint_subscription = harness.node.create_subscription(
            TrajectorySetpoint,
            f'{PX4_PREFIX}/in/trajectory_setpoint',
            self.setpoint,
            qos,
            callback_group=harness.group,
        )
        self.command_subscription = harness.node.create_subscription(
            VehicleCommand,
            f'{PX4_PREFIX}/in/vehicle_command',
            self.command,
            qos,
            callback_group=harness.group,
        )
        self.state_subscription = harness.node.create_subscription(
            VehicleState,
            'vehicle_state',
            self.state,
            10,
            callback_group=harness.group,
        )
        self.timer = harness.node.create_timer(0.02, self.publish, callback_group=harness.group)

    def publish(self) -> None:
        """Allow independent stream loss, stale sample replay, and estimator resets."""
        with self.harness.lock:
            if self.advance_timestamps:
                self.local.timestamp += 20_000
                self.local.timestamp_sample = self.local.timestamp
            if self.publish_position:
                self.position_publisher.publish(self.local)
            if self.publish_status:
                self.status_publisher.publish(self.status)
            if self.publish_land:
                self.land_publisher.publish(self.land)

    def setpoint(self, message: TrajectorySetpoint) -> None:
        """Retain the actual float32 command emitted at the PX4 boundary."""
        with self.harness.lock:
            self.harness.setpoints.append(copy.deepcopy(message))
            self.harness.record('px4_setpoint', message)

    def state(self, message: VehicleState) -> None:
        """Observe real adapter freshness and coordinate conversion."""
        with self.harness.lock:
            self.harness.states.append(copy.deepcopy(message))
            self.harness.record('vehicle_state', message)

    def command(self, message: VehicleCommand) -> None:
        """Confirm transitions only when the scenario permits acknowledgements."""
        with self.harness.lock:
            self.harness.commands.append(copy.deepcopy(message))
            self.harness.record('px4_command', message)
            if not self.acknowledge_commands:
                return
            self.acknowledgement_publisher.publish(
                VehicleCommandAck(
                    command=message.command,
                    result=VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
                    target_system=self.acknowledgement_target,
                    target_component=1,
                )
            )
            if message.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                self.status.nav_state = (
                    VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    if message.param2 == OFFBOARD_CUSTOM_MODE
                    else VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                )
            elif message.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
                self.status.arming_state = (
                    VehicleStatus.ARMING_STATE_ARMED
                    if message.param1 == 1.0
                    else VehicleStatus.ARMING_STATE_DISARMED
                )
            elif message.command == VehicleCommand.VEHICLE_CMD_DO_ORBIT:
                self.status.nav_state = VehicleStatus.NAVIGATION_STATE_ORBIT


class ControlledAction:
    """Supply delayed acceptance, contradictory results, and non-cooperative cancellation."""

    def __init__(self, harness: Harness, action_type: ActionType, name: str) -> None:
        """Create one controllable action endpoint beneath vehicle_control."""
        self.harness = harness
        self.action_type = action_type
        self.response_delay_sec = 0.0
        self.reject = False
        self.hold = False
        self.ignore_cancel = False
        self.success = True
        self.status = 'succeed'
        self.completed_waypoints = 1
        self.received: list[Goal] = []
        self.cancellations = 0
        self.started = threading.Event()
        self.release = threading.Event()
        self.server = ActionServer(
            harness.node,
            action_type,
            f'vehicle_control/{name}',
            execute_callback=self.execute,
            goal_callback=self.accept,
            cancel_callback=self.cancel,
            callback_group=harness.group,
        )
        harness.servers.append(self)

    def accept(self, goal: Goal) -> GoalResponse:
        """Delay goal responses independently of parent command deadlines."""
        with self.harness.lock:
            self.received.append(copy.deepcopy(goal))
            self.harness.record('child_goal', goal)
        self.harness.stopping.wait(self.response_delay_sec)
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def cancel(self, _handle: ServerGoalHandle) -> CancelResponse:
        """Record cancellation requests even when the child refuses to terminate."""
        self.cancellations += 1
        return CancelResponse.REJECT if self.ignore_cancel else CancelResponse.ACCEPT

    def execute(self, handle: ServerGoalHandle) -> Result:
        """Return an independently chosen ROS status and application success field."""
        self.started.set()
        while self.hold and not self.release.is_set() and not self.harness.stopping.is_set():
            if handle.is_cancel_requested and not self.ignore_cancel:
                handle.canceled()
                return self.action_type.Result(success=False, message='Synthetic Hold confirmed')
            time.sleep(POLL_INTERVAL_SEC)
        result: Result = self.action_type.Result(
            success=self.success, message='Injected child result'
        )
        if isinstance(result, FlyMission.Result):
            result.completed_waypoints = self.completed_waypoints
        if self.status == 'abort' or self.harness.stopping.is_set():
            handle.abort()
        else:
            handle.succeed()
        self.harness.record('child_result', result)
        return result
