"""Exercise real application nodes through an isolated synthetic PX4 boundary."""

from __future__ import annotations

import copy
import hashlib
import heapq
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
import uuid
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Protocol, TextIO, TypeVar, cast

import rclpy
from ament_index_python.packages import get_package_prefix
from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rosidl_runtime_py.convert import message_to_yaml
from swarm_interfaces.action import (
    ExecuteFlightPlan,
    ExecuteWaypointMission,
    FollowTrajectory,
    Land,
    NavigateTo,
    OrbitPoint,
    Takeoff,
)
from swarm_interfaces.msg import VehicleState
from swarm_interfaces.srv import PlanFlight

type JsonValue = None | bool | int | float | str | list[JsonValue] | dict[str, JsonValue]
type Goal = (
    Takeoff.Goal
    | NavigateTo.Goal
    | FollowTrajectory.Goal
    | OrbitPoint.Goal
    | Land.Goal
    | ExecuteWaypointMission.Goal
    | ExecuteFlightPlan.Goal
)
type InputMessage = (
    VehicleLocalPosition | VehicleStatus | VehicleLandDetected | BatteryStatus | VehicleCommandAck
)
type Action = (
    type[Takeoff]
    | type[NavigateTo]
    | type[FollowTrajectory]
    | type[OrbitPoint]
    | type[Land]
    | type[ExecuteWaypointMission]
    | type[ExecuteFlightPlan]
)

ResultT = TypeVar('ResultT')
POLL_SEC: float = 0.01
TELEMETRY_TIMEOUT_SEC: float = 0.3
RESPONSE_TIMEOUT_SEC: float = 4.0
OBSERVATION_SEC: float = 0.7
MAXIMUM_EVENTS: int = 50_000
RESOURCE_SAMPLE_SEC: float = 0.25


class RosMessage(Protocol):
    """Describe the generated ROS serialization boundary used by diagnostic logging."""

    def get_fields_and_field_types(self) -> dict[str, str]: ...


class FeedbackEnvelope(Protocol):
    feedback: RosMessage


class ActionResult(Protocol):
    success: bool
    message: str


class ActionResponse(Protocol):
    status: int
    result: ActionResult


@dataclass
class LinkFault:
    """Configure bounded delivery impairment on one synthetic telemetry or ACK stream."""

    paused: bool = False
    drop_probability: float = 0.0
    delay_sec: float = 0.0
    jitter_sec: float = 0.0
    freeze_timestamp: bool = False
    timestamp_offset_us: int = 0


class Peer(Node):
    """Own synthetic telemetry, child processes, and a timestamped diagnostic timeline."""

    def __init__(self, directory: Path, seed: int) -> None:
        self.directory: Path = directory
        self.started_at: float = time.monotonic()
        self.namespace: str = '/torture_' + uuid.uuid4().hex
        self.prefix: str = self.namespace + '/fmu'
        super().__init__('fault_peer', namespace=self.namespace)
        self.random = random.Random(seed)
        self.events: TextIO = (directory / 'events.jsonl').open('w', encoding='utf-8')
        self.event_count: int = 0
        self.recent: deque[dict[str, JsonValue]] = deque(maxlen=40)
        self.processes: dict[str, subprocess.Popen[bytes]] = {}
        self.expected_dead: set[str] = set()
        self.cleanup_errors: list[str] = []
        self.action_clients: dict[str, ActionClient] = {}
        self.counts: Counter[str] = Counter()
        self.commands: deque[VehicleCommand] = deque(maxlen=512)
        self.setpoints: deque[TrajectorySetpoint] = deque(maxlen=512)
        self.heartbeat_times: deque[float] = deque(maxlen=4096)
        self.latest_state: VehicleState | None = None
        self.position = VehicleLocalPosition()
        self.position.x, self.position.y, self.position.z = 5.0, 7.0, -3.0
        self.position.xy_valid = self.position.z_valid = True
        self.position.v_xy_valid = self.position.v_z_valid = True
        self.position.heading_good_for_control = True
        self.position.xy_global = self.position.z_global = True
        self.position.ref_lat, self.position.ref_lon, self.position.ref_alt = 47.0, 8.0, 500.0
        self.position.ref_timestamp = 1
        self.status = VehicleStatus()
        self.status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.status.vehicle_type = VehicleStatus.VEHICLE_TYPE_ROTARY_WING
        self.status.system_id = 1
        self.status.component_id = 1
        self.status.pre_flight_checks_pass = True
        self.land = VehicleLandDetected()
        self.battery = BatteryStatus()
        self.battery.remaining = 0.9
        self.follow_setpoints: bool = False
        self.apply_commands: bool = True
        self.ack_result: int = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        self.command_results: dict[int, int] = {}
        self.ack_noise: bool = False
        self.ack_target_system: int = 255
        self.landing_complete_at: float | None = None
        self.delivery_sequence: int = 0
        self.pending: list[tuple[float, int, str, InputMessage]] = []
        self.links: dict[str, LinkFault] = {
            name: LinkFault() for name in ('position', 'status', 'land', 'battery', 'ack')
        }
        self.qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.telemetry_publishers: dict[str, Publisher] = {
            'position': self.create_publisher(
                VehicleLocalPosition, self.prefix + '/out/vehicle_local_position_v1', self.qos
            ),
            'status': self.create_publisher(
                VehicleStatus, self.prefix + '/out/vehicle_status_v1', self.qos
            ),
            'land': self.create_publisher(
                VehicleLandDetected, self.prefix + '/out/vehicle_land_detected', self.qos
            ),
            'battery': self.create_publisher(
                BatteryStatus, self.prefix + '/out/battery_status_v1', self.qos
            ),
            'ack': self.create_publisher(
                VehicleCommandAck, self.prefix + '/out/vehicle_command_ack', self.qos
            ),
        }
        self.create_subscription(
            VehicleCommand, self.prefix + '/in/vehicle_command', self.on_command, self.qos
        )
        self.create_subscription(
            TrajectorySetpoint, self.prefix + '/in/trajectory_setpoint', self.on_setpoint, self.qos
        )
        self.create_subscription(
            OffboardControlMode,
            self.prefix + '/in/offboard_control_mode',
            self.on_heartbeat,
            self.qos,
        )
        self.create_subscription(VehicleState, 'vehicle_state', self.on_state, 10)
        self.create_timer(0.05, self.tick)
        self.resource_sample_at: float = 0.0
        self.peak_rss_kib: Counter[str] = Counter()
        self.event(
            'configuration',
            seed=seed,
            namespace=self.namespace,
            prefix=self.prefix,
            telemetry_timeout_sec=TELEMETRY_TIMEOUT_SEC,
        )

    def event(self, kind: str, **fields: JsonValue) -> None:
        """Flush each event so a killed or crashed case retains its last observations."""
        if self.event_count >= MAXIMUM_EVENTS:
            raise RuntimeError('Diagnostic event budget exceeded; stopping the case')
        record: dict[str, JsonValue] = {
            'sequence': self.event_count,
            'elapsed_sec': time.monotonic() - self.started_at,
            'monotonic_ns': time.monotonic_ns(),
            'wall_time_ns': time.time_ns(),
            'kind': kind,
            **fields,
        }
        self.events.write(json.dumps(record, allow_nan=False) + '\n')
        self.events.flush()
        self.recent.append(record)
        self.event_count += 1
        if kind in ('fault', 'process_fault', 'check', 'deadline_exceeded', 'phase'):
            self.get_logger().info(json.dumps(record, allow_nan=False))

    def check(self, condition: bool, contract: str, actual: JsonValue, expected: JsonValue) -> None:
        """Write measured assertion values before raising a contract failure."""
        self.event('check', contract=contract, actual=actual, expected=expected, passed=condition)
        if not condition:
            raise AssertionError(f'{contract}: actual={actual!r}, expected={expected!r}')

    def message(self, kind: str, message: RosMessage) -> None:
        """Preserve every field, including NaN, through the generated ROS YAML serializer."""
        self.counts[kind] += 1
        self.event(kind, ros_type=type(message).__name__, payload_yaml=message_to_yaml(message))

    def fault(self, stream: str, fault: LinkFault) -> None:
        """Apply and record a fault before the next stream sample is scheduled."""
        self.links[stream] = fault
        self.event('fault', stream=stream, configuration=cast(dict[str, JsonValue], asdict(fault)))

    def ground(self) -> None:
        """Initialize a disarmed landed vehicle without moving any real vehicle."""
        self.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.status.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.land.landed = True
        self.position.z = 0.0
        self.event('ground_state')

    def start(self, enabled: bool = True, planner: bool = True) -> None:
        """Launch existing executables with per-case topics and explicit startup parameters."""
        settings: dict[str, dict[str, bool | int | float | str]] = {
            'vehicle_control': {
                'enable_flight_commands': enabled,
                'px4_topic_prefix': self.prefix,
                'px4_system_id': 1,
                'telemetry_timeout_sec': TELEMETRY_TIMEOUT_SEC,
                'command_ack_timeout_sec': 0.2,
                'transition_timeout_sec': 0.5,
                'takeoff_timeout_sec': 3.0,
                'default_navigation_timeout_sec': 3.0,
            },
            'mission_execution': {
                'vehicle_id': 'torture_vehicle',
                'telemetry_timeout_sec': TELEMETRY_TIMEOUT_SEC,
                'action_server_timeout_sec': 1.0,
                'starting_position_timeout_sec': 1.0,
            },
        }
        if planner:
            settings['mission_planner'] = {}
        for package, parameters in settings.items():
            executable: str = (
                'mission_planner_node' if package == 'mission_planner' else package + '_node'
            )
            binary: Path = Path(get_package_prefix(package)) / 'lib' / package / executable
            parameter_file: Path = self.directory / (package + '-parameters.json')
            parameter_file.write_text(
                json.dumps({'/**': {'ros__parameters': parameters}}), encoding='utf-8'
            )
            log_directory: Path = self.directory / package
            log_directory.mkdir()
            environment: dict[str, str] = dict(os.environ)
            environment['ROS_LOG_DIR'] = str(log_directory)
            environment['ASAN_OPTIONS'] = 'detect_leaks=1:log_path=' + str(log_directory / 'asan')
            environment['UBSAN_OPTIONS'] = 'print_stacktrace=1:log_path=' + str(
                log_directory / 'ubsan'
            )
            arguments: list[str] = [
                str(binary),
                '--ros-args',
                '-r',
                '__ns:=' + self.namespace,
                '--params-file',
                str(parameter_file),
                '--log-level',
                'info',
                '--log-level',
                self.namespace[1:] + '.' + package + ':=debug',
                '--disable-stdout-logs',
                '--enable-external-lib-logs',
            ]
            if package == 'mission_planner':
                arguments = [
                    sys.executable,
                    str(Path(__file__).with_name('python_node.py')),
                    *arguments,
                ]
            process = subprocess.Popen(
                arguments, env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            self.processes[package] = process
            self.event(
                'process_started',
                package=package,
                pid=process.pid,
                arguments=arguments,
                binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
            )
        self.wait(
            lambda: all(
                self.telemetry_publishers[name].get_subscription_count() > 0
                for name in ('position', 'status', 'land')
            ),
            'PX4 subscriptions',
            6.0,
        )
        self.wait(
            lambda: self.latest_state is not None and self.latest_state.position_valid,
            'initial valid application state',
            3.0,
        )
        self.client('navigate_to', NavigateTo)
        self.wait(
            lambda: self.count_subscribers('vehicle_state') >= 2,
            'observer and mission state subscriptions',
            3.0,
        )
        if planner:
            planning_client = self.create_client(PlanFlight, 'mission_planner/plan_flight')
            self.wait(planning_client.service_is_ready, 'planner initialization', 6.0)
        self.wait(
            lambda: (
                self.latest_state is not None
                and self.latest_state.position_valid
                and self.latest_state.landed == self.land.landed
                and self.latest_state.armed
                == (self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
                and self.latest_state.offboard
                == (self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            ),
            'complete initial control snapshot',
            3.0,
        )
        self.pump(0.3)
        self.event('phase', name='ready_for_fault_injection')

    def schedule(self, stream: str, message: InputMessage) -> None:
        """Queue a copy so delays, jitter, and replay do not mutate earlier samples."""
        fault: LinkFault = self.links[stream]
        if fault.paused or self.random.random() < fault.drop_probability:
            self.counts['dropped_' + stream] += 1
            self.event('sample_dropped', stream=stream, source_timestamp_us=message.timestamp)
            return
        delay_sec: float = max(
            0.0, fault.delay_sec + self.random.uniform(-fault.jitter_sec, fault.jitter_sec)
        )
        self.delivery_sequence += 1
        heapq.heappush(
            self.pending,
            (time.monotonic() + delay_sec, self.delivery_sequence, stream, copy.deepcopy(message)),
        )

    def tick(self) -> None:
        """Advance the protocol peer; this peer does not model vehicle physics or PX4 failsafes."""
        now: float = time.monotonic()
        if self.landing_complete_at is not None and now >= self.landing_complete_at:
            self.land.landed = True
            self.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
            self.position.z = 0.0
            self.landing_complete_at = None
            self.event('synthetic_touchdown')
        samples: dict[str, InputMessage] = {
            'position': self.position,
            'status': self.status,
            'land': self.land,
            'battery': self.battery,
        }
        for stream, message in samples.items():
            fault: LinkFault = self.links[stream]
            if not fault.freeze_timestamp:
                message.timestamp = max(0, time.monotonic_ns() // 1000 + fault.timestamp_offset_us)
            self.schedule(stream, message)
        while self.pending and self.pending[0][0] <= now:
            _, _, stream, message = heapq.heappop(self.pending)
            self.telemetry_publishers[stream].publish(message)
            self.message('injected_' + stream, message)

    def on_command(self, command: VehicleCommand) -> None:
        """Record commands and optionally supply a protocol-level transition and ACK."""
        self.commands.append(command)
        self.message('command', command)
        result: int = self.command_results.get(command.command, self.ack_result)
        if self.apply_commands and result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            if command.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
                self.status.nav_state = (
                    VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    if command.param2 == 6.0
                    else VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                )
            elif command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
                self.status.arming_state = (
                    VehicleStatus.ARMING_STATE_ARMED
                    if command.param1 > 0.0
                    else VehicleStatus.ARMING_STATE_DISARMED
                )
            elif command.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
                self.status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                self.landing_complete_at = time.monotonic() + 0.25
            elif command.command == VehicleCommand.VEHICLE_CMD_DO_ORBIT:
                self.status.nav_state = VehicleStatus.NAVIGATION_STATE_ORBIT
        acknowledgement = VehicleCommandAck()
        acknowledgement.timestamp = time.monotonic_ns() // 1000
        acknowledgement.command = command.command
        acknowledgement.result = result
        acknowledgement.target_system = self.ack_target_system
        acknowledgement.target_component = 1
        self.schedule('ack', acknowledgement)
        if self.ack_noise:
            unrelated: VehicleCommandAck = copy.deepcopy(acknowledgement)
            unrelated.command = (
                VehicleCommand.VEHICLE_CMD_NAV_LAND
                if command.command != VehicleCommand.VEHICLE_CMD_NAV_LAND
                else VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            )
            self.schedule('ack', unrelated)

    def on_setpoint(self, setpoint: TrajectorySetpoint) -> None:
        """Observe requested motion without treating a setpoint as measured flight performance."""
        self.setpoints.append(setpoint)
        self.message('setpoint', setpoint)
        if self.follow_setpoints and all(math.isfinite(value) for value in setpoint.position):
            self.position.x, self.position.y, self.position.z = (
                float(value) for value in setpoint.position
            )
            if (
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.position.z < -0.5
            ):
                self.land.landed = False

    def on_heartbeat(self, message: OffboardControlMode) -> None:
        self.heartbeat_times.append(time.monotonic())
        self.message('heartbeat', message)

    def on_state(self, state: VehicleState) -> None:
        self.latest_state = state
        self.message('vehicle_state', state)

    def pump(self, duration_sec: float) -> None:
        """Spin bounded observations and fail on unexpected node exits."""
        deadline: float = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=POLL_SEC)
            for name, process in self.processes.items():
                if name not in self.expected_dead and process.poll() is not None:
                    raise RuntimeError(
                        f'{name} exited unexpectedly with code {process.returncode}; inspect {self.directory / name}'
                    )
            if time.monotonic() >= self.resource_sample_at:
                self.resource_sample_at = time.monotonic() + RESOURCE_SAMPLE_SEC
                self.event('resources', processes=self.resources())

    def resources(self) -> dict[str, JsonValue]:
        """Record process RSS and threads without requiring psutil or another dependency."""
        result: dict[str, JsonValue] = {}
        for name, process in self.processes.items():
            try:
                fields: dict[str, str] = dict(
                    line.split(':', 1)
                    for line in Path(f'/proc/{process.pid}/status').read_text().splitlines()
                    if ':' in line
                )
                cpu_fields: list[str] = (
                    Path(f'/proc/{process.pid}/stat').read_text().split(') ', 1)[1].split()
                )
                result[name] = {
                    'pid': process.pid,
                    'rss_kib': int(fields.get('VmRSS', '0 kB').split()[0]),
                    'threads': int(fields.get('Threads', '0')),
                    'state': fields.get('State', '').strip(),
                    'cpu_time_sec': (int(cpu_fields[11]) + int(cpu_fields[12]))
                    / os.sysconf('SC_CLK_TCK'),
                }
                self.peak_rss_kib[name] = max(
                    self.peak_rss_kib[name], int(fields.get('VmRSS', '0 kB').split()[0])
                )
            except FileNotFoundError:
                result[name] = {'pid': process.pid, 'exit_code': process.poll()}
        return result

    def wait(self, predicate: Callable[[], bool], description: str, timeout_sec: float) -> None:
        """Give every expected response a local deadline and a named diagnostic failure."""
        deadline: float = time.monotonic() + timeout_sec
        while not predicate() and time.monotonic() < deadline:
            self.pump(POLL_SEC)
        if not predicate():
            self.event('deadline_exceeded', operation=description, timeout_sec=timeout_sec)
            raise AssertionError(f'{description} did not complete within {timeout_sec:.2f} s')

    def resolve(
        self, future: Future[ResultT], description: str, timeout_sec: float = RESPONSE_TIMEOUT_SEC
    ) -> ResultT:
        self.wait(future.done, description, timeout_sec)
        result: ResultT | None = future.result()
        if result is None:
            raise RuntimeError(f'{description} returned no value')
        return result

    def client(self, operation: str, action: Action, internal: bool = False) -> ActionClient:
        name: str = ('vehicle_control/' if internal else 'mission_execution/') + operation
        if operation == 'execute_waypoint_mission':
            name = operation
        if name not in self.action_clients:
            self.action_clients[name] = ActionClient(self, action, name)
        client: ActionClient = self.action_clients[name]
        self.wait(client.server_is_ready, name + ' discovery', RESPONSE_TIMEOUT_SEC)
        return client

    def send(
        self, operation: str, action: Action, goal: Goal, internal: bool = False
    ) -> ClientGoalHandle:
        self.message('goal_' + operation, goal)
        handle: ClientGoalHandle = self.resolve(
            self.client(operation, action, internal).send_goal_async(
                goal, feedback_callback=self.on_feedback
            ),
            operation + ' acceptance',
        )
        self.event(
            'goal_response',
            operation=operation,
            accepted=handle.accepted,
            goal_id=[int(value) for value in handle.goal_id.uuid],
        )
        return handle

    def on_feedback(self, envelope: FeedbackEnvelope) -> None:
        self.message('feedback', envelope.feedback)

    def outcome(
        self, handle: ClientGoalHandle, timeout_sec: float = RESPONSE_TIMEOUT_SEC
    ) -> ActionResponse:
        response: ActionResponse = self.resolve(
            handle.get_result_async(), 'terminal action result', timeout_sec
        )
        self.event(
            'action_result',
            status=response.status,
            success=response.result.success,
            message=response.result.message,
        )
        return response

    def cancel(self, handle: ClientGoalHandle) -> ActionResponse:
        cancellation = self.resolve(handle.cancel_goal_async(), 'cancellation response')
        self.event(
            'cancel_response',
            return_code=cancellation.return_code,
            goals_canceling=len(cancellation.goals_canceling),
        )
        return self.outcome(handle)

    def stop_process(self, name: str, stop_signal: signal.Signals = signal.SIGKILL) -> None:
        self.event('process_fault', package=name, signal=stop_signal.name)
        self.expected_dead.add(name)
        self.processes[name].send_signal(stop_signal)

    def close(self) -> None:
        """Stop all owned nodes and flush teardown even after a contract assertion fails."""
        for name, process in reversed(list(self.processes.items())):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
                    if name not in self.expected_dead:
                        self.cleanup_errors.append(name + ' required SIGKILL during shutdown')
            self.event('process_stopped', package=name, exit_code=process.returncode)
            if name not in self.expected_dead and process.returncode not in (0, -signal.SIGTERM):
                self.cleanup_errors.append(
                    f'{name} exited with {process.returncode}; inspect its native/sanitizer logs'
                )
        if self.cleanup_errors:
            self.event('cleanup_errors', errors=self.cleanup_errors)
        self.destroy_node()
        self.events.close()
        if self.cleanup_errors:
            raise RuntimeError('; '.join(self.cleanup_errors))
