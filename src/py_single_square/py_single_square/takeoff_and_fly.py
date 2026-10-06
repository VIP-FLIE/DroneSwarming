"""Arms, climbs, flies a square, lands. Offboard control, no mission upload.

Run it with PX4 SITL and MicroXRCEAgent already running:
    ros2 run py_single_square fly
    ros2 run py_single_square fly --ros-args -p altitude:=8.0 -p side:=10.0
    ros2 run py_single_square fly --ros-args -p px4_ns:=px4_1 -p sys_id:=2

Offboard mode has one rule: PX4 will not accept the mode switch until it is
already receiving a stream of setpoints, and it drops out of offboard if that
stream stops for more than about half a second. The timer below publishes on
every tick, in every state, from before the mode switch until after landing.
"""

from __future__ import annotations

import math

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.publisher import Publisher

from py_single_square.px4_qos import PX4_QOS, PX4_CMD_QOS, topic_prefix

TICK_HZ: float = 10.0
SETPOINTS_BEFORE_ARM: int = 10
ARMING_TIMEOUT_S: float = 60.0
ARMING_ATTEMPT_INTERVAL_TICKS: int = 5
ARMING_QUIET_PERIOD_S: float = 3.0
WAYPOINT_RADIUS_M: float = 0.5
NANOSECONDS_PER_MICROSECOND: int = 1000
PX4_COMPONENT_ID: int = 1
BASE_MODE_CUSTOM_ENABLED: float = 1.0
CUSTOM_MAIN_MODE_OFFBOARD: float = 6.0

NAV_STATE_OFFBOARD: int = VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMING_STATE_ARMED: int = VehicleStatus.ARMING_STATE_ARMED

Waypoint = tuple[float, float, float]


class TakeoffAndFly(Node):

    def __init__(self) -> None:
        super().__init__('takeoff_and_fly')
        self.declare_parameter('px4_ns', '')
        self.declare_parameter('sys_id', 1)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('side', 5.0)

        prefix: str = topic_prefix(self.get_parameter('px4_ns').value)
        self._system_id: int = int(self.get_parameter('sys_id').value)
        altitude_m: float = float(self.get_parameter('altitude').value)
        side_m: float = float(self.get_parameter('side').value)

        self._offboard_mode_publisher: Publisher = self.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', PX4_QOS)
        self._setpoint_publisher: Publisher = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', PX4_QOS)
        self._command_publisher: Publisher = self.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command', PX4_CMD_QOS)

        self.create_subscription(
            VehicleLocalPosition, f'{prefix}/fmu/out/vehicle_local_position',
            self._on_position, PX4_QOS)
        self.create_subscription(
            VehicleStatus, f'{prefix}/fmu/out/vehicle_status_v1',
            self._on_status, PX4_QOS)

        self._waypoints: list[Waypoint] = [
            (0.0,      0.0,     -altitude_m),
            (side_m,   0.0,     -altitude_m),
            (side_m,   side_m,  -altitude_m),
            (0.0,      side_m,  -altitude_m),
            (0.0,      0.0,     -altitude_m),
        ]
        self._waypoint_index: int = 0
        self._ticks: int = 0
        self._arming_attempts: int = 0
        self._arming_start_time: rclpy.time.Time | None = None
        self._landing: bool = False
        self._position: VehicleLocalPosition | None = None
        self._status: VehicleStatus | None = None

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            f'target altitude {altitude_m} m, {side_m} m square, '
            f'talking to {prefix or "/"}fmu/...')

    # -- subscriptions ------------------------------------------------------
    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self._position = msg

    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg

    # -- outgoing -----------------------------------------------------------
    def _stamp(self) -> int:
        return int(self.get_clock().now().nanoseconds / NANOSECONDS_PER_MICROSECOND)

    def _publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = self._stamp()
        self._offboard_mode_publisher.publish(msg)

    def _publish_setpoint(self, north: float, east: float, down: float,
                          yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.position = [north, east, down]
        msg.yaw = yaw
        msg.timestamp = self._stamp()
        self._setpoint_publisher.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0,
                      param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self._system_id
        msg.target_component = PX4_COMPONENT_ID
        msg.source_system = self._system_id
        msg.source_component = PX4_COMPONENT_ID
        msg.from_external = True
        msg.timestamp = self._stamp()
        self._command_publisher.publish(msg)

    # -- helpers ------------------------------------------------------------
    def _is_armed(self) -> bool:
        return self._status is not None and self._status.arming_state == ARMING_STATE_ARMED

    def _is_offboard(self) -> bool:
        return self._status is not None and self._status.nav_state == NAV_STATE_OFFBOARD

    def _distance_to(self, target: Waypoint) -> float:
        north, east, down = target
        if self._position is None:
            return float('inf')
        return math.sqrt(
            (self._position.x - north) ** 2 +
            (self._position.y - east) ** 2 +
            (self._position.z - down) ** 2)

    # -- state machine ------------------------------------------------------
    def _tick(self) -> None:
        if self._landing:
            if self._status is not None and not self._is_armed():
                # PX4 disarms after touchdown. Flight is over.
                self.get_logger().info('landed and disarmed. Flight complete.')
            return

        self._publish_offboard_mode()
        self._publish_setpoint(*self._waypoints[self._waypoint_index])
        self._ticks += 1

        # Build up the setpoint stream before asking PX4 for anything.
        if self._ticks < SETPOINTS_BEFORE_ARM:
            return
        if not self._is_armed():
            self._request_arming()
            return
        if not self._is_offboard():
            self._request_offboard()
            return
        self._advance_waypoint()

    def _request_arming(self) -> None:
        if self._is_armed():
            return
        if self._arming_start_time is None:
            self._arming_start_time = self.get_clock().now()

        waiting_s: float = (self.get_clock().now() - self._arming_start_time).nanoseconds / 1e9
        if waiting_s > ARMING_TIMEOUT_S:
            self.get_logger().error(
                f'Gave up arming after {ARMING_TIMEOUT_S:.0f} s. '
                'Check pxh> commander status for failsafe flags.')
            return

        self._arming_attempts += 1
        if self._arming_attempts % ARMING_ATTEMPT_INTERVAL_TICKS == 0:
            self._send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        if waiting_s >= ARMING_QUIET_PERIOD_S:
            self.get_logger().warn(
                f'not armed yet ({waiting_s:.0f} s of {ARMING_TIMEOUT_S:.0f}).',
                throttle_duration_sec=5.0)

    def _request_offboard(self) -> None:
        self._send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=BASE_MODE_CUSTOM_ENABLED,
            param2=CUSTOM_MAIN_MODE_OFFBOARD)

    def _advance_waypoint(self) -> None:
        if self._position is None:
            return
        if self._distance_to(self._waypoints[self._waypoint_index]) >= WAYPOINT_RADIUS_M:
            return
        self._waypoint_index += 1
        if self._waypoint_index >= len(self._waypoints):
            self.get_logger().info('square complete, landing')
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._landing = True
            return
        north, east, down = self._waypoints[self._waypoint_index]
        self.get_logger().info(
            f'waypoint {self._waypoint_index}/{len(self._waypoints) - 1}: '
            f'n={north:.1f} e={east:.1f} alt={-down:.1f}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TakeoffAndFly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()