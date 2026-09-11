"""Arms, climbs, flies a square, lands. Offboard control, no mission upload.

Run it with PX4 SITL and MicroXRCEAgent already running:
    ros2 run drone_vip fly
    ros2 run drone_vip fly --ros-args -p altitude:=8.0 -p side:=10.0
    ros2 run drone_vip fly --ros-args -p px4_ns:=px4_1 -p sys_id:=2

Offboard mode has one rule that trips everyone up: PX4 will not accept the
mode switch until it is already receiving a stream of setpoints, and it will
drop out of offboard if that stream stops for more than about half a second.
So the timer below publishes on every tick, in every state, from before the
mode switch until after landing.
"""

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

from drone_vip.px4_qos import PX4_QOS, PX4_CMD_QOS, topic_prefix

TICK_HZ = 10.0
SETPOINTS_BEFORE_ARM = 10            # 1 second of stream at 10 Hz
ARM_RETRY_TICKS = 30                 # retry arm for 3 seconds (BEST_EFFORT can drop)
WAYPOINT_RADIUS_M = 0.5
# Read off px4_msgs rather than hardcoded, so it tracks whatever branch the
# container was built against.
NAV_STATE_OFFBOARD = VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMING_STATE_ARMED = VehicleStatus.ARMING_STATE_ARMED


class TakeoffAndFly(Node):

    def __init__(self):
        super().__init__('takeoff_and_fly')
        self.declare_parameter('px4_ns', '')
        self.declare_parameter('sys_id', 1)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('side', 5.0)

        prefix = topic_prefix(self.get_parameter('px4_ns').value)
        self.sys_id = int(self.get_parameter('sys_id').value)
        alt = float(self.get_parameter('altitude').value)
        side = float(self.get_parameter('side').value)

        self.mode_pub = self.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command', PX4_CMD_QOS)

        self.create_subscription(
            VehicleLocalPosition, f'{prefix}/fmu/out/vehicle_local_position',
            self._on_position, PX4_QOS)
        self.create_subscription(
            VehicleStatus, f'{prefix}/fmu/out/vehicle_status_v1',
            self._on_status, PX4_QOS)

        # NED. Down is negative, so -alt is alt metres above the start point.
        self.waypoints = [
            (0.0,   0.0,  -alt),
            (side,  0.0,  -alt),
            (side,  side, -alt),
            (0.0,   side, -alt),
            (0.0,   0.0,  -alt),
        ]
        self.waypoint_index = 0
        self.ticks = 0
        self.position = None
        self.status = None
        self.landing = False
        self._offboard_requested = False

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            f'target altitude {alt} m, {side} m square, '
            f'talking to {prefix or "/"}fmu/...')

    # -- subscriptions ------------------------------------------------------
    def _on_position(self, msg):
        self.position = msg

    def _on_status(self, msg):
        self.status = msg

    # -- outgoing -----------------------------------------------------------
    def _stamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = self._stamp()
        self.mode_pub.publish(msg)

    def _publish_setpoint(self, north, east, down, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(north), float(east), float(down)]
        msg.yaw = float(yaw)
        msg.timestamp = self._stamp()
        self.setpoint_pub.publish(msg)

    def _send_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        for key, value in params.items():
            setattr(msg, key, float(value))
        msg.target_system = self.sys_id
        msg.target_component = 1
        msg.source_system = self.sys_id
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._stamp()
        self.command_pub.publish(msg)

    # -- helpers ------------------------------------------------------------
    def _is_armed(self):
        return self.status is not None and self.status.arming_state == ARMING_STATE_ARMED

    # -- state machine ------------------------------------------------------
    def _tick(self):
        # Always publish, on every tick. This is the stream PX4 requires.
        self._publish_offboard_mode()

        if self.landing:
            return

        target = self.waypoints[self.waypoint_index]
        self._publish_setpoint(*target)
        self.ticks += 1

        # --- Phase 0: build up the setpoint stream -------------------------
        if self.ticks < SETPOINTS_BEFORE_ARM:
            return

        # --- Phase 1: arm with retries -------------------------------------
        if not self._is_armed():
            if self.ticks <= SETPOINTS_BEFORE_ARM + ARM_RETRY_TICKS:
                if self.ticks == SETPOINTS_BEFORE_ARM:
                    self.get_logger().info('sending arm command (will retry)')
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0)
                return
            self.get_logger().error(
                f'arming failed after {ARM_RETRY_TICKS} retries. '
                'Check pxh> commander status for failsafe flags.')
            return

        # --- Phase 2: switch to offboard once armed ------------------------
        if self.status is not None and self.status.nav_state != NAV_STATE_OFFBOARD:
            if not self._offboard_requested:
                self.get_logger().info('armed, switching to offboard mode')
                self._send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0, param2=6.0)
                self._offboard_requested = True
            return

        # --- Phase 3: fly the square ---------------------------------------
        if self.position is None:
            return

        if self._distance_to(target) < WAYPOINT_RADIUS_M:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info('square complete, landing')
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing = True
                return
            n, e, d = self.waypoints[self.waypoint_index]
            self.get_logger().info(
                f'waypoint {self.waypoint_index}/{len(self.waypoints) - 1}: '
                f'n={n:.1f} e={e:.1f} alt={-d:.1f}')

    def _distance_to(self, target):
        n, e, d = target
        p = self.position
        return math.sqrt((p.x - n) ** 2 + (p.y - e) ** 2 + (p.z - d) ** 2)


def main(args=None):
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
