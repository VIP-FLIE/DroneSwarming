"""Prints the drone's position. The smallest useful PX4 + ROS 2 node.

Run it with:
    ros2 run py_single_square listener

For the second vehicle in a multi-vehicle sim:
    ros2 run py_single_square listener --ros-args -p px4_ns:=px4_1
"""

from __future__ import annotations

from typing import Any

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node

from py_single_square.px4_qos import PX4_QOS, topic_prefix


def _constants(msg_class: type[Any], prefix: str) -> dict[int, str]:
    """Build a value -> name table from a message class's own constants.

    Read off px4_msgs at runtime rather than hardcoded, so these labels cannot
    drift out of sync with the installed px4_msgs branch.
    """
    return {
        getattr(msg_class, name): name[len(prefix):]
        for name in dir(msg_class)
        if name.startswith(prefix) and isinstance(getattr(msg_class, name), int)
    }


ARMING_STATE: dict[int, str] = _constants(VehicleStatus, 'ARMING_STATE_')
NAV_STATE: dict[int, str] = _constants(VehicleStatus, 'NAVIGATION_STATE_')
REPORT_INTERVAL_S: float = 1.0


class Listener(Node):

    def __init__(self) -> None:
        super().__init__('listener')
        self.declare_parameter('px4_ns', '')
        prefix: str = topic_prefix(self.get_parameter('px4_ns').value)

        self.position: VehicleLocalPosition | None = None
        self.status: VehicleStatus | None = None

        self.create_subscription(
            VehicleLocalPosition, f'{prefix}/fmu/out/vehicle_local_position',
            self._on_position, PX4_QOS)
        self.create_subscription(
            VehicleStatus, f'{prefix}/fmu/out/vehicle_status_v1',
            self._on_status, PX4_QOS)

        self.create_timer(REPORT_INTERVAL_S, self._report)
        self.get_logger().info(f'listening on {prefix or "/"}fmu/out/...')

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self.position = msg

    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def _report(self) -> None:
        if self.position is None:
            self.get_logger().warn(
                'no messages yet. Is PX4 running and is MicroXRCEAgent up?')
            return

        position: VehicleLocalPosition = self.position
        arming_state: str | int = '?'
        nav_state: str | int = '?'
        if self.status is not None:
            arming_state = ARMING_STATE.get(
                self.status.arming_state, self.status.arming_state)
            nav_state = NAV_STATE.get(self.status.nav_state, self.status.nav_state)

        # PX4 uses NED, so z is negative when the drone is above its origin.
        self.get_logger().info(
            f'n={position.x:7.2f}  e={position.y:7.2f}  d={position.z:7.2f}  '
            f'(alt {-position.z:5.2f} m)  heading={position.heading:6.2f} rad  '
            f'{arming_state}/{nav_state}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Listener()
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