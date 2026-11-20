"""Vehicle-control admission and telemetry tests against synthetic PX4 DDS peers."""

from __future__ import annotations

import math
import time

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from swarm_interfaces.action import FlyMission, NavigateTo
from swarm_interfaces.msg import VehicleState

from support import OPERATIONS, WireCase, geographic, valid_goals

PX4_PREFIX = '/blind_px4/fmu'


class VehicleTorture(WireCase):
    def boot(self, *, enabled: bool | None = None) -> None:
        self.publish_state = False
        self.samples: list[VehicleState] = []
        self.commands: list[VehicleCommand | TrajectorySetpoint | OffboardControlMode] = []
        self.node.create_subscription(
            VehicleState, 'vehicle_state', self.samples.append, qos_profile_sensor_data
        )
        parameters: dict[str, str | bool] = {
            'px4_topic_prefix': PX4_PREFIX,
            'mission_connection_url': 'udpin://127.0.0.1:14679',
        }
        if enabled is not None:
            parameters['enable_flight_commands'] = enabled
        self.launch('vehicle_control', **parameters)
        self.until(lambda: bool(self.samples), 12.0)
        for message_type, suffix in [
            (VehicleCommand, 'vehicle_command'),
            (TrajectorySetpoint, 'trajectory_setpoint'),
            (OffboardControlMode, 'offboard_control_mode'),
        ]:
            self.node.create_subscription(
                message_type,
                f'{PX4_PREFIX}/in/{suffix}',
                self.capture_output,
                qos_profile_sensor_data,
            )
        self.pause()

    def capture_output(
        self, message: VehicleCommand | TrajectorySetpoint | OffboardControlMode
    ) -> None:
        """Record every observed PX4 command, setpoint, and control-mode message."""
        self.commands.append(message)
        self.trace('px4_output', message=message)

    def sensor_stream(self, *, airborne: bool = True) -> None:
        self.local = VehicleLocalPosition(
            x=2.0,
            y=5.0,
            z=-3.0,
            vx=0.2,
            vy=0.5,
            vz=-0.3,
            xy_valid=True,
            z_valid=True,
            v_xy_valid=True,
            v_z_valid=True,
            xy_global=True,
            z_global=True,
            ref_timestamp=1,
            ref_lat=0.0,
            ref_lon=0.0,
            ref_alt=100.0,
            heading=0.0,
            heading_good_for_control=True,
        )
        self.status = VehicleStatus(
            arming_state=VehicleStatus.ARMING_STATE_ARMED
            if airborne
            else VehicleStatus.ARMING_STATE_DISARMED,
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD
            if airborne
            else VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
            vehicle_type=VehicleStatus.VEHICLE_TYPE_ROTARY_WING,
            system_id=1,
            component_id=1,
            pre_flight_checks_pass=True,
        )
        self.land = VehicleLandDetected(landed=not airborne)
        self.trace('px4_sensor_baseline', local=self.local, status=self.status, land=self.land)
        self.sensor_enabled = True
        self.sensor_publishers = []
        for message in [self.local, self.status, self.land]:
            wire_type = f'px4_msgs/msg/{type(message).__name__}'
            matches = [
                name
                for name, types in self.node.get_topic_names_and_types()
                if name.startswith(f'{PX4_PREFIX}/out/') and wire_type in types
            ]
            self.assertEqual(
                len(matches), 1, f'Expected one subscription for {wire_type}: {matches}'
            )
            # Reliable transient-local publishers satisfy both weaker and equal reader requests.
            sensor_quality = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            publisher = self.node.create_publisher(type(message), matches[0], sensor_quality)
            self.sensor_publishers.append((publisher, message))
        self.sensor_timer = self.node.create_timer(
            0.02, self.publish_sensors, callback_group=self.callback_group
        )
        self.until(
            lambda: all(
                publisher.get_subscription_count() > 0 for publisher, _ in self.sensor_publishers
            )
        )
        self.until(
            lambda: self.samples[-1].position_valid and self.samples[-1].global_position_valid
        )

    def publish_sensors(self) -> None:
        if self.sensor_enabled:
            for publisher, message in self.sensor_publishers:
                message.timestamp = time.monotonic_ns() // 1000
                publisher.publish(message)

    def assert_no_flight_output(self) -> None:
        self.pause()
        self.assertEqual(self.commands, [], 'Rejected input produced PX4 flight output')
        self.healthy()

    def test_default_disabled_rejects_all_actions_without_px4_output(self) -> None:
        self.boot()
        self.sensor_stream()
        for operation, goal in valid_goals().items():
            with self.subTest(operation=operation):
                self.unsuccessful(f'vehicle_control/{operation}', OPERATIONS[operation], goal)
        self.assert_no_flight_output()

    def test_enabled_without_telemetry_rejects_all_actions(self) -> None:
        self.boot(enabled=True)
        for operation, goal in valid_goals().items():
            with self.subTest(operation=operation):
                self.unsuccessful(f'vehicle_control/{operation}', OPERATIONS[operation], goal)
        self.assert_no_flight_output()

    def test_finite_enu_conversion_and_stale_validity(self) -> None:
        self.boot(enabled=True)
        self.sensor_stream()
        sample = self.samples[-1]
        self.trace('vehicle_state_canary', sample=sample)
        for actual, expected in [
            (sample.position.x, 5.0),
            (sample.position.y, 2.0),
            (sample.position.z, 3.0),
            (sample.velocity.x, 0.5),
            (sample.velocity.y, 0.2),
            (sample.velocity.z, 0.3),
        ]:
            self.assertAlmostEqual(actual, expected, places=5)
        self.assertTrue(sample.velocity_valid)
        self.assertTrue(sample.global_reference_valid)
        self.assertTrue(sample.global_position_valid)
        self.sensor_enabled = False
        self.pause(2.4)
        self.assertFalse(self.samples[-1].position_valid)
        self.assertFalse(self.samples[-1].velocity_valid)
        self.assertFalse(self.samples[-1].global_position_valid)
        self.assertFalse(self.samples[-1].offboard)
        self.unsuccessful('vehicle_control/navigate_to', NavigateTo, valid_goals()['navigate_to'])
        self.assert_no_flight_output()

    def test_poisoned_telemetry_cannot_remain_valid(self) -> None:
        self.boot(enabled=True)
        self.sensor_stream()
        for field, value, validity in [
            ('x', math.nan, 'position_valid'),
            ('y', math.inf, 'position_valid'),
            ('z', -math.inf, 'position_valid'),
            ('vx', math.nan, 'velocity_valid'),
            ('heading', math.nan, 'yaw_valid'),
            ('ref_lat', 91.0, 'global_reference_valid'),
            ('ref_lon', math.inf, 'global_reference_valid'),
            ('ref_alt', math.nan, 'global_reference_valid'),
        ]:
            old_value = getattr(self.local, field)
            setattr(self.local, field, value)
            self.pause()
            with self.subTest(field=field):
                sample = self.samples[-1]
                self.trace('poisoned_telemetry', field=field, value=value, state=sample)
                self.assertFalse(getattr(sample, validity))
            setattr(self.local, field, old_value)
            self.pause()
        self.assertTrue(self.samples[-1].position_valid)
        self.assertTrue(self.samples[-1].global_position_valid)
        self.assert_no_flight_output()

    def test_invalid_navigation_with_fresh_telemetry_never_commands_px4(self) -> None:
        self.boot(enabled=True)
        self.sensor_stream()
        for axis in ['x', 'y', 'z']:
            for value in [math.nan, math.inf, -math.inf]:
                goal = valid_goals()['navigate_to']
                setattr(goal.target_position, axis, value)
                with self.subTest(axis=axis, value=value):
                    self.unsuccessful('vehicle_control/navigate_to', NavigateTo, goal)
        for value in [math.nan, math.inf, -1.0, 3601.0]:
            goal = valid_goals()['navigate_to']
            goal.timeout_sec = value
            self.unsuccessful('vehicle_control/navigate_to', NavigateTo, goal)
        self.assert_no_flight_output()

    def test_invalid_geographic_upload_never_commands_px4(self) -> None:
        self.boot(enabled=True)
        self.sensor_stream(airborne=False)
        for destinations in [
            [],
            [geographic(91.0, 0.0, 103.0)],
            [geographic(0.0, 181.0, 103.0)],
            [geographic(math.nan, 0.0, 103.0)],
            [geographic(0.0, 0.0, math.inf)],
            [geographic(0.0, 0.0, 103.0)] * 257,
        ]:
            goal = valid_goals()['fly_mission']
            goal.waypoint_positions = destinations
            self.unsuccessful('vehicle_control/fly_mission', FlyMission, goal)
        self.assert_no_flight_output()
