# Copyright 2026 DroneSwarming maintainers.
"""Check flight readiness sequencing against PX4 yaw alignment behavior."""

import time
from collections import deque
from dataclasses import replace
from unittest.mock import Mock

import pytest
from flight_test.runner import FlightSample, FlightTestFailure, FlightTestRunner

from flight_test import runner as runner_module


def make_sample(yaw_valid: bool, armed: bool = False) -> FlightSample:
    """Build a fresh state sample at the selected flight phase."""
    return FlightSample(
        received_monotonic_sec=time.monotonic(),
        header_stamp_sec=0.0,
        frame_id='local_enu',
        position_enu=(0.0, 0.0, 0.0),
        velocity_enu=(0.0, 0.0, 0.0),
        yaw_rad=0.0,
        position_valid=True,
        velocity_valid=True,
        yaw_valid=yaw_valid,
        armed=armed,
        offboard=armed,
        landed=not armed,
        setpoint_publication_count=0,
    )


def test_landed_readiness_does_not_require_final_yaw_alignment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Allow takeoff when PX4 reports valid landed position but unfinished yaw alignment."""
    runner = FlightTestRunner.__new__(FlightTestRunner)
    runner.case_id = 'FT-02'
    runner.samples = deque([make_sample(yaw_valid=False)])
    client = Mock()
    client.server_is_ready.return_value = True
    monkeypatch.setattr(runner_module.rclpy, 'ok', Mock(return_value=True))
    monkeypatch.setattr(runner_module.rclpy, 'spin_once', Mock())
    monkeypatch.setattr(
        FlightTestRunner,
        'get_node_names_and_namespaces',
        Mock(return_value=[('vehicle_control', '/')]),
    )

    runner._wait_for_ready(1.0, (client,))


def test_heading_command_waits_for_airborne_yaw_alignment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Accept heading commands only after fresh airborne yaw becomes valid."""
    runner = FlightTestRunner.__new__(FlightTestRunner)
    runner.samples = deque([make_sample(yaw_valid=False, armed=True)])
    runner.telemetry_fault = None
    runner.expect_offboard = True
    monkeypatch.setattr(runner_module.rclpy, 'ok', Mock(return_value=True))

    def publish_aligned_yaw(node: FlightTestRunner, timeout_sec: float) -> None:
        """Advance the simulated vehicle state to aligned yaw."""
        node.samples.append(
            replace(
                node.samples[-1],
                received_monotonic_sec=time.monotonic(),
                yaw_valid=True,
            )
        )

    monkeypatch.setattr(runner_module.rclpy, 'spin_once', publish_aligned_yaw)

    runner._wait_for_airborne_yaw()


def test_airborne_yaw_timeout_fails_before_heading_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Abort a heading case if PX4 never reports final yaw alignment."""
    runner = FlightTestRunner.__new__(FlightTestRunner)
    runner.samples = deque([make_sample(yaw_valid=False, armed=True)])
    runner.telemetry_fault = None
    runner.expect_offboard = True
    monkeypatch.setattr(runner_module, 'AIRBORNE_YAW_TIMEOUT_SEC', 0.0)
    monkeypatch.setattr(runner_module.rclpy, 'ok', Mock(return_value=True))

    with pytest.raises(FlightTestFailure, match='PX4 yaw remained invalid'):
        runner._wait_for_airborne_yaw()
