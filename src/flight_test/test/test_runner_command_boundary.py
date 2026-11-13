# Copyright 2026 DroneSwarming maintainers.
"""Verify every flight case creates command clients only for mission execution."""

from pathlib import Path
from unittest.mock import Mock

import pytest
import rclpy
from flight_test.runner import FlightTestRunner

from flight_test import runner as runner_module


@pytest.mark.parametrize('case_id', [f'FT-{index:02d}' for index in range(7)])
def test_case_command_clients_target_mission_execution(
    case_id: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Retain all operation types while excluding internal execution endpoints."""
    action_client = Mock()
    monkeypatch.setattr(runner_module, 'ActionClient', action_client)
    rclpy.init()
    runner = FlightTestRunner(case_id, tmp_path)
    try:
        endpoints: set[str] = {call.args[2] for call in action_client.call_args_list}
        assert endpoints == {
            '/execute_waypoint_mission',
            '/mission_execution/takeoff',
            '/mission_execution/navigate_to',
            '/mission_execution/follow_trajectory',
            '/mission_execution/orbit_point',
            '/mission_execution/land',
        }
        assert len(action_client.call_args_list) == len(endpoints)
    finally:
        runner.telemetry_file.close()
        runner.command_file.close()
        runner.feedback_file.close()
        runner.destroy_node()
        rclpy.shutdown()
