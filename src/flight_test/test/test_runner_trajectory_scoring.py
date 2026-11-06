# Copyright 2026 DroneSwarming maintainers.
"""Verify that trajectory cases grade commanded outcomes rather than motion timing."""

from collections import deque
from collections.abc import Callable
from unittest.mock import Mock

import pytest
from flight_test.runner import FlightSample, FlightTestRunner

from swarm_interfaces.action import FollowTrajectory


def make_sample(
    x_m: float, y_m: float = 0.0, altitude_m: float = 3.0, speed_mps: float = 3.0
) -> FlightSample:
    """Build a valid airborne sample for the outbound and return path."""
    return FlightSample(
        received_monotonic_sec=0.0,
        header_stamp_sec=0.0,
        frame_id='local_enu',
        position_enu=(x_m, y_m, altitude_m),
        velocity_enu=(speed_mps, 0.0, 0.0),
        yaw_rad=0.0,
        position_valid=True,
        velocity_valid=True,
        yaw_valid=True,
        armed=True,
        offboard=True,
        landed=False,
        setpoint_publication_count=0,
    )


def score_trajectory_case(
    flight_samples: list[FlightSample], run_case: Callable[[FlightTestRunner], None]
) -> FlightTestRunner:
    """Grade one trajectory case against supplied telemetry without PX4."""
    runner: FlightTestRunner = FlightTestRunner.__new__(FlightTestRunner)
    runner.samples = deque([make_sample(0.0)])
    runner.assertions = []
    runner.measurements = {}
    runner.last_trajectory_started_monotonic_sec = 0.0
    runner._takeoff = Mock()
    runner._wait_for_airborne_yaw = Mock()
    runner._record_trajectory_point = Mock()
    runner._land = Mock()

    def current_sample(require_yaw: bool = False) -> FlightSample:
        """Return the latest synthetic telemetry sample."""
        return runner.samples[-1]

    def run_trajectory(goal: FollowTrajectory.Goal) -> None:
        """Append synthetic telemetry after the runner records its start index."""
        runner.samples.extend(flight_samples)

    runner._current_sample = Mock(side_effect=current_sample)
    runner._run_trajectory = Mock(side_effect=run_trajectory)

    run_case(runner)

    return runner


def test_ft02_accepts_path_without_a_fixed_stop_duration() -> None:
    """Accept an outbound and return flight with no sustained low-speed samples."""
    flight_samples: list[FlightSample] = [
        make_sample(x_m) for x_m in (1.0, 4.0, 8.0, 11.8, 12.0, 10.0, 6.0, 2.0, 0.2)
    ]

    runner: FlightTestRunner = score_trajectory_case(
        flight_samples, FlightTestRunner._run_fast_stop_and_return
    )

    assert all(assertion['passed'] for assertion in runner.assertions)
    assert runner._land.called


def test_ft02_rejects_deviation_from_the_commanded_path() -> None:
    """Fail a flight that visits its endpoints but leaves the path corridor."""
    flight_samples: list[FlightSample] = [
        make_sample(4.0),
        make_sample(7.0, y_m=2.0),
        make_sample(12.0),
        make_sample(0.2),
    ]

    with pytest.raises(AssertionError, match='out-and-back path error'):
        score_trajectory_case(
            flight_samples, FlightTestRunner._run_fast_stop_and_return
        )


def test_ft02_requires_return_after_outbound_visit() -> None:
    """Exclude the launch position from return scoring."""
    flight_samples: list[FlightSample] = [
        make_sample(4.0),
        make_sample(8.0),
        make_sample(12.0),
    ]

    with pytest.raises(AssertionError, match='return position error'):
        score_trajectory_case(
            flight_samples, FlightTestRunner._run_fast_stop_and_return
        )


def test_ft02_rejects_gross_observed_overspeed() -> None:
    """Fail a path-compliant flight that exceeds the speed ceiling."""
    flight_samples: list[FlightSample] = [
        make_sample(4.0),
        make_sample(8.0, speed_mps=5.5),
        make_sample(12.0),
        make_sample(0.2),
    ]

    with pytest.raises(AssertionError, match='out-and-back horizontal speed'):
        score_trajectory_case(
            flight_samples, FlightTestRunner._run_fast_stop_and_return
        )


def test_ft05_accepts_target_visits_without_a_fixed_settling_duration() -> None:
    """Accept a climb and descent even when no telemetry interval settles for half a second."""
    flight_samples: list[FlightSample] = [
        make_sample(0.0, altitude_m=4.0),
        make_sample(0.0, altitude_m=5.8),
        make_sample(0.0, altitude_m=6.0),
        make_sample(0.0, altitude_m=4.5),
        make_sample(0.0, altitude_m=3.2),
    ]

    runner: FlightTestRunner = score_trajectory_case(
        flight_samples, FlightTestRunner._run_altitude_steps
    )

    assert all(assertion['passed'] for assertion in runner.assertions)
    assert runner.measurements['climb_settle_duration_sec'] == 0.0
    assert runner.measurements['descent_settle_duration_sec'] == 0.0


def test_ft05_requires_descent_after_climb() -> None:
    """Exclude the launch altitude from descent scoring."""
    flight_samples: list[FlightSample] = [
        make_sample(0.0, altitude_m=4.0),
        make_sample(0.0, altitude_m=6.0),
        make_sample(0.0, altitude_m=5.0),
    ]

    with pytest.raises(AssertionError, match='descent altitude error'):
        score_trajectory_case(flight_samples, FlightTestRunner._run_altitude_steps)
