# Copyright 2026 DroneSwarming maintainers.
"""Test flight-path measurements and threshold failures."""

import math

import pytest
from flight_test.metrics import (
    cross_track_distance,
    distance_2d,
    distance_3d,
    percentile,
    require_at_least,
    require_at_most,
    wrapped_angle_error,
)


def test_position_distances_use_horizontal_and_vertical_axes() -> None:
    """Check horizontal and 3D distances on independent axes."""
    assert distance_2d((0.0, 0.0, 0.0), (3.0, 4.0, 9.0)) == pytest.approx(5.0)
    assert distance_3d((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == pytest.approx(5.0)


def test_wrapped_heading_error_crosses_pi_boundary() -> None:
    """Check shortest-angle error across the pi boundary."""
    assert wrapped_angle_error(-math.pi + 0.1, math.pi - 0.1) == pytest.approx(0.2)


def test_cross_track_distance_clamps_to_segment_endpoints() -> None:
    """Check cross-track distance clamps to each segment endpoint."""
    assert cross_track_distance(
        (5.0, 2.0, 1.0), (0.0, 0.0, 0.0), (10.0, 0.0, 5.0)
    ) == pytest.approx(2.0)
    assert cross_track_distance(
        (12.0, 0.0, 0.0), (0.0, 0.0, 0.0), (10.0, 0.0, 0.0)
    ) == pytest.approx(2.0)


def test_percentile_interpolates_and_rejects_invalid_input() -> None:
    """Check percentile interpolation and invalid sample rejection."""
    assert percentile((0.0, 10.0), 0.95) == pytest.approx(9.5)
    with pytest.raises(ValueError):
        percentile((), 0.5)
    with pytest.raises(ValueError):
        percentile((1.0, math.inf), 0.5)


def test_threshold_assertions_include_metric_and_observed_value() -> None:
    """Check threshold assertions include the metric and observed value."""
    require_at_least('peak speed', 3.5, 3.0)
    require_at_most('radial error', 1.0, 1.25)
    with pytest.raises(AssertionError, match='peak speed was 2.000'):
        require_at_least('peak speed', 2.0, 3.0)
    with pytest.raises(AssertionError, match='radial error was 2.000'):
        require_at_most('radial error', 2.0, 1.25)
