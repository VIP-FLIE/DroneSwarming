# Copyright 2026 DroneSwarming maintainers.
"""Calculate geometry and error metrics for recorded flight samples."""

from __future__ import annotations

import math
from collections.abc import Sequence

Vector3 = tuple[float, float, float]


def distance_2d(first: Vector3, second: Vector3) -> float:
    """Return horizontal distance between two ENU positions in meters."""
    return math.hypot(first[0] - second[0], first[1] - second[1])


def distance_3d(first: Vector3, second: Vector3) -> float:
    """Return Euclidean distance between two ENU positions in meters."""
    return math.hypot(distance_2d(first, second), first[2] - second[2])


def wrapped_angle_error(target_rad: float, observed_rad: float) -> float:
    """Return signed shortest angular error in radians."""
    return math.atan2(
        math.sin(target_rad - observed_rad),
        math.cos(target_rad - observed_rad),
    )


def cross_track_distance(point: Vector3, start: Vector3, end: Vector3) -> float:
    """Return horizontal distance from a point to a bounded line segment."""
    segment_x: float = end[0] - start[0]
    segment_y: float = end[1] - start[1]
    segment_length_squared: float = segment_x * segment_x + segment_y * segment_y
    if segment_length_squared == 0.0:
        return distance_2d(point, start)
    projection_fraction: float = (
        (point[0] - start[0]) * segment_x + (point[1] - start[1]) * segment_y
    ) / segment_length_squared
    bounded_fraction: float = min(1.0, max(0.0, projection_fraction))
    closest_point: Vector3 = (
        start[0] + bounded_fraction * segment_x,
        start[1] + bounded_fraction * segment_y,
        point[2],
    )
    return distance_2d(point, closest_point)


def percentile(values: Sequence[float], fraction: float) -> float:
    """Return a linearly interpolated percentile for finite samples."""
    if not values:
        raise ValueError('percentile requires at least one sample')
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError('percentile fraction must be finite and between zero and one')
    if any(not math.isfinite(value) for value in values):
        raise ValueError('percentile samples must be finite')
    ordered_values: list[float] = sorted(values)
    rank: float = fraction * (len(ordered_values) - 1)
    lower_index: int = math.floor(rank)
    upper_index: int = math.ceil(rank)
    interpolation_fraction: float = rank - lower_index
    return (
        ordered_values[lower_index] * (1.0 - interpolation_fraction)
        + ordered_values[upper_index] * interpolation_fraction
    )


def require_at_least(metric_name: str, actual: float, minimum: float) -> None:
    """Raise a diagnostic assertion when a metric is below its minimum."""
    if not math.isfinite(actual) or actual < minimum:
        raise AssertionError(f'{metric_name} was {actual:.3f}, required at least {minimum:.3f}')


def require_at_most(metric_name: str, actual: float, maximum: float) -> None:
    """Raise a diagnostic assertion when a metric exceeds its maximum."""
    if not math.isfinite(actual) or actual > maximum:
        raise AssertionError(f'{metric_name} was {actual:.3f}, required at most {maximum:.3f}')
