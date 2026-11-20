# Portions Copyright (c) 2026, vijaymanbajracharya. See LICENSE.sar-drone.
"""Generate parallel search stripes inside convex polygons and circles."""

import math
from collections.abc import Sequence
from typing import NamedTuple

MAXIMUM_MISSION_WAYPOINTS = 256
MAXIMUM_AREA_VERTICES = 512
GEOMETRY_TOLERANCE_M = 1e-7


class Point(NamedTuple):
    """Store east and north coordinates in the path planning frame."""

    east_m: float
    north_m: float


def _cross(first: Point, second: Point, third: Point) -> float:
    """Measure turn orientation for convex perimeter validation."""
    return (second.east_m - first.east_m) * (third.north_m - second.north_m) - (
        second.north_m - first.north_m
    ) * (third.east_m - second.east_m)


def plan_lawnmower(
    corners: Sequence[Point],
    stripe_gap_m: float,
    stripe_heading_rad: float | None = None,
) -> list[Point]:
    """Clip alternating stripes to a bounded convex polygon."""
    vertex_count: int = len(corners)
    if (
        not 3 <= vertex_count <= MAXIMUM_AREA_VERTICES
        or not math.isfinite(stripe_gap_m)
        or stripe_gap_m <= 0.0
    ):
        raise ValueError(
            'a polygon with 3 to 512 vertices and a positive finite stripe gap is required'
        )
    if any(not math.isfinite(value) for corner in corners for value in corner):
        raise ValueError('corner coordinates must be finite')
    if stripe_heading_rad is not None and not math.isfinite(stripe_heading_rad):
        raise ValueError('stripe heading must be finite')

    first_turn: float = _cross(corners[0], corners[1], corners[2])
    if abs(first_turn) <= GEOMETRY_TOLERANCE_M:
        raise ValueError('vertices must form a nondegenerate convex polygon in perimeter order')
    orientation: float = 1.0 if first_turn > 0.0 else -1.0
    for edge_index in range(vertex_count):
        next_index: int = (edge_index + 1) % vertex_count
        if any(
            orientation * _cross(corners[edge_index], corners[next_index], corners[vertex_index])
            <= GEOMETRY_TOLERANCE_M
            for vertex_index in range(vertex_count)
            if vertex_index not in (edge_index, next_index)
        ):
            raise ValueError('vertices must form a nondegenerate convex polygon in perimeter order')

    first: Point = corners[0]
    second: Point = corners[1]
    stripe_length_m: float = math.hypot(
        second.east_m - first.east_m, second.north_m - first.north_m
    )
    if stripe_heading_rad is None:
        along_east: float = (second.east_m - first.east_m) / stripe_length_m
        along_north: float = (second.north_m - first.north_m) / stripe_length_m
        normal_east: float = -along_north if orientation > 0.0 else along_north
        normal_north: float = along_east if orientation > 0.0 else -along_east
    else:
        along_east = math.cos(stripe_heading_rad)
        along_north = math.sin(stripe_heading_rad)
        normal_east = -along_north
        normal_north = along_east
    projections: list[float] = [
        (corner.east_m - first.east_m) * normal_east
        + (corner.north_m - first.north_m) * normal_north
        for corner in corners
    ]
    minimum_projection_m: float = min(projections)
    maximum_projection_m: float = max(projections)
    width_m: float = maximum_projection_m - minimum_projection_m
    stripe_intervals: float = width_m / stripe_gap_m
    if stripe_intervals > (MAXIMUM_MISSION_WAYPOINTS // 2) - 1:
        raise ValueError('stripe gap would exceed the mission waypoint limit')

    offsets_m: list[float] = [
        min(minimum_projection_m + index * stripe_gap_m, maximum_projection_m)
        for index in range(math.ceil(stripe_intervals) + 1)
    ]
    waypoints: list[Point] = []
    for stripe_index, offset_m in enumerate(offsets_m):
        intersections: list[Point] = []
        for edge_index in range(vertex_count):
            next_index: int = (edge_index + 1) % vertex_count
            start_projection: float = projections[edge_index]
            end_projection: float = projections[next_index]
            if abs(end_projection - start_projection) <= GEOMETRY_TOLERANCE_M:
                if abs(offset_m - start_projection) <= GEOMETRY_TOLERANCE_M:
                    intersections.extend((corners[edge_index], corners[next_index]))
                continue
            fraction: float = (offset_m - start_projection) / (end_projection - start_projection)
            if -GEOMETRY_TOLERANCE_M <= fraction <= 1.0 + GEOMETRY_TOLERANCE_M:
                fraction = min(1.0, max(0.0, fraction))
                start: Point = corners[edge_index]
                end: Point = corners[next_index]
                intersections.append(
                    Point(
                        start.east_m + fraction * (end.east_m - start.east_m),
                        start.north_m + fraction * (end.north_m - start.north_m),
                    )
                )
        if not intersections:
            raise ValueError('could not intersect a stripe with the search area')
        intersections.sort(
            key=lambda point: (
                (point.east_m - first.east_m) * along_east
                + (point.north_m - first.north_m) * along_north
            )
        )
        stripe_endpoints: tuple[Point, ...] = (
            (intersections[0], intersections[-1])
            if math.dist(intersections[0], intersections[-1]) > GEOMETRY_TOLERANCE_M
            else (intersections[0],)
        )
        if stripe_index % 2:
            stripe_endpoints = tuple(reversed(stripe_endpoints))
        waypoints.extend(Point(point.east_m, point.north_m) for point in stripe_endpoints)
    return waypoints


def plan_circle_lawnmower(
    center: Point,
    radius_m: float,
    stripe_gap_m: float,
    stripe_heading_rad: float,
) -> list[Point]:
    """Generate alternating chords across a bounded circular search area."""
    if not all(
        math.isfinite(value) for value in (*center, radius_m, stripe_gap_m, stripe_heading_rad)
    ):
        raise ValueError('circle search values must be finite')
    if radius_m <= 0.0 or stripe_gap_m <= 0.0:
        raise ValueError('circle radius and stripe gap must be positive')
    stripe_intervals: float = 2.0 * radius_m / stripe_gap_m
    if stripe_intervals > (MAXIMUM_MISSION_WAYPOINTS // 2) - 1:
        raise ValueError('stripe gap would exceed the mission waypoint limit')

    along_east: float = math.cos(stripe_heading_rad)
    along_north: float = math.sin(stripe_heading_rad)
    normal_east: float = -along_north
    normal_north: float = along_east
    waypoints: list[Point] = []
    for stripe_index in range(math.ceil(stripe_intervals) + 1):
        offset_m: float = min(-radius_m + stripe_index * stripe_gap_m, radius_m)
        half_length_m: float = math.sqrt(max(0.0, radius_m**2 - offset_m**2))
        center_east_m: float = center.east_m + offset_m * normal_east
        center_north_m: float = center.north_m + offset_m * normal_north
        if half_length_m <= GEOMETRY_TOLERANCE_M:
            endpoints: tuple[Point, ...] = (Point(center_east_m, center_north_m),)
        else:
            endpoints = (
                Point(
                    center_east_m - half_length_m * along_east,
                    center_north_m - half_length_m * along_north,
                ),
                Point(
                    center_east_m + half_length_m * along_east,
                    center_north_m + half_length_m * along_north,
                ),
            )
        if stripe_index % 2:
            endpoints = tuple(reversed(endpoints))
        waypoints.extend(Point(endpoint.east_m, endpoint.north_m) for endpoint in endpoints)
    return waypoints
