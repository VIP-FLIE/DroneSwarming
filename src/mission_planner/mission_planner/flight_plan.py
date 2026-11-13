# Portions Copyright (c) 2026, vijaymanbajracharya. See LICENSE.sar-drone.
"""Parse a versioned flight plan in the local ENU frame."""

import json
import math
from dataclasses import dataclass
from typing import NoReturn, cast

from .lawnmower import (
    MAXIMUM_AREA_VERTICES,
    MAXIMUM_MISSION_WAYPOINTS,
    Point,
    plan_circle_lawnmower,
    plan_lawnmower,
)

type JsonValue = None | bool | int | float | str | list[JsonValue] | dict[str, JsonValue]

PLAN_VERSION = 1
LOCAL_FRAME_ID = 'local_enu'
MAXIMUM_ACCEPTANCE_RADIUS_M = 10.0
MAXIMUM_PLAN_JSON_LENGTH = 65_536
MINIMUM_ORBIT_RADIUS_M = 1.0
MAXIMUM_ORBIT_RADIUS_M = 1000.0
MAXIMUM_ORBIT_SPEED_M_S = 10.0
DEFAULT_ORBIT_SPEED_M_S = 1.0


@dataclass(frozen=True)
class CircleOrbit:
    """Describe one counterclockwise orbit in the planning frame."""

    center: Point
    radius_m: float
    speed_m_s: float


@dataclass(frozen=True)
class FlightPlan:
    """Store expanded path geometry and the legacy constant flight altitude."""

    takeoff_altitude_m: float
    waypoint_acceptance_radius_m: float
    waypoints: tuple[Point, ...]
    orbit: CircleOrbit | None


def _reject_constant(token: str) -> NoReturn:
    """Reject JSON extensions for non-finite numbers."""
    raise ValueError(f'nonfinite JSON number {token} is not allowed')


def _unique_object(pairs: list[tuple[str, JsonValue]]) -> dict[str, JsonValue]:
    """Reject duplicate keys before a JSON object becomes a mapping."""
    fields: dict[str, JsonValue] = dict(pairs)
    if len(fields) != len(pairs):
        raise ValueError('duplicate flight plan JSON field')
    return fields


def object_fields(
    value: JsonValue,
    required: set[str],
    optional: set[str] | None = None,
) -> dict[str, JsonValue]:
    """Enforce the required and permitted fields of one JSON object."""
    if not isinstance(value, dict):
        raise ValueError('flight plan objects must be JSON objects')
    allowed: set[str] = required | (optional or set())
    if not required.issubset(value) or not set(value).issubset(allowed):
        raise ValueError(
            f'expected fields {sorted(required)} and optional {sorted(allowed - required)}'
        )
    return value


def finite_number(value: JsonValue, name: str, is_positive: bool = False) -> float:
    """Reject booleans, non-finite numbers, and invalid positive quantities."""
    if type(value) not in (int, float):
        raise ValueError(f'{name} must be a number')
    try:
        number: float = float(value)
    except OverflowError as error:
        raise ValueError(f'{name} is outside the supported numeric range') from error
    if not math.isfinite(number) or (is_positive and number <= 0.0):
        raise ValueError(f'{name} must be finite' + (' and positive' if is_positive else ''))
    return number


def _point(value: JsonValue) -> Point:
    """Decode a finite pair of planning-frame coordinates."""
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError('each point must be an [east_m, north_m] array')
    return Point(finite_number(value[0], 'east_m'), finite_number(value[1], 'north_m'))


def _points(value: JsonValue, minimum_count: int, maximum_count: int) -> list[Point]:
    """Validate the number and coordinates of path vertices."""
    if not isinstance(value, list) or not minimum_count <= len(value) <= maximum_count:
        raise ValueError(f'point count must be between {minimum_count} and {maximum_count}')
    return [_point(point) for point in value]


def _route(points: list[Point]) -> list[Point]:
    """Preserve route order and reject repeated consecutive destinations."""
    waypoints: list[Point] = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point: Point = points[index + 1]
            east_delta_m: float = next_point.east_m - point.east_m
            north_delta_m: float = next_point.north_m - point.north_m
            if math.hypot(east_delta_m, north_delta_m) <= 0.0:
                raise ValueError('consecutive waypoints must differ')
        waypoints.append(Point(point.east_m, point.north_m))
    return waypoints


def expand_path(
    path: dict[str, JsonValue],
    maximum_distance_m: float,
) -> tuple[list[Point], CircleOrbit | None]:
    """Expand a bounded waypoint, orbit, or lawnmower path."""
    path_type: JsonValue = path.get('type')
    if path_type == 'waypoints':
        object_fields(path, {'type', 'points'})
        points: list[Point] = _points(path['points'], 1, MAXIMUM_MISSION_WAYPOINTS)
        return _route(points), None
    if path_type == 'circle':
        object_fields(
            path,
            {'type', 'center', 'radius_m'},
            {
                'point_spacing_m',
                'speed_m_s',
            },
        )
        center: Point = _point(path['center'])
        radius_m: float = finite_number(path['radius_m'], 'radius_m', True)
        if 'point_spacing_m' in path:
            finite_number(path['point_spacing_m'], 'point_spacing_m', True)
        if math.hypot(*center) + radius_m > maximum_distance_m:
            raise ValueError('circle exceeds the horizontal distance limit')
        if not MINIMUM_ORBIT_RADIUS_M <= radius_m <= MAXIMUM_ORBIT_RADIUS_M:
            raise ValueError('circle radius must be between 1 and 1000 meters')
        speed_m_s: float = DEFAULT_ORBIT_SPEED_M_S
        if 'speed_m_s' in path:
            speed_m_s = finite_number(path['speed_m_s'], 'speed_m_s', True)
        maximum_speed_m_s: float = min(MAXIMUM_ORBIT_SPEED_M_S, math.sqrt(2.0 * radius_m))
        if speed_m_s > maximum_speed_m_s:
            raise ValueError(
                f'circle speed must not exceed {maximum_speed_m_s:.2f} m/s for the requested radius'
            )
        return [], CircleOrbit(center, radius_m, speed_m_s)
    if path_type == 'lawnmower':
        object_fields(path, {'type', 'area', 'stripe_gap_m'}, {'stripe_heading_deg'})
        stripe_gap_m: float = finite_number(path['stripe_gap_m'], 'stripe_gap_m', True)
        heading_rad: float | None = None
        if 'stripe_heading_deg' in path:
            heading_rad = math.radians(
                math.remainder(
                    finite_number(path['stripe_heading_deg'], 'stripe_heading_deg'), 360.0
                )
            )
        area: dict[str, JsonValue] = object_fields(
            path['area'], {'type'}, {'vertices', 'center', 'radius_m'}
        )
        if area['type'] == 'polygon':
            object_fields(area, {'type', 'vertices'})
            vertices: list[Point] = _points(area['vertices'], 3, MAXIMUM_AREA_VERTICES)
        elif area['type'] == 'circle':
            object_fields(area, {'type', 'center', 'radius_m'})
            center = _point(area['center'])
            radius_m = finite_number(area['radius_m'], 'radius_m', True)
            if math.hypot(*center) + radius_m > maximum_distance_m:
                raise ValueError('circle exceeds the horizontal distance limit')
            if heading_rad is None:
                heading_rad = 0.0
            return plan_circle_lawnmower(center, radius_m, stripe_gap_m, heading_rad), None
        else:
            raise ValueError('lawnmower area type must be polygon or circle')
        if any(math.hypot(*vertex) > maximum_distance_m for vertex in vertices):
            raise ValueError('search area exceeds the horizontal distance limit')
        return plan_lawnmower(vertices, stripe_gap_m, heading_rad), None
    raise ValueError('path type must be waypoints, circle, or lawnmower')


def decode_document(plan_json: str) -> dict[str, JsonValue]:
    """Read bounded UTF-8 JSON and reject duplicate fields and non-finite constants."""
    if (
        len(plan_json) > MAXIMUM_PLAN_JSON_LENGTH
        or len(plan_json.encode('utf-8')) > MAXIMUM_PLAN_JSON_LENGTH
    ):
        raise ValueError('flight plan JSON exceeds 65536 bytes')
    try:
        # json.loads has an Any return type; every value is validated before use.
        decoded: JsonValue = cast(
            JsonValue,
            json.loads(
                plan_json, parse_constant=_reject_constant, object_pairs_hook=_unique_object
            ),
        )
    except (json.JSONDecodeError, RecursionError) as error:
        raise ValueError(f'invalid flight plan JSON: {error}') from error
    if not isinstance(decoded, dict):
        raise ValueError('flight plan must be a JSON object')
    return decoded


def parse_flight_plan(
    plan_json: str,
    maximum_altitude_m: float,
    maximum_distance_m: float,
) -> FlightPlan:
    """Decode the SAR version 1 local-origin contract with bounded geometry."""
    return parse_legacy_fields(decode_document(plan_json), maximum_altitude_m, maximum_distance_m)


def parse_legacy_fields(
    document: dict[str, JsonValue],
    maximum_altitude_m: float,
    maximum_distance_m: float,
) -> FlightPlan:
    """Validate the legacy coordinate contract for file parsing and fleet compilation."""
    if not all(
        math.isfinite(value) and value > 0.0 for value in (maximum_altitude_m, maximum_distance_m)
    ):
        raise ValueError('legacy planning bounds must be finite and positive')
    plan: dict[str, JsonValue] = object_fields(
        document,
        {
            'version',
            'frame_id',
            'takeoff_altitude_m',
            'waypoint_acceptance_radius_m',
            'path',
        },
    )
    if type(plan['version']) is not int or plan['version'] != PLAN_VERSION:
        raise ValueError(f'flight plan version must be {PLAN_VERSION}')
    if plan['frame_id'] != LOCAL_FRAME_ID:
        raise ValueError(f'flight plan frame_id must be {LOCAL_FRAME_ID}')
    altitude_m: float = finite_number(plan['takeoff_altitude_m'], 'takeoff_altitude_m', True)
    if altitude_m > maximum_altitude_m:
        raise ValueError('flight altitude exceeds the configured limit')
    acceptance_radius_m: float = finite_number(
        plan['waypoint_acceptance_radius_m'], 'waypoint_acceptance_radius_m', True
    )
    if acceptance_radius_m > MAXIMUM_ACCEPTANCE_RADIUS_M:
        raise ValueError('waypoint acceptance radius exceeds 10 meters')
    path: dict[str, JsonValue] = object_fields(
        plan['path'],
        {'type'},
        {
            'points',
            'center',
            'radius_m',
            'point_spacing_m',
            'speed_m_s',
            'area',
            'stripe_gap_m',
            'stripe_heading_deg',
        },
    )
    waypoints, orbit = expand_path(path, maximum_distance_m)
    if any(math.hypot(point.east_m, point.north_m) > maximum_distance_m for point in waypoints):
        raise ValueError('flight path exceeds the configured horizontal distance limit')
    return FlightPlan(altitude_m, acceptance_radius_m, tuple(waypoints), orbit)
