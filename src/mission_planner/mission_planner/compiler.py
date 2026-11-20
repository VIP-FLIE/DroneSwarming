# Portions Copyright (c) 2026, vijaymanbajracharya. See LICENSE.sar-drone.
"""Compile explicit fleet assignments into one vehicle's mission-origin offsets."""

import math
import re
from dataclasses import dataclass
from typing import Literal, cast

from .flight_plan import (
    MAXIMUM_ACCEPTANCE_RADIUS_M,
    CircleOrbit,
    JsonValue,
    object_fields,
    finite_number,
    expand_path,
    decode_document,
    parse_legacy_fields,
)
from .geography import GeographicOrigin, enu_to_geographic, geographic_to_enu
from .lawnmower import MAXIMUM_MISSION_WAYPOINTS, Point

MAXIMUM_ASSIGNMENTS: int = 64
HARD_MAXIMUM_ALTITUDE_M: float = 120.0
HARD_MAXIMUM_DISTANCE_M: float = 1000.0
MAXIMUM_ORBIT_TIMEOUT_SEC: float = 600.0
MINIMUM_ORBIT_TIMEOUT_SEC: float = 60.0
ORBIT_APPROACH_MARGIN_SEC: float = 30.0
MAXIMUM_ORBIT_ACCELERATION_MPS2: float = 2.0
VEHICLE_ID_PATTERN: re.Pattern[str] = re.compile(r'[A-Za-z][A-Za-z0-9_-]{0,63}\Z')
Frame = Literal['local_enu', 'relative_enu', 'wgs84']


@dataclass(frozen=True)
class PlanningLimits:
    """Bound each vehicle's flight and retain the execution boundary's orbit limits."""

    maximum_altitude_m: float = 30.0
    maximum_distance_m: float = 100.0
    maximum_orbit_radius_m: float = 20.0
    maximum_orbit_speed_mps: float = 3.0
    maximum_orbit_center_distance_m: float = 100.0

    def __post_init__(self) -> None:
        """Reject non-finite limits and values beyond the inherited hard bounds."""
        limits: tuple[tuple[float, float], ...] = (
            (self.maximum_altitude_m, HARD_MAXIMUM_ALTITUDE_M),
            (self.maximum_distance_m, HARD_MAXIMUM_DISTANCE_M),
            (self.maximum_orbit_radius_m, HARD_MAXIMUM_DISTANCE_M),
            (self.maximum_orbit_speed_mps, 10.0),
            (self.maximum_orbit_center_distance_m, HARD_MAXIMUM_DISTANCE_M),
        )
        if any(not math.isfinite(value) or not 0.0 < value <= maximum for value, maximum in limits):
            raise ValueError('invalid planner altitude, distance, radius, or speed limit')


@dataclass(frozen=True)
class StartingPosition:
    """Capture local coordinates and an optional validated geographic frame reference."""

    east_m: float
    north_m: float
    up_m: float
    reference: GeographicOrigin | None = None

    def __post_init__(self) -> None:
        """Reject unavailable or non-finite local coordinates before compiling a plan."""
        if not all(math.isfinite(value) for value in (self.east_m, self.north_m, self.up_m)):
            raise ValueError('starting local_enu position must be finite')


@dataclass(frozen=True)
class MissionInstructions:
    """Describe only selected high-level operations, with offsets from one start position."""

    takeoff_altitude_m: float
    waypoint_offsets: tuple[tuple[float, float, float], ...]
    position_tolerance_m: float
    orbit_center_offset: tuple[float, float, float] | None
    orbit_radius_m: float = 0.0
    orbit_speed_mps: float = 0.0
    orbit_timeout_sec: float = 0.0


@dataclass(frozen=True)
class FlightIntent:
    """Retain canonical geometry independently of any vehicle's estimator origin."""

    frame: Frame
    takeoff_altitude_m: float
    acceptance_radius_m: float
    waypoints: tuple[Point, ...]
    orbit: CircleOrbit | None
    origin: GeographicOrigin | None


def parse_origin(value: JsonValue) -> GeographicOrigin:
    """Decode an explicit shared geographic origin with an AMSL height."""
    origin: dict[str, JsonValue] = object_fields(
        value, {'latitude_deg', 'longitude_deg', 'altitude_amsl_m'}
    )
    return GeographicOrigin(
        finite_number(origin['latitude_deg'], 'latitude_deg'),
        finite_number(origin['longitude_deg'], 'longitude_deg'),
        finite_number(origin['altitude_amsl_m'], 'altitude_amsl_m'),
    )


def geographic_pair(value: JsonValue, altitude_amsl_m: float) -> GeographicOrigin:
    """Decode a latitude/longitude pair without using GeoJSON's longitude-first order."""
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError('GPS coordinates must be [latitude_deg, longitude_deg] pairs')
    return GeographicOrigin(
        finite_number(value[0], 'latitude_deg'),
        finite_number(value[1], 'longitude_deg'),
        altitude_amsl_m,
    )


def project_path(
    path: dict[str, JsonValue], altitude_amsl_m: float
) -> tuple[dict[str, JsonValue], GeographicOrigin]:
    """Use a path-derived shared anchor so fleet GPS routes expand identically."""
    anchor: GeographicOrigin | None = None

    def project_point(value: JsonValue) -> list[JsonValue]:
        """Project each geographic vertex into the path's shared planning frame."""
        nonlocal anchor
        position: GeographicOrigin = geographic_pair(value, altitude_amsl_m)
        if anchor is None:
            anchor = position
        east_m, north_m, _ = geographic_to_enu(position, anchor)
        return [east_m, north_m]

    projected: dict[str, JsonValue] = dict(path)
    if path.get('type') == 'waypoints':
        points: JsonValue = path.get('points')
        if not isinstance(points, list) or not 1 <= len(points) <= MAXIMUM_MISSION_WAYPOINTS:
            raise ValueError('waypoint count must be between 1 and 256')
        projected['points'] = [project_point(point) for point in points]
    elif path.get('type') == 'circle':
        projected['center'] = project_point(path.get('center'))
    elif path.get('type') == 'lawnmower':
        area: dict[str, JsonValue] = object_fields(
            path.get('area'), {'type'}, {'vertices', 'center', 'radius_m'}
        )
        projected_area: dict[str, JsonValue] = dict(area)
        if area['type'] == 'polygon':
            vertices: JsonValue = area.get('vertices')
            if not isinstance(vertices, list) or not 3 <= len(vertices) <= 512:
                raise ValueError('polygon vertex count must be between 3 and 512')
            projected_area['vertices'] = [project_point(vertex) for vertex in vertices]
        elif area['type'] == 'circle':
            projected_area['center'] = project_point(area.get('center'))
        else:
            raise ValueError('lawnmower area type must be polygon or circle')
        projected['area'] = projected_area
    else:
        raise ValueError('path type must be waypoints, circle, or lawnmower')
    if anchor is None:
        raise ValueError('GPS path has no coordinates')
    return projected, anchor


def parse_intent(entry: dict[str, JsonValue]) -> FlightIntent:
    """Validate one assignment without needing another vehicle's local reference."""
    required: set[str] = {'frame_id', 'takeoff_altitude_m', 'waypoint_acceptance_radius_m', 'path'}
    object_fields(
        entry,
        required | {'vehicle_ids'},
        {'origin', 'altitude_amsl_m'},
    )
    frame_value: JsonValue = entry['frame_id']
    if frame_value not in ('relative_enu', 'wgs84'):
        raise ValueError('version 2 requires relative_enu or wgs84')
    # Membership validation above narrows the external string to the supported frame set.
    frame: Frame = cast(Frame, frame_value)
    altitude_m: float = finite_number(entry['takeoff_altitude_m'], 'takeoff_altitude_m', True)
    if altitude_m > HARD_MAXIMUM_ALTITUDE_M:
        raise ValueError('takeoff altitude exceeds the 120 m hard limit')
    acceptance_m: float = finite_number(
        entry['waypoint_acceptance_radius_m'], 'waypoint_acceptance_radius_m', True
    )
    if acceptance_m > MAXIMUM_ACCEPTANCE_RADIUS_M:
        raise ValueError('waypoint acceptance radius exceeds 10 m')
    origin: GeographicOrigin | None = None
    altitude_amsl_m: float | None = None
    if frame == 'wgs84':
        if 'altitude_amsl_m' not in entry or 'origin' in entry:
            raise ValueError('wgs84 requires altitude_amsl_m and does not accept origin')
        altitude_amsl_m = finite_number(entry['altitude_amsl_m'], 'altitude_amsl_m')
    elif 'altitude_amsl_m' in entry:
        raise ValueError('relative_enu uses altitude above its origin, not altitude_amsl_m')
    elif 'origin' in entry:
        origin = parse_origin(entry['origin'])
    path: JsonValue = entry['path']
    if not isinstance(path, dict):
        raise ValueError('path must be an object')
    if frame == 'wgs84' and altitude_amsl_m is not None:
        path, origin = project_path(path, altitude_amsl_m)
    waypoints, orbit = expand_path(path, HARD_MAXIMUM_DISTANCE_M)
    if any(math.hypot(*point) > HARD_MAXIMUM_DISTANCE_M for point in waypoints):
        raise ValueError('path exceeds the 1000 m hard planning envelope')
    return FlightIntent(frame, altitude_m, acceptance_m, tuple(waypoints), orbit, origin)


def select_intent(document: dict[str, JsonValue], vehicle_id: str) -> FlightIntent:
    """Require exactly one explicit assignment for this vehicle in a fleet document."""
    if not VEHICLE_ID_PATTERN.fullmatch(vehicle_id):
        raise ValueError(
            'vehicle_id must be 1 to 64 ASCII letters, digits, underscores, or hyphens, starting with a letter'
        )
    version: JsonValue = document.get('version')
    if type(version) is not int:
        raise ValueError('version must be integer 1 or 2')
    if version == 1:
        legacy = parse_legacy_fields(document, HARD_MAXIMUM_ALTITUDE_M, HARD_MAXIMUM_DISTANCE_M)
        return FlightIntent(
            'local_enu',
            legacy.takeoff_altitude_m,
            legacy.waypoint_acceptance_radius_m,
            legacy.waypoints,
            legacy.orbit,
            None,
        )
    if version != 2:
        raise ValueError('unsupported flight plan version')
    object_fields(document, {'version', 'missions'})
    missions: JsonValue = document['missions']
    if not isinstance(missions, list) or not 1 <= len(missions) <= MAXIMUM_ASSIGNMENTS:
        raise ValueError('missions must contain 1 to 64 explicit assignments')
    assigned_ids: set[str] = set()
    selected: FlightIntent | None = None
    for value in missions:
        if not isinstance(value, dict):
            raise ValueError('each mission assignment must be an object')
        identifiers: JsonValue = value.get('vehicle_ids')
        if not isinstance(identifiers, list) or not 1 <= len(identifiers) <= MAXIMUM_ASSIGNMENTS:
            raise ValueError('vehicle_ids must contain 1 to 64 explicit vehicle IDs')
        for identifier in identifiers:
            if not isinstance(identifier, str) or not VEHICLE_ID_PATTERN.fullmatch(identifier):
                raise ValueError('invalid assignment vehicle ID')
            if identifier in assigned_ids:
                raise ValueError(f'duplicate assignment for vehicle {identifier}')
            assigned_ids.add(identifier)
        intent: FlightIntent = parse_intent(value)
        if vehicle_id in identifiers:
            selected = intent
    if selected is None:
        raise ValueError(f'no mission assignment for vehicle {vehicle_id}')
    return selected


def compile_plan(
    plan_json: str,
    vehicle_id: str,
    start: StartingPosition,
    limits: PlanningLimits = PlanningLimits(),
) -> MissionInstructions:
    """Translate the selected intent into offsets from this vehicle's captured mission start."""
    intent: FlightIntent = select_intent(decode_document(plan_json), vehicle_id)
    if intent.origin is not None and start.reference is None:
        raise ValueError('GPS or shared-origin flight requires a valid geographic reference')
    takeoff_climb_m: float = intent.takeoff_altitude_m
    if intent.frame == 'local_enu':
        takeoff_climb_m -= start.up_m
    if not 0.0 < takeoff_climb_m <= limits.maximum_altitude_m:
        raise ValueError('takeoff climb exceeds the configured altitude limit')

    def offset(point: Point) -> tuple[float, float, float]:
        """Convert a canonical path point into the captured vehicle-start coordinate contract."""
        if intent.origin is not None and start.reference is not None:
            height_m: float = 0.0 if intent.frame == 'wgs84' else intent.takeoff_altitude_m
            geographic: GeographicOrigin = enu_to_geographic(*point, height_m, intent.origin)
            east_m, north_m, up_m = geographic_to_enu(geographic, start.reference)
            return east_m - start.east_m, north_m - start.north_m, up_m - start.up_m
        if intent.frame == 'relative_enu':
            return point.east_m, point.north_m, intent.takeoff_altitude_m
        return point.east_m - start.east_m, point.north_m - start.north_m, takeoff_climb_m

    def validate_offset(position: tuple[float, float, float], radius_m: float = 0.0) -> None:
        """Check the entire selected path envelope before mission execution can take off."""
        if not all(math.isfinite(value) for value in position):
            raise ValueError('translated coordinates must be finite')
        if math.hypot(position[0], position[1]) + radius_m > limits.maximum_distance_m:
            raise ValueError('path exceeds the configured horizontal distance from mission start')
        if not 0.0 < position[2] <= limits.maximum_altitude_m:
            raise ValueError(
                'path altitude must be above mission start and within the configured limit'
            )

    offsets: tuple[tuple[float, float, float], ...] = tuple(
        offset(point) for point in intent.waypoints
    )
    for position in offsets:
        validate_offset(position)
    if intent.orbit is None:
        return MissionInstructions(takeoff_climb_m, offsets, intent.acceptance_radius_m, None)
    orbit: CircleOrbit = intent.orbit
    center: tuple[float, float, float] = offset(orbit.center)
    validate_offset(center, orbit.radius_m)
    if (
        math.hypot(center[0] + start.east_m, center[1] + start.north_m)
        > limits.maximum_orbit_center_distance_m
    ):
        raise ValueError('orbit center exceeds the vehicle-control local-origin distance limit')
    if (
        orbit.radius_m > limits.maximum_orbit_radius_m
        or orbit.speed_m_s > limits.maximum_orbit_speed_mps
        or orbit.speed_m_s**2 / orbit.radius_m >= MAXIMUM_ORBIT_ACCELERATION_MPS2
    ):
        raise ValueError('orbit exceeds configured radius, speed, or acceleration limits')
    timeout_sec: float = math.tau * orbit.radius_m / orbit.speed_m_s + ORBIT_APPROACH_MARGIN_SEC
    if not math.isfinite(timeout_sec) or timeout_sec > MAXIMUM_ORBIT_TIMEOUT_SEC:
        raise ValueError('orbit duration exceeds the 600 second planning limit')
    return MissionInstructions(
        takeoff_climb_m,
        offsets,
        intent.acceptance_radius_m,
        center,
        orbit.radius_m,
        orbit.speed_m_s,
        max(MINIMUM_ORBIT_TIMEOUT_SEC, timeout_sec),
    )
