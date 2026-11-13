"""Check frame conversion, fleet isolation, and validation before vehicle commands."""

import json
import math
from dataclasses import replace

import pytest
from mission_planner.compiler import (
    HARD_MAXIMUM_DISTANCE_M,
    MissionInstructions,
    PlanningLimits,
    StartingPosition,
    compile_plan,
)
from mission_planner.flight_plan import JsonValue
from mission_planner.geography import (
    EARTH_RADIUS_M,
    GeographicOrigin,
    enu_to_geographic,
    geographic_to_enu,
)


def assignment(vehicle_ids: list[str] | None = None) -> dict[str, JsonValue]:
    """Create a two-point route relative to the executing vehicle's captured start."""
    return {
        'vehicle_ids': vehicle_ids or ['uav_1'],
        'frame_id': 'relative_enu',
        'takeoff_altitude_m': 3.0,
        'waypoint_acceptance_radius_m': 0.5,
        'path': {'type': 'waypoints', 'points': [[3.0, 0.0], [3.0, 3.0]]},
    }


def document(*missions: dict[str, JsonValue]) -> str:
    """Encode a fleet document with explicit per-vehicle assignments."""
    return json.dumps({'version': 2, 'missions': missions or [assignment()]})


def test_vehicle_relative_origin_is_captured_once() -> None:
    planned: MissionInstructions = compile_plan(
        document(), 'uav_1', StartingPosition(50.0, 70.0, 2.0)
    )
    assert planned.waypoint_offsets == ((3.0, 0.0, 3.0), (3.0, 3.0, 3.0))
    assert planned.takeoff_altitude_m == 3.0


def test_legacy_local_origin_and_absolute_altitude_are_preserved() -> None:
    encoded: str = json.dumps(
        {
            'version': 1,
            'frame_id': 'local_enu',
            'takeoff_altitude_m': 5.0,
            'waypoint_acceptance_radius_m': 1.0,
            'path': {'type': 'waypoints', 'points': [[8, 7]]},
        }
    )
    planned = compile_plan(encoded, 'uav_1', StartingPosition(2, 3, 1))
    assert planned.takeoff_altitude_m == 4.0
    assert planned.waypoint_offsets == ((6.0, 4.0, 4.0),)


def test_fleet_members_get_only_their_explicit_assignment() -> None:
    first = assignment(['uav_1'])
    second = assignment(['uav_2'])
    second['path'] = {'type': 'waypoints', 'points': [[-4, 0]]}
    encoded: str = document(first, second)
    assert compile_plan(encoded, 'uav_1', StartingPosition(0, 0, 0)).waypoint_offsets[0][0] == 3
    assert compile_plan(encoded, 'uav_2', StartingPosition(0, 0, 0)).waypoint_offsets[0][0] == -4
    with pytest.raises(ValueError, match='no mission assignment'):
        compile_plan(encoded, 'uav_3', StartingPosition(0, 0, 0))


def test_explicit_multi_vehicle_assignment_has_no_wildcard_fallback() -> None:
    encoded: str = document(assignment(['uav_1', 'uav_2']))
    first = compile_plan(encoded, 'uav_1', StartingPosition(1, 2, 0))
    second = compile_plan(encoded, 'uav_2', StartingPosition(5, 8, 0))
    assert first == second
    with pytest.raises(ValueError, match='invalid assignment vehicle ID'):
        compile_plan(document(assignment(['*'])), 'uav_1', StartingPosition(0, 0, 0))


@pytest.mark.parametrize(
    'missions',
    [
        [assignment(['uav_1', 'uav_1'])],
        [assignment(['uav_1']), assignment(['uav_1'])],
    ],
)
def test_duplicate_assignments_are_rejected(missions: list[dict[str, JsonValue]]) -> None:
    with pytest.raises(ValueError, match='duplicate assignment'):
        compile_plan(document(*missions), 'uav_1', StartingPosition(0, 0, 0))


def test_unselected_assignment_cannot_hide_invalid_json_fields() -> None:
    other = assignment(['uav_2'])
    other['unknown'] = True
    with pytest.raises(ValueError, match='expected fields'):
        compile_plan(document(assignment(), other), 'uav_1', StartingPosition(0, 0, 0))


@pytest.mark.parametrize('latitude_deg,longitude_deg', [(0.0, 0.0), (60.0, 10.0), (0.0, 179.999)])
def test_geographic_round_trip_and_axis_convention(
    latitude_deg: float, longitude_deg: float
) -> None:
    reference = GeographicOrigin(latitude_deg, longitude_deg, 500.0)
    destination = enu_to_geographic(20.0, 30.0, 5.0, reference)
    assert geographic_to_enu(destination, reference) == pytest.approx((20.0, 30.0, 5.0), abs=1e-8)
    east = enu_to_geographic(100.0, 0.0, 0.0, GeographicOrigin(0, 0, 0))
    assert east.longitude_deg == pytest.approx(math.degrees(100.0 / EARTH_RADIUS_M))


def test_absolute_gps_uses_real_reference_and_explicit_amsl_altitude() -> None:
    reference = GeographicOrigin(47.0, 8.0, 500.0)
    destination = enu_to_geographic(13.0, 24.0, 7.0, reference)
    mission = assignment()
    mission['frame_id'] = 'wgs84'
    mission['altitude_amsl_m'] = destination.altitude_amsl_m
    mission['path'] = {
        'type': 'waypoints',
        'points': [[destination.latitude_deg, destination.longitude_deg]],
    }
    start = StartingPosition(10.0, 20.0, 2.0, reference)
    planned = compile_plan(document(mission), 'uav_1', start)
    assert planned.waypoint_offsets[0] == pytest.approx((3.0, 4.0, 5.0), abs=1e-8)
    with pytest.raises(ValueError, match='valid geographic reference'):
        compile_plan(document(mission), 'uav_1', replace(start, reference=None))


def test_shared_relative_anchor_is_independent_of_vehicle_estimator_origin() -> None:
    origin = GeographicOrigin(47.0, 8.0, 500.0)
    second_reference = enu_to_geographic(40.0, 10.0, 0.0, origin)
    mission = assignment(['uav_1', 'uav_2'])
    mission['origin'] = {'latitude_deg': 47.0, 'longitude_deg': 8.0, 'altitude_amsl_m': 500.0}
    encoded = document(mission)
    first = compile_plan(encoded, 'uav_1', StartingPosition(0, 0, 0, origin))
    second = compile_plan(encoded, 'uav_2', StartingPosition(0, 0, 0, second_reference))
    first_global = enu_to_geographic(*first.waypoint_offsets[0], origin)
    second_global = enu_to_geographic(*second.waypoint_offsets[0], second_reference)
    assert first_global.latitude_deg == pytest.approx(second_global.latitude_deg, abs=1e-10)
    assert first_global.longitude_deg == pytest.approx(second_global.longitude_deg, abs=1e-10)
    assert first_global.altitude_amsl_m == second_global.altitude_amsl_m


def test_gps_lawnmower_expands_identically_for_two_vehicle_references() -> None:
    reference = GeographicOrigin(47.0, 8.0, 500.0)
    corners = [
        enu_to_geographic(east, north, 0, reference)
        for east, north in ((0, 0), (10, 0), (10, 6), (0, 6))
    ]
    mission = assignment(['uav_1', 'uav_2'])
    mission['frame_id'] = 'wgs84'
    mission['altitude_amsl_m'] = 503.0
    mission['path'] = {
        'type': 'lawnmower',
        'area': {
            'type': 'polygon',
            'vertices': [[corner.latitude_deg, corner.longitude_deg] for corner in corners],
        },
        'stripe_gap_m': 2.0,
        'stripe_heading_deg': 0.0,
    }
    other_reference = enu_to_geographic(20, 0, 0, reference)
    first = compile_plan(document(mission), 'uav_1', StartingPosition(0, 0, 0, reference))
    second = compile_plan(document(mission), 'uav_2', StartingPosition(0, 0, 0, other_reference))
    assert len(first.waypoint_offsets) == len(second.waypoint_offsets)
    for first_point, second_point in zip(
        first.waypoint_offsets, second.waypoint_offsets, strict=True
    ):
        first_global = enu_to_geographic(*first_point, reference)
        second_global = enu_to_geographic(*second_point, other_reference)
        assert first_global.latitude_deg == pytest.approx(second_global.latitude_deg, abs=1e-10)
        assert first_global.longitude_deg == pytest.approx(second_global.longitude_deg, abs=1e-10)


def test_remote_unselected_gps_route_does_not_use_this_vehicle_geofence() -> None:
    remote = assignment(['uav_2'])
    remote['frame_id'] = 'wgs84'
    remote['altitude_amsl_m'] = 503.0
    remote['path'] = {'type': 'waypoints', 'points': [[20.0, -20.0]]}
    assert (
        compile_plan(
            document(assignment(), remote), 'uav_1', StartingPosition(0, 0, 0)
        ).takeoff_altitude_m
        == 3.0
    )


@pytest.mark.parametrize(
    'encoded',
    [
        '{"version":2,"version":2}',
        '{"version":NaN}',
        '[]',
        json.dumps({'version': True}),
        '{"version":2,"missions":[]}',
        '[' * 1100 + '0' + ']' * 1100,
        ' ' * 65537,
    ],
)
def test_invalid_documents_never_produce_instructions(encoded: str) -> None:
    with pytest.raises(ValueError):
        compile_plan(encoded, 'uav_1', StartingPosition(0, 0, 0))


@pytest.mark.parametrize(
    'change',
    [
        {'takeoff_altitude_m': True},
        {'takeoff_altitude_m': math.inf},
        {'takeoff_altitude_m': -1},
        {'frame_id': 'ned'},
        {'waypoint_acceptance_radius_m': 0},
        {'origin': {'latitude_deg': 91, 'longitude_deg': 0, 'altitude_amsl_m': 0}},
    ],
)
def test_invalid_assignment_values_are_rejected(change: dict[str, JsonValue]) -> None:
    with pytest.raises(ValueError):
        compile_plan(document(assignment() | change), 'uav_1', StartingPosition(0, 0, 0))


def test_entire_route_is_bounded_before_takeoff() -> None:
    mission = assignment()
    mission['path'] = {'type': 'waypoints', 'points': [[1, 0], [101, 0]]}
    with pytest.raises(ValueError, match='horizontal distance'):
        compile_plan(document(mission), 'uav_1', StartingPosition(0, 0, 0))
    with pytest.raises(ValueError):
        PlanningLimits(maximum_distance_m=HARD_MAXIMUM_DISTANCE_M + 1)


def test_circle_limits_include_the_entire_circle_and_timing() -> None:
    mission = assignment()
    mission['path'] = {'type': 'circle', 'center': [2, 3], 'radius_m': 5, 'speed_m_s': 1}
    planned = compile_plan(document(mission), 'uav_1', StartingPosition(50, 20, 2))
    assert planned.waypoint_offsets == ()
    assert planned.orbit_center_offset == (2, 3, 3)
    assert planned.orbit_radius_m == 5
    mission['path'] = {'type': 'circle', 'center': [99, 0], 'radius_m': 5}
    with pytest.raises(ValueError, match='horizontal distance'):
        compile_plan(document(mission), 'uav_1', StartingPosition(0, 0, 0))
    mission['path'] = {'type': 'circle', 'center': [0, 0], 'radius_m': 5, 'speed_m_s': 1e-300}
    with pytest.raises(ValueError, match='duration'):
        compile_plan(document(mission), 'uav_1', StartingPosition(0, 0, 0))


def test_circle_is_checked_against_controller_center_limit_before_takeoff() -> None:
    mission = assignment()
    mission['path'] = {'type': 'circle', 'center': [2, 0], 'radius_m': 5}
    with pytest.raises(ValueError, match='vehicle-control local-origin'):
        compile_plan(document(mission), 'uav_1', StartingPosition(99, 0, 0))
