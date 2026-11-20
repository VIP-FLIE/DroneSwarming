"""Check geographic compilation, shared-map alignment, relative headings, and bounds."""

import json
import math

import pytest
from mission_planner.compiler import (
    HARD_MAXIMUM_DISTANCE_M,
    MissionInstructions,
    PlanningLimits,
    StartingPosition,
    compile_plan,
)
from mission_planner.flight_plan import JsonValue
from swarm_geometry.geography import (
    GeographicOrigin,
    enu_to_geographic,
    geographic_to_enu,
)

REFERENCE = GeographicOrigin(47.0, 8.0, 500.0)


def assignment(vehicle_ids: list[str] | None = None) -> dict[str, JsonValue]:
    """Create an explicitly assigned route in mission-start ENU meters."""
    return {
        'vehicle_ids': vehicle_ids or ['uav_1'],
        'frame_id': 'relative_enu',
        'takeoff_altitude_m': 3.0,
        'waypoint_acceptance_radius_m': 0.5,
        'path': {'type': 'waypoints', 'points': [[3.0, 0.0], [3.0, 3.0]]},
    }


def document(*missions: dict[str, JsonValue]) -> str:
    """Encode explicit fleet assignments."""
    return json.dumps({'version': 2, 'missions': missions or [assignment()]})


def start(east_m: float = 0, north_m: float = 0, up_m: float = 0) -> StartingPosition:
    """Provide a geographic anchor independent of the tested route."""
    return StartingPosition(east_m, north_m, up_m, REFERENCE)


def test_relative_targets_resolve_once_at_the_captured_position() -> None:
    planned: MissionInstructions = compile_plan(document(), 'uav_1', start(50, 70, 2))
    anchor = enu_to_geographic(50, 70, 2, REFERENCE)
    assert geographic_to_enu(planned.waypoint_positions[0], anchor) == pytest.approx(
        (3, 0, 3), abs=1e-8
    )
    assert geographic_to_enu(planned.waypoint_positions[1], anchor) == pytest.approx(
        (3, 3, 3), abs=1e-8
    )
    assert planned.takeoff_altitude_m == 3


@pytest.mark.parametrize(
    'yaw_rad,expected',
    [(0.0, (3, 0, 3)), (math.pi / 2, (0, 3, 3)), (math.pi, (-3, 0, 3))],
)
def test_forward_follows_captured_heading(
    yaw_rad: float, expected: tuple[float, float, float]
) -> None:
    mission = assignment()
    mission['frame_id'] = 'relative_body'
    planned = compile_plan(
        document(mission), 'uav_1', StartingPosition(0, 0, 0, REFERENCE, yaw_rad)
    )
    assert geographic_to_enu(planned.waypoint_positions[0], REFERENCE) == pytest.approx(
        expected, abs=1e-8
    )


def test_body_frame_requires_valid_heading() -> None:
    mission = assignment()
    mission['frame_id'] = 'relative_body'
    for yaw in (None, math.nan, math.inf):
        with pytest.raises(ValueError, match='yaw'):
            compile_plan(document(mission), 'uav_1', StartingPosition(0, 0, 0, REFERENCE, yaw))


def test_every_mission_requires_geographic_localization() -> None:
    with pytest.raises(ValueError, match='geographic reference'):
        compile_plan(document(), 'uav_1', StartingPosition(0, 0, 0))


def test_legacy_local_origin_and_absolute_altitude_are_preserved() -> None:
    encoded = json.dumps(
        {
            'version': 1,
            'frame_id': 'local_enu',
            'takeoff_altitude_m': 5,
            'waypoint_acceptance_radius_m': 1,
            'path': {'type': 'waypoints', 'points': [[8, 7]]},
        }
    )
    planned = compile_plan(encoded, 'uav_1', start(2, 3, 1))
    assert planned.takeoff_altitude_m == 4
    assert geographic_to_enu(planned.waypoint_positions[0], REFERENCE) == pytest.approx(
        (8, 7, 5), abs=1e-8
    )


def test_fleet_members_select_only_their_assignment() -> None:
    second = assignment(['uav_2'])
    second['path'] = {'type': 'waypoints', 'points': [[-4, 0]]}
    encoded = document(assignment(), second)
    first_plan = compile_plan(encoded, 'uav_1', start())
    second_plan = compile_plan(encoded, 'uav_2', start())
    assert geographic_to_enu(first_plan.waypoint_positions[0], REFERENCE)[0] == pytest.approx(3)
    assert geographic_to_enu(second_plan.waypoint_positions[0], REFERENCE)[0] == pytest.approx(-4)
    with pytest.raises(ValueError, match='no mission assignment'):
        compile_plan(encoded, 'uav_3', start())


@pytest.mark.parametrize('identifiers', [['uav_1', 'uav_1'], ['*']])
def test_duplicate_and_wildcard_vehicle_ids_are_rejected(
    identifiers: list[str],
) -> None:
    with pytest.raises(ValueError):
        compile_plan(document(assignment(identifiers)), 'uav_1', start())


def test_unselected_assignment_is_still_schema_validated() -> None:
    other = assignment(['uav_2'])
    other['unknown'] = True
    with pytest.raises(ValueError, match='expected fields'):
        compile_plan(document(assignment(), other), 'uav_1', start())


def test_exact_gps_waypoint_is_preserved_without_a_projection_round_trip() -> None:
    mission = assignment()
    mission['frame_id'] = 'wgs84'
    mission['altitude_amsl_m'] = 507.0
    mission['path'] = {
        'type': 'waypoints',
        'points': [[47.000123456789, 8.000234567891]],
    }
    planned = compile_plan(document(mission), 'uav_1', start(10, 20, 2))
    assert planned.waypoint_positions == (GeographicOrigin(47.000123456789, 8.000234567891, 507),)


@pytest.mark.parametrize('frame_id', ['relative_enu', 'shared_enu'])
def test_shared_anchor_gives_identical_targets_for_different_estimator_origins(
    frame_id: str,
) -> None:
    mission = assignment(['uav_1', 'uav_2'])
    mission['frame_id'] = frame_id
    mission['origin'] = {'latitude_deg': 47, 'longitude_deg': 8, 'altitude_amsl_m': 500}
    other_reference = enu_to_geographic(40, 10, 0, REFERENCE)
    first = compile_plan(document(mission), 'uav_1', start())
    second = compile_plan(document(mission), 'uav_2', StartingPosition(0, 0, 0, other_reference))
    assert first.waypoint_positions == second.waypoint_positions


def test_shared_frame_requires_an_anchor_and_body_frame_rejects_one() -> None:
    mission = assignment()
    mission['frame_id'] = 'shared_enu'
    with pytest.raises(ValueError, match='origin'):
        compile_plan(document(mission), 'uav_1', start())
    mission['frame_id'] = 'relative_body'
    mission['origin'] = {'latitude_deg': 47, 'longitude_deg': 8, 'altitude_amsl_m': 500}
    with pytest.raises(ValueError, match='origin'):
        compile_plan(document(mission), 'uav_1', start())


def test_gps_lawnmower_geometry_is_identical_across_vehicle_frames() -> None:
    corners = [
        enu_to_geographic(east, north, 0, REFERENCE)
        for east, north in ((0, 0), (10, 0), (10, 6), (0, 6))
    ]
    mission = assignment(['uav_1', 'uav_2'])
    mission['frame_id'] = 'wgs84'
    mission['altitude_amsl_m'] = 503
    mission['path'] = {
        'type': 'lawnmower',
        'area': {
            'type': 'polygon',
            'vertices': [[corner.latitude_deg, corner.longitude_deg] for corner in corners],
        },
        'stripe_gap_m': 2,
        'stripe_heading_deg': 0,
    }
    first = compile_plan(document(mission), 'uav_1', start())
    second = compile_plan(
        document(mission),
        'uav_2',
        StartingPosition(0, 0, 0, enu_to_geographic(20, 0, 0, REFERENCE)),
    )
    assert first.waypoint_positions == second.waypoint_positions


@pytest.mark.parametrize(
    'encoded',
    [
        '{"version":2,"version":2}',
        '{"version":NaN}',
        '[]',
        '{"version":true}',
        '{"version":2,"missions":[]}',
        '[' * 1100 + '0' + ']' * 1100,
        ' ' * 65537,
    ],
)
def test_invalid_documents_never_produce_instructions(encoded: str) -> None:
    with pytest.raises(ValueError):
        compile_plan(encoded, 'uav_1', start())


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
        compile_plan(document(assignment() | change), 'uav_1', start())


def test_entire_route_is_bounded_before_takeoff() -> None:
    mission = assignment()
    mission['path'] = {'type': 'waypoints', 'points': [[1, 0], [101, 0]]}
    with pytest.raises(ValueError, match='horizontal distance'):
        compile_plan(document(mission), 'uav_1', start())
    with pytest.raises(ValueError):
        PlanningLimits(maximum_distance_m=HARD_MAXIMUM_DISTANCE_M + 1)


def test_circle_limits_include_radius_and_timing() -> None:
    mission = assignment()
    mission['path'] = {
        'type': 'circle',
        'center': [2, 3],
        'radius_m': 5,
        'speed_m_s': 1,
    }
    planned = compile_plan(document(mission), 'uav_1', start())
    assert planned.waypoint_positions == ()
    assert planned.orbit_center_geographic is not None
    assert geographic_to_enu(planned.orbit_center_geographic, REFERENCE) == pytest.approx(
        (2, 3, 3), abs=1e-8
    )
    mission['path'] = {'type': 'circle', 'center': [99, 0], 'radius_m': 5}
    with pytest.raises(ValueError, match='horizontal distance'):
        compile_plan(document(mission), 'uav_1', start())
    mission['path'] = {
        'type': 'circle',
        'center': [0, 0],
        'radius_m': 5,
        'speed_m_s': 1e-300,
    }
    with pytest.raises(ValueError, match='duration'):
        compile_plan(document(mission), 'uav_1', start())


def test_circle_obeys_local_backend_limit_before_takeoff() -> None:
    mission = assignment()
    mission['path'] = {'type': 'circle', 'center': [2, 0], 'radius_m': 5}
    with pytest.raises(ValueError, match='vehicle-control local-origin'):
        compile_plan(document(mission), 'uav_1', start(99))


def test_unselected_remote_route_does_not_use_this_vehicles_flight_envelope() -> None:
    remote = assignment(['uav_2'])
    remote['frame_id'] = 'wgs84'
    remote['altitude_amsl_m'] = 503
    remote['path'] = {'type': 'waypoints', 'points': [[20, -20]]}
    planned = compile_plan(document(assignment(), remote), 'uav_1', start())
    assert len(planned.waypoint_positions) == 2


def test_duplicate_vehicle_ids_across_assignments_are_rejected() -> None:
    with pytest.raises(ValueError, match='duplicate assignment'):
        compile_plan(document(assignment(), assignment()), 'uav_1', start())


@pytest.mark.parametrize('point', [[100, 0], [0, 100]])
def test_exact_metric_boundary_survives_geographic_projection(point: list[float]) -> None:
    mission = assignment()
    mission['path'] = {'type': 'waypoints', 'points': [point]}
    assert len(compile_plan(document(mission), 'uav_1', start()).waypoint_positions) == 1
    mission['path'] = {'type': 'waypoints', 'points': [[100.0001, 0]]}
    with pytest.raises(ValueError, match='horizontal distance'):
        compile_plan(document(mission), 'uav_1', start())
