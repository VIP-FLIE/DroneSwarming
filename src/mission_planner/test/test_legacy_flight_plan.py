# Portions Copyright (c) 2026, vijaymanbajracharya. See LICENSE.sar-drone.
"""Versioned JSON flight-plan parsing and path expansion tests."""

import json
import math
import unittest

from mission_planner.flight_plan import JsonValue, parse_flight_plan


class FlightPlanTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base_plan: dict[str, JsonValue] = {
            'version': 1,
            'frame_id': 'local_enu',
            'takeoff_altitude_m': 5.0,
            'waypoint_acceptance_radius_m': 1.0,
        }

    def _parse(self, path: dict[str, JsonValue]) -> tuple[tuple[float, float], ...]:
        plan_json: str = json.dumps(self.base_plan | {'path': path})
        plan = parse_flight_plan(plan_json, 30.0, 100.0)
        return tuple((point.east_m, point.north_m) for point in plan.waypoints)

    def test_explicit_waypoints_can_trace_a_triangle(self) -> None:
        points = self._parse(
            {
                'type': 'waypoints',
                'points': [[0, 0], [6, 0], [3, 5], [0, 0]],
            }
        )
        self.assertEqual(points, ((0, 0), (6, 0), (3, 5), (0, 0)))

    def test_circle_is_planned_as_a_px4_orbit(self) -> None:
        plan_json: str = json.dumps(
            self.base_plan
            | {
                'path': {
                    'type': 'circle',
                    'center': [1, 2],
                    'radius_m': 5,
                    'speed_m_s': 1.5,
                }
            }
        )
        plan = parse_flight_plan(plan_json, 30.0, 100.0)
        self.assertEqual(plan.waypoints, ())
        self.assertIsNotNone(plan.orbit)
        assert plan.orbit is not None
        self.assertEqual(plan.orbit.center.east_m, 1.0)
        self.assertEqual(plan.orbit.center.north_m, 2.0)
        self.assertEqual(plan.orbit.radius_m, 5.0)
        self.assertEqual(plan.orbit.speed_m_s, 1.5)

    def test_lawnmower_accepts_an_octagon(self) -> None:
        vertices: list[list[float]] = [
            [10.0 * math.cos(index * math.pi / 4), 10.0 * math.sin(index * math.pi / 4)]
            for index in range(8)
        ]
        points = self._parse(
            {
                'type': 'lawnmower',
                'area': {'type': 'polygon', 'vertices': vertices},
                'stripe_gap_m': 2,
                'stripe_heading_deg': 0,
            }
        )
        self.assertGreater(len(points), 8)
        self.assertLessEqual(len(points), 256)

    def test_lawnmower_circle_endpoints_lie_on_circle(self) -> None:
        points = self._parse(
            {
                'type': 'lawnmower',
                'area': {'type': 'circle', 'center': [0, 0], 'radius_m': 5},
                'stripe_gap_m': 2,
            }
        )
        self.assertTrue(all(math.isclose(math.hypot(*point), 5) for point in points))

    def test_rejects_wrong_frame_duplicate_fields_and_nonfinite_numbers(self) -> None:
        plan_json: str = json.dumps(
            self.base_plan
            | {
                'frame_id': 'wgs84',
                'path': {'type': 'waypoints', 'points': [[0, 0]]},
            }
        )
        with self.assertRaises(ValueError):
            parse_flight_plan(plan_json, 30.0, 100.0)
        with self.assertRaises(ValueError):
            parse_flight_plan('{"version":1,"version":2}', 30.0, 100.0)
        with self.assertRaises(ValueError):
            self._parse(
                {'type': 'circle', 'center': [0, 0], 'radius_m': math.nan, 'point_spacing_m': 2}
            )

    def test_rejects_out_of_bounds_area(self) -> None:
        with self.assertRaises(ValueError):
            self._parse(
                {
                    'type': 'lawnmower',
                    'area': {'type': 'circle', 'center': [99, 0], 'radius_m': 5},
                    'stripe_gap_m': 2,
                }
            )


if __name__ == '__main__':
    unittest.main()
