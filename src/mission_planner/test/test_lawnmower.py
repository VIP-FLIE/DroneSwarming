# Portions Copyright (c) 2026, vijaymanbajracharya. See LICENSE.sar-drone.
"""Geometry tests for bounded lawnmower mission generation."""

import math
import unittest

from mission_planner.lawnmower import Point, plan_lawnmower


class LawnmowerTests(unittest.TestCase):
    def test_rectangle_alternates_direction_and_reaches_both_edges(self) -> None:
        corners: list[Point] = [
            Point(0.0, 0.0),
            Point(10.0, 0.0),
            Point(10.0, 6.0),
            Point(0.0, 6.0),
        ]
        waypoints = plan_lawnmower(corners, 2.0)
        self.assertEqual(len(waypoints), 8)
        self.assertEqual(
            [(point.east_m, point.north_m) for point in waypoints],
            [(0, 0), (10, 0), (10, 2), (0, 2), (0, 4), (10, 4), (10, 6), (0, 6)],
        )

    def test_rotated_clockwise_area_keeps_requested_gap(self) -> None:
        corners: list[Point] = [
            Point(0.0, 0.0),
            Point(0.0, 10.0),
            Point(6.0, 10.0),
            Point(6.0, 0.0),
        ]
        waypoints = plan_lawnmower(corners, 3.0)
        self.assertEqual(len(waypoints), 6)
        self.assertEqual(
            [(point.east_m, point.north_m) for point in waypoints],
            [(0, 0), (0, 10), (3, 10), (3, 0), (6, 0), (6, 10)],
        )

    def test_skew_quadrilateral_clips_stripes_to_area(self) -> None:
        corners: list[Point] = [
            Point(0.0, 0.0),
            Point(10.0, 0.0),
            Point(8.0, 6.0),
            Point(1.0, 5.0),
        ]
        waypoints = plan_lawnmower(corners, 2.0)
        self.assertEqual(len(waypoints), 7)
        self.assertAlmostEqual(waypoints[2].east_m, 9.333333333333334)
        self.assertAlmostEqual(waypoints[3].east_m, 0.4)
        self.assertEqual((waypoints[-1].east_m, waypoints[-1].north_m), (8, 6))

    def test_rejects_invalid_area_or_waypoint_count(self) -> None:
        rectangle: list[Point] = [
            Point(0.0, 0.0),
            Point(10.0, 0.0),
            Point(10.0, 6.0),
            Point(0.0, 6.0),
        ]
        crossed: list[Point] = [rectangle[0], rectangle[2], rectangle[1], rectangle[3]]
        with self.assertRaises(ValueError):
            plan_lawnmower(crossed, 2.0)
        with self.assertRaises(ValueError):
            plan_lawnmower(rectangle, 0.01)
        with self.assertRaises(ValueError):
            plan_lawnmower(rectangle, math.nan)
        star: list[Point] = [
            Point(math.cos(4 * math.pi * index / 5), math.sin(4 * math.pi * index / 5))
            for index in range(5)
        ]
        with self.assertRaises(ValueError):
            plan_lawnmower(star, 0.2)


if __name__ == '__main__':
    unittest.main()
