"""Planner service tests derived from the version 2 schema and geometry contract."""

from __future__ import annotations

import copy
import json
import math
import os
import random

from swarm_interfaces.srv import PlanFlight

from support import JsonValue, VehiclePeer, WireCase, fleet_plan, starting_state

SEED = int(os.environ.get('BLIND_SEED', '20261119'))
FUZZ_COUNT = int(os.environ.get('BLIND_FUZZ_COUNT', '64'))
EARTH_RADIUS_M = 6_371_000.0


class PlannerTorture(WireCase):
    def setUp(self) -> None:
        super().setUp()
        self.peer = VehiclePeer(self)
        self.launch('mission_planner', vehicle_id='uav_1')
        self.planner()
        self.canary()

    def tearDown(self) -> None:
        self.assertEqual(self.peer.received, [], 'Planning issued a flight action')
        self.healthy()

    def reject(self, content: str) -> None:
        response = self.plan(content)
        self.assertFalse(response.success, f'Invalid plan accepted: {content[:160]}')
        self.assertTrue(response.message, 'Invalid plan requires a diagnostic')

    def test_malformed_json_and_byte_limit(self) -> None:
        corpus = [
            '',
            ' ',
            '{',
            '[',
            'null',
            'true',
            '42',
            '"plan"',
            '[]',
            '{}',
            '{"version":2,"version":2,"missions":[]}',
            '{"version":NaN}',
            '{"version":Infinity}',
            '{"version":1e309}',
            '{"version":2}\x00',
            '\ufeff{"version":2}',
            '[' * 1500 + ']' * 1500,
            '{' + ' ' * 65_536 + '}',
            '{"' + '🚁' * 17_000 + '":0}',
        ]
        for index, content in enumerate(corpus):
            with self.subTest(index=index):
                self.reject(content)
        self.canary()

    def test_schema_numeric_and_geometry_adversaries(self) -> None:
        corpus: list[dict[str, JsonValue]] = []
        for field in [
            'vehicle_ids',
            'frame_id',
            'takeoff_altitude_m',
            'waypoint_acceptance_radius_m',
            'path',
        ]:
            for value in [None, {}, [], True]:
                plan = fleet_plan()
                plan['missions'][0][field] = value
                corpus.append(plan)
            plan = fleet_plan()
            del plan['missions'][0][field]
            corpus.append(plan)
        for field, values in {
            'takeoff_altitude_m': [
                0.0,
                -1.0,
                31.0,
                1e30,
                math.nan,
                math.inf,
                -math.inf,
                '3',
            ],
            'waypoint_acceptance_radius_m': [
                0.0,
                -1.0,
                10.1,
                math.nan,
                math.inf,
                '0.5',
            ],
            'frame_id': ['ned', '', 'LOCAL_ENU', '../../fmu', '🛩'],
            'vehicle_ids': [
                [],
                ['*'],
                ['1uav'],
                ['uav/1'],
                ['é'],
                ['a' * 65],
                ['uav_1', 'uav_1'],
                [f'uav_{index}' for index in range(65)],
            ],
        }.items():
            for value in values:
                plan = fleet_plan()
                plan['missions'][0][field] = value
                corpus.append(plan)
        for points in [
            [],
            [[True, 0]],
            [[None, 0]],
            [[math.nan, 0]],
            [[math.inf, 0]],
            [[101, 0]],
            [[1]],
            [[1, 2, 3]],
            [[1, '2']],
            [[1, 0]] * 257,
        ]:
            plan = fleet_plan()
            plan['missions'][0]['path']['points'] = points
            corpus.append(plan)
        for radius_m, speed_m_s, east_m in [
            (0.9, 1, 0),
            (21, 1, 0),
            (2, 3.1, 0),
            (1, 2, 0),
            (10, 1, 95),
        ]:
            plan = fleet_plan()
            plan['missions'][0]['path'] = {
                'type': 'circle',
                'center': [east_m, 0],
                'radius_m': radius_m,
                'speed_m_s': speed_m_s,
            }
            corpus.append(plan)
        for vertices in [
            [[0, 0], [1, 1]],
            [[0, 0], [4, 4], [0, 4], [4, 0]],
            [[0, 0], [1, 0], [2, 0]],
            [[0, 0]] * 513,
        ]:
            plan = fleet_plan()
            plan['missions'][0]['path'] = {
                'type': 'lawnmower',
                'area': {'type': 'polygon', 'vertices': vertices},
                'stripe_gap_m': 1,
            }
            corpus.append(plan)
        plan = fleet_plan()
        plan['missions'] *= 65
        corpus.append(plan)
        plan = fleet_plan()
        other = copy.deepcopy(plan['missions'][0])
        other['vehicle_ids'] = ['uav_2']
        other['path']['points'] = [[False, 0]]
        plan['missions'].append(other)
        corpus.append(plan)
        for index, plan in enumerate(corpus):
            with self.subTest(index=index):
                self.reject(json.dumps(plan))
        self.canary()

    def test_seeded_invalid_flood_and_recovery(self) -> None:
        generator = random.Random(SEED)
        futures = []
        for index in range(FUZZ_COUNT):
            plan = fleet_plan()
            # Unknown root keys are forbidden independently of the randomized payload.
            plan[f'blind_{index}'] = generator.choice(
                [None, [generator.random()], {'x': '🚁' * generator.randrange(64)}]
            )
            request = PlanFlight.Request(
                plan_json=json.dumps(plan),
                vehicle_id='uav_1',
                starting_state=self.state,
            )
            self.trace('flood_request', index=index, seed=SEED, request=request)
            futures.append(self.plan_client.call_async(request))
        try:
            self.until(lambda: all(future.done() for future in futures), 10.0)
        finally:
            self.trace(
                'flood_completion',
                completed=sum(future.done() for future in futures),
                submitted=len(futures),
            )
            for index, future in enumerate(futures):
                if future.done() and future.exception() is None:
                    self.trace('flood_response', index=index, response=future.result())
        for future in futures:
            self.assertFalse(self.resolve(future).success)
        self.canary()

    def test_valid_random_routes_and_independent_coordinate_oracle(self) -> None:
        generator = random.Random(SEED)
        for index in range(FUZZ_COUNT):
            plan = fleet_plan()
            points = [
                [generator.uniform(-10, 10), generator.uniform(-10, 10)]
                for _ in range(generator.randint(1, 12))
            ]
            plan['missions'][0]['path']['points'] = points
            with self.subTest(index=index):
                response = self.plan(json.dumps(plan))
                self.assertTrue(response.success, response.message)
                self.assertEqual(len(response.waypoint_positions), len(points))
                for authored, destination in zip(points, response.waypoint_positions, strict=True):
                    # At the equator these small-angle distances are accurate within millimeters.
                    self.assertAlmostEqual(
                        math.radians(destination.longitude_deg) * EARTH_RADIUS_M,
                        authored[0],
                        delta=0.02,
                    )
                    self.assertAlmostEqual(
                        math.radians(destination.latitude_deg) * EARTH_RADIUS_M,
                        authored[1],
                        delta=0.02,
                    )
                    self.assertAlmostEqual(destination.altitude_amsl_m, 103.0, places=5)

    def test_body_rotation_gps_preservation_and_assignment_selection(self) -> None:
        plan = fleet_plan()
        plan['missions'][0]['frame_id'] = 'relative_body'
        self.state.yaw_rad = math.pi / 2
        response = self.plan(json.dumps(plan))
        self.assertTrue(response.success, response.message)
        self.assertAlmostEqual(
            math.radians(response.waypoint_positions[0].latitude_deg) * EARTH_RADIUS_M,
            3.0,
            delta=0.02,
        )
        self.assertAlmostEqual(
            math.radians(response.waypoint_positions[1].longitude_deg) * EARTH_RADIUS_M,
            -3.0,
            delta=0.02,
        )
        plan = fleet_plan()
        plan['missions'][0]['frame_id'] = 'wgs84'
        plan['missions'][0]['altitude_amsl_m'] = 104.25
        plan['missions'][0]['path']['points'] = [[0.0000123456789, 0.0000234567891]]
        response = self.plan(json.dumps(plan))
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.waypoint_positions[0].latitude_deg, 0.0000123456789)
        self.assertEqual(response.waypoint_positions[0].longitude_deg, 0.0000234567891)
        self.assertFalse(self.plan(json.dumps(plan), vehicle_id='uav_2').success)
        self.assertFalse(self.plan(json.dumps(plan), vehicle_id='*').success)

    def test_poisoned_starting_geography_and_heading(self) -> None:
        for field, value in [
            ('latitude_deg', math.nan),
            ('latitude_deg', 91.0),
            ('longitude_deg', math.inf),
            ('longitude_deg', 181.0),
            ('altitude_amsl_m', math.nan),
        ]:
            state = starting_state()
            setattr(state.geographic_position, field, value)
            with self.subTest(field=field, value=value):
                self.assertFalse(self.plan(json.dumps(fleet_plan()), state).success)
        state = starting_state()
        state.global_position_valid = False
        self.assertFalse(self.plan(json.dumps(fleet_plan()), state).success)
        plan = fleet_plan()
        plan['missions'][0]['frame_id'] = 'relative_body'
        for yaw_rad, yaw_valid in [(math.nan, True), (math.inf, True), (0.0, False)]:
            state = starting_state()
            state.yaw_rad = yaw_rad
            state.yaw_valid = yaw_valid
            self.assertFalse(self.plan(json.dumps(plan), state).success)
        self.canary()

    def test_exact_document_and_waypoint_limits(self) -> None:
        content = json.dumps(fleet_plan())
        content += ' ' * (65_536 - len(content.encode('utf-8')))
        self.assertTrue(self.plan(content).success)
        self.reject(content + ' ')
        plan = fleet_plan()
        plan['missions'][0]['path']['points'] = [[1.0 + index / 256.0, 0.0] for index in range(256)]
        response = self.plan(json.dumps(plan))
        self.assertTrue(response.success, response.message)
        self.assertEqual(len(response.waypoint_positions), 256)
        plan['missions'][0]['path']['points'].append([2.0, 0.0])
        self.reject(json.dumps(plan))
        self.canary()

    def test_tiny_stripe_spacing_cannot_amplify_unbounded_work(self) -> None:
        plan = fleet_plan()
        plan['missions'][0]['path'] = {
            'type': 'lawnmower',
            'area': {'type': 'polygon', 'vertices': [[0, 0], [4, 0], [4, 4], [0, 4]]},
            'stripe_gap_m': 1e-12,
        }
        self.reject(json.dumps(plan))
        self.canary()
