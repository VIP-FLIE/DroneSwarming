# Copyright 2026 DroneSwarming maintainers.
"""Probe planner callback exception containment, expansion limits, and ROS float narrowing."""

from __future__ import annotations

import json
import math
import random
import time
import unittest

from harness import Harness, JsonValue, resolve, resolve_batch, starting_state

from swarm_interfaces.srv import PlanFlight

MUTATION_SEED = 20261119


def plan_document() -> dict[str, JsonValue]:
    """Return a canonical valid document for mutation at the ROS service boundary."""
    return {
        'version': 2,
        'missions': [
            {
                'vehicle_ids': ['uav_1'],
                'frame_id': 'relative_enu',
                'takeoff_altitude_m': 3.0,
                'waypoint_acceptance_radius_m': 0.5,
                'path': {'type': 'waypoints', 'points': [[1.0, 0.0], [2.0, 0.0]]},
            }
        ],
    }


def request(plan_json: str | None = None) -> PlanFlight.Request:
    """Supply a fixed geographic start with the document under test."""
    return PlanFlight.Request(
        plan_json=json.dumps(plan_document()) if plan_json is None else plan_json,
        vehicle_id='uav_1',
        starting_state=starting_state(),
    )


class PlannerFaults(unittest.TestCase):
    """Stress the real planning service without sending flight actions."""

    def setUp(self) -> None:
        """Launch a fresh planning process for each service-boundary attack."""
        self.harness = Harness()
        self.addCleanup(self.harness.close)
        self.harness.launch('mission_planner')
        self.client = self.harness.node.create_client(
            PlanFlight,
            'mission_planner/plan_flight',
            callback_group=self.harness.group,
        )
        self.assertTrue(self.client.wait_for_service(timeout_sec=8.0))

    def test_valid_plan_positive_control(self) -> None:
        """The service must compile a bounded route into fixed geographic targets."""
        response = resolve(self.client.call_async(request()))
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.takeoff_altitude_m, 3.0)
        self.assertEqual(len(response.waypoint_positions), 2)
        self.assertTrue(
            all(position.altitude_amsl_m == 503.0 for position in response.waypoint_positions)
        )

    def test_positive_tolerance_cannot_underflow_to_default_sentinel(self) -> None:
        """A positive JSON tolerance must survive float32 narrowing or be rejected."""
        plan_json: str = json.dumps(plan_document()).replace(
            '"waypoint_acceptance_radius_m": 0.5',
            '"waypoint_acceptance_radius_m": 1e-50',
        )
        response = resolve(self.client.call_async(request(plan_json)))
        self.assertTrue(
            not response.success or response.position_tolerance_m > 0.0,
            'Planner returned success with a positive tolerance narrowed to zero',
        )

    def test_positive_takeoff_cannot_underflow_to_zero(self) -> None:
        """A successful response cannot contain a zero-meter takeoff after float narrowing."""
        # A shared route keeps its waypoint altitude valid independently of takeoff climb.
        plan_json: str = json.dumps(
            {
                'version': 2,
                'missions': [
                    {
                        'vehicle_ids': ['uav_1'],
                        'frame_id': 'shared_enu',
                        'origin': {
                            'latitude_deg': 47.0,
                            'longitude_deg': 8.0,
                            'altitude_amsl_m': 503.0,
                        },
                        'takeoff_altitude_m': 1e-50,
                        'waypoint_acceptance_radius_m': 0.5,
                        'path': {'type': 'waypoints', 'points': [[1.0, 0.0]]},
                    }
                ],
            }
        )
        response = resolve(self.client.call_async(request(plan_json)))
        self.assertTrue(
            not response.success or response.takeoff_altitude_m > 0.0,
            'Planner returned success with a positive takeoff narrowed to zero',
        )

    def test_parser_poison_corpus_and_recovery(self) -> None:
        """Malformed JSON, deep recursion, duplicate keys, and huge numbers stay contained."""
        valid_json: str = json.dumps(plan_document())
        corpus: list[str] = [
            '',
            '{',
            '[]',
            'null',
            'true',
            '"string"',
            '{"version":2,"version":1}',
            '[' * 1500 + '0' + ']' * 1500,
            '{"version":' + '9' * 5000 + '}',
            valid_json.replace('3.0', 'NaN', 1),
            valid_json.replace('3.0', 'Infinity', 1),
            valid_json.replace('3.0', '1e999', 1),
            valid_json.replace('3.0', 'true', 1),
            valid_json.replace('relative_enu', 'relative_enu\\u0000'),
            ' ' * 65537,
            '"' + '\u2603' * 22000 + '"',
        ]
        generator = random.Random(MUTATION_SEED)
        for _ in range(64):
            cut: int = generator.randrange(1, len(valid_json))
            corpus.append(valid_json[:cut])
        for index, plan_json in enumerate(corpus):
            with self.subTest(index=index, seed=MUTATION_SEED):
                response = resolve(self.client.call_async(request(plan_json)))
                self.assertFalse(response.success, f'Poison input {index} was accepted')
                self.harness.assert_alive()
        self.assertTrue(resolve(self.client.call_async(request())).success)

    def test_nonfinite_snapshot_corpus_and_recovery(self) -> None:
        """Validity flags cannot authorize NaN, infinite, or out-of-range telemetry."""
        for value in (math.nan, math.inf, -math.inf):
            for field in ('local_position', 'latitude', 'longitude', 'altitude'):
                with self.subTest(field=field, value=value):
                    malformed = request()
                    if field == 'local_position':
                        malformed.starting_state.position.x = value
                    elif field == 'latitude':
                        malformed.starting_state.geographic_position.latitude_deg = value
                    elif field == 'longitude':
                        malformed.starting_state.geographic_position.longitude_deg = value
                    else:
                        malformed.starting_state.geographic_position.altitude_amsl_m = value
                    response = resolve(self.client.call_async(malformed))
                    self.assertFalse(response.success)
        self.assertTrue(resolve(self.client.call_async(request())).success)

    def test_expansion_bombs_are_rejected_before_allocation(self) -> None:
        """Subnormal stripe gaps and oversized routes must respect expansion bounds."""
        documents: list[dict[str, JsonValue]] = [
            {
                'version': 1,
                'frame_id': 'local_enu',
                'takeoff_altitude_m': 3.0,
                'waypoint_acceptance_radius_m': 0.5,
                'path': {
                    'type': 'lawnmower',
                    'stripe_gap_m': 5e-324,
                    'area': {'type': 'circle', 'center': [0, 0], 'radius_m': 10},
                },
            },
            {
                'version': 1,
                'frame_id': 'local_enu',
                'takeoff_altitude_m': 3.0,
                'waypoint_acceptance_radius_m': 0.5,
                'path': {'type': 'waypoints', 'points': [[index, 0] for index in range(257)]},
            },
        ]
        for document in documents:
            response = resolve(self.client.call_async(request(json.dumps(document))))
            self.assertFalse(response.success)
        self.harness.assert_alive()
        self.assertTrue(resolve(self.client.call_async(request())).success)

    def test_service_flood_recovers_after_rejections(self) -> None:
        """Queued malformed requests must not starve a subsequent valid request."""
        started_sec: float = time.monotonic()
        futures = [self.client.call_async(request('{}')) for _ in range(128)]
        for response in resolve_batch(futures):
            self.assertFalse(response.success)
        response = resolve(self.client.call_async(request()), 5.0)
        self.assertTrue(response.success)
        self.assertLess(time.monotonic() - started_sec, 10.0)
        self.harness.assert_alive()
