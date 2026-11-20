"""Verify geographic axes, poles, date-line wrapping, and projection failure cases."""

import math

import pytest
from swarm_geometry.geography import (
    EARTH_RADIUS_M,
    GeographicOrigin,
    enu_to_geographic,
    geographic_to_enu,
)


@pytest.mark.parametrize(
    'latitude_deg,longitude_deg',
    [(0, 0), (60, 10), (89.9, 120), (0, 179.999), (-60, -179.999)],
)
def test_projection_preserves_meter_offsets_and_amsl_height(
    latitude_deg: float, longitude_deg: float
) -> None:
    reference = GeographicOrigin(latitude_deg, longitude_deg, 500)
    target = enu_to_geographic(20, 30, 5, reference)
    assert geographic_to_enu(target, reference) == pytest.approx((20, 30, 5), abs=1e-7)
    assert -180 <= target.longitude_deg <= 180
    assert target.altitude_amsl_m == 505


def test_known_equatorial_distance() -> None:
    target = GeographicOrigin(1, 0, 103)
    assert geographic_to_enu(target, GeographicOrigin(0, 0, 100)) == pytest.approx(
        (0, EARTH_RADIUS_M * math.pi / 180, 3)
    )


def test_antipodal_position_is_rejected() -> None:
    with pytest.raises(ValueError, match='antipodal'):
        geographic_to_enu(GeographicOrigin(0, 180, 0), GeographicOrigin(0, 0, 0))


def test_large_offsets_cannot_wrap_into_a_nearby_destination() -> None:
    with pytest.raises(ValueError, match='unique local projection'):
        enu_to_geographic(40030176, 0, 0, GeographicOrigin(0, 0, 0))
