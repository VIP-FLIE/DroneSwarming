# Copyright 2026 DroneSwarming maintainers.
"""Map geographic coordinates through the spherical projection used by the PX4 adapter."""

import math
from dataclasses import dataclass

EARTH_RADIUS_M: float = 6_371_000.0
PROJECTION_TOLERANCE_RAD: float = 1e-12


@dataclass(frozen=True)
class GeographicOrigin:
    """Describe a WGS84 latitude/longitude and an altitude in meters AMSL."""

    latitude_deg: float
    longitude_deg: float
    altitude_amsl_m: float

    def __post_init__(self) -> None:
        """Reject non-finite coordinates and out-of-range latitude or longitude."""
        if not (
            math.isfinite(self.latitude_deg)
            and -90.0 <= self.latitude_deg <= 90.0
            and math.isfinite(self.longitude_deg)
            and -180.0 <= self.longitude_deg <= 180.0
            and math.isfinite(self.altitude_amsl_m)
        ):
            raise ValueError('invalid WGS84 reference or altitude AMSL')


def geographic_to_enu(
    position: GeographicOrigin, reference: GeographicOrigin
) -> tuple[float, float, float]:
    """Project a WGS84 position into the reference's east/north/up frame."""
    latitude_rad: float = math.radians(position.latitude_deg)
    reference_latitude_rad: float = math.radians(reference.latitude_deg)
    longitude_delta_rad: float = math.radians(
        math.remainder(position.longitude_deg - reference.longitude_deg, 360.0)
    )
    cosine_distance: float = math.sin(reference_latitude_rad) * math.sin(latitude_rad) + math.cos(
        reference_latitude_rad
    ) * math.cos(latitude_rad) * math.cos(longitude_delta_rad)
    # atan2 preserves small displacements that acos loses close to the reference.
    east_component: float = math.cos(latitude_rad) * math.sin(longitude_delta_rad)
    north_component: float = math.cos(reference_latitude_rad) * math.sin(latitude_rad) - math.sin(
        reference_latitude_rad
    ) * math.cos(latitude_rad) * math.cos(longitude_delta_rad)
    sine_distance: float = math.hypot(east_component, north_component)
    distance_rad: float = math.atan2(sine_distance, cosine_distance)
    if sine_distance < PROJECTION_TOLERANCE_RAD and cosine_distance < 0.0:
        raise ValueError('antipodal coordinates have no unique local projection')
    scale: float = distance_rad / sine_distance if sine_distance > 0.0 else 1.0
    return (
        EARTH_RADIUS_M * scale * east_component,
        EARTH_RADIUS_M * scale * north_component,
        position.altitude_amsl_m - reference.altitude_amsl_m,
    )


def enu_to_geographic(
    east_m: float, north_m: float, up_m: float, reference: GeographicOrigin
) -> GeographicOrigin:
    """Invert the local spherical projection without changing the AMSL altitude datum."""
    if not all(math.isfinite(value) for value in (east_m, north_m, up_m)):
        raise ValueError('ENU coordinates must be finite')
    north_rad: float = north_m / EARTH_RADIUS_M
    east_rad: float = east_m / EARTH_RADIUS_M
    distance_rad: float = math.hypot(north_rad, east_rad)
    latitude_rad: float = math.radians(reference.latitude_deg)
    longitude_rad: float = math.radians(reference.longitude_deg)
    if distance_rad > 0.0:
        sine_distance: float = math.sin(distance_rad)
        cosine_distance: float = math.cos(distance_rad)
        latitude_sine: float = (
            cosine_distance * math.sin(latitude_rad)
            + north_rad * sine_distance * math.cos(latitude_rad) / distance_rad
        )
        longitude_rad += math.atan2(
            east_rad * sine_distance,
            distance_rad * math.cos(latitude_rad) * cosine_distance
            - north_rad * math.sin(latitude_rad) * sine_distance,
        )
        latitude_rad = math.asin(min(1.0, max(-1.0, latitude_sine)))
    return GeographicOrigin(
        math.degrees(latitude_rad),
        math.remainder(math.degrees(longitude_rad), 360.0),
        reference.altitude_amsl_m + up_m,
    )
