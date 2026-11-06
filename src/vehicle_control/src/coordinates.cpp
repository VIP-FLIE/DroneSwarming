// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/coordinates.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vehicle_control
{
namespace
{

constexpr double earth_radius_m = 6371000.0;
constexpr double pi = 3.14159265358979323846;
constexpr double degrees_per_radian = 180.0 / pi;
constexpr double radians_per_degree = pi / 180.0;
constexpr double maximum_vehicle_command_float =
  static_cast<double>(std::numeric_limits<float>::max());

}

Position enu_to_ned(const Position & position) noexcept
{
  return {position[1], position[0], -position[2]};
}

Position ned_to_enu(const Position & position) noexcept
{
  return {position[1], position[0], -position[2]};
}

double distance(const Position & first, const Position & second) noexcept
{
  return std::hypot(
    std::hypot(first[0] - second[0], first[1] - second[1]), first[2] - second[2]);
}

std::optional<GlobalPosition> enu_to_global(
  const Position & position_enu, const double reference_latitude_deg,
  const double reference_longitude_deg, const double reference_altitude_amsl_m) noexcept
{
  if (!std::isfinite(position_enu[0]) || !std::isfinite(position_enu[1]) ||
    !std::isfinite(position_enu[2]) || !std::isfinite(reference_latitude_deg) ||
    !std::isfinite(reference_longitude_deg) || !std::isfinite(reference_altitude_amsl_m) ||
    reference_latitude_deg < -90.0 || reference_latitude_deg > 90.0 ||
    reference_longitude_deg < -180.0 || reference_longitude_deg > 180.0)
  {
    return std::nullopt;
  }

  const double north_rad = position_enu[1] / earth_radius_m;
  const double east_rad = position_enu[0] / earth_radius_m;
  const double angular_distance_rad = std::hypot(north_rad, east_rad);
  const double reference_latitude_rad = reference_latitude_deg * radians_per_degree;
  const double reference_longitude_rad = reference_longitude_deg * radians_per_degree;
  double latitude_rad = reference_latitude_rad;
  double longitude_rad = reference_longitude_rad;

  if (angular_distance_rad > 0.0) {
    const double sin_distance = std::sin(angular_distance_rad);
    const double cos_distance = std::cos(angular_distance_rad);
    const double latitude_sine = std::clamp(
      cos_distance * std::sin(reference_latitude_rad) +
      north_rad * sin_distance * std::cos(reference_latitude_rad) / angular_distance_rad,
      -1.0, 1.0);
    latitude_rad = std::asin(latitude_sine);
    longitude_rad += std::atan2(
      east_rad * sin_distance,
      angular_distance_rad * std::cos(reference_latitude_rad) * cos_distance -
      north_rad * std::sin(reference_latitude_rad) * sin_distance);
  }

  double longitude_deg = longitude_rad * degrees_per_radian;
  if (longitude_deg > 180.0) {
    longitude_deg -= 360.0;
  } else if (longitude_deg < -180.0) {
    longitude_deg += 360.0;
  }
  const GlobalPosition global_position{
    latitude_rad * degrees_per_radian,
    longitude_deg,
    reference_altitude_amsl_m + position_enu[2]};
  if (!std::isfinite(global_position.latitude_deg) ||
    !std::isfinite(global_position.longitude_deg) ||
    !std::isfinite(global_position.altitude_amsl_m) ||
    std::abs(global_position.altitude_amsl_m) > maximum_vehicle_command_float ||
    global_position.latitude_deg < -90.0 || global_position.latitude_deg > 90.0)
  {
    return std::nullopt;
  }
  return global_position;
}

}  // namespace vehicle_control
