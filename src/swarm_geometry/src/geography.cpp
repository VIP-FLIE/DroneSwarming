// Copyright 2026 DroneSwarming maintainers.
#include "swarm_geometry/geography.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace swarm_geometry
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
  if (!std::isfinite(angular_distance_rad) || angular_distance_rad >= pi) {
    return std::nullopt;
  }
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

bool is_valid(const GlobalPosition & position) noexcept
{
  return std::isfinite(position.latitude_deg) && std::abs(position.latitude_deg) <= 90.0 &&
         std::isfinite(position.longitude_deg) && std::abs(position.longitude_deg) <= 180.0 &&
         std::isfinite(position.altitude_amsl_m) &&
         std::abs(position.altitude_amsl_m) <= maximum_vehicle_command_float;
}

GlobalPosition from_message(const GeographicPosition & position) noexcept
{
  return {position.latitude_deg, position.longitude_deg, position.altitude_amsl_m};
}

GeographicPosition to_message(const GlobalPosition & position)
{
  GeographicPosition message;
  message.latitude_deg = position.latitude_deg;
  message.longitude_deg = position.longitude_deg;
  message.altitude_amsl_m = position.altitude_amsl_m;
  return message;
}

std::optional<Position> global_to_enu(
  const GlobalPosition & position, const GlobalPosition & reference) noexcept
{
  if (!is_valid(position) || !is_valid(reference)) {
    return std::nullopt;
  }
  const double latitude = position.latitude_deg * radians_per_degree;
  const double reference_latitude = reference.latitude_deg * radians_per_degree;
  const double longitude_delta = std::remainder(
    position.longitude_deg - reference.longitude_deg, 360.0) * radians_per_degree;
  const double east = std::cos(latitude) * std::sin(longitude_delta);
  const double north = std::cos(reference_latitude) * std::sin(latitude) -
    std::sin(reference_latitude) * std::cos(latitude) * std::cos(longitude_delta);
  const double cosine_distance = std::sin(reference_latitude) * std::sin(latitude) +
    std::cos(reference_latitude) * std::cos(latitude) * std::cos(longitude_delta);
  const double sine_distance = std::hypot(east, north);
  constexpr double antipodal_tolerance = 1.0e-12;
  if (sine_distance < antipodal_tolerance && cosine_distance < 0.0) {
    return std::nullopt;
  }
  const double angular_distance = std::atan2(sine_distance, cosine_distance);
  const double scale = sine_distance > 0.0 ? angular_distance / sine_distance : 1.0;
  return Position{earth_radius_m * scale * east, earth_radius_m * scale * north,
    position.altitude_amsl_m - reference.altitude_amsl_m};
}

Position body_to_enu(const Position & offset, const double yaw_rad) noexcept
{
  return {std::cos(yaw_rad) * offset[0] - std::sin(yaw_rad) * offset[1],
    std::sin(yaw_rad) * offset[0] + std::cos(yaw_rad) * offset[1], offset[2]};
}

}  // namespace swarm_geometry
