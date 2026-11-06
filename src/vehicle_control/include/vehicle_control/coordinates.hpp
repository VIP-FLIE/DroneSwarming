// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <array>
#include <optional>

namespace vehicle_control
{

/// Application position ordered as east, north, and up in meters.
using Position = std::array<double, 3>;

/// Geographic center and altitude expected by PX4's orbit vehicle command.
struct GlobalPosition
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_amsl_m{0.0};
};

/// Converts a local ENU position to the PX4 NED frame.
Position enu_to_ned(const Position & position) noexcept;
/// Converts a PX4 NED position to the application local ENU frame.
Position ned_to_enu(const Position & position) noexcept;
/// Returns the three-dimensional distance between two positions in meters.
double distance(const Position & first, const Position & second) noexcept;
/// Converts local ENU meters to WGS84 using PX4's v1.17 azimuthal projection.
std::optional<GlobalPosition> enu_to_global(
  const Position & position_enu, double reference_latitude_deg,
  double reference_longitude_deg, double reference_altitude_amsl_m) noexcept;

}  // namespace vehicle_control
