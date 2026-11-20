// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <array>
#include <optional>
#include <swarm_interfaces/msg/geographic_position.hpp>

namespace swarm_geometry
{
using Position = std::array<double, 3>;
using GeographicPosition = swarm_interfaces::msg::GeographicPosition;
// Bound floating-point projection error without changing authored metric limits.
constexpr double projection_tolerance_m = 0.001;

/// Geographic coordinates with altitude in meters AMSL.
struct GlobalPosition
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_amsl_m{0.0};
};

/// Checks coordinate ranges and representability at the PX4 altitude boundary.
bool is_valid(const GlobalPosition & position) noexcept;
/// Copies a geographic ROS message into a calculation value.
GlobalPosition from_message(const GeographicPosition & position) noexcept;
/// Copies a geographic calculation value into a ROS message.
GeographicPosition to_message(const GlobalPosition & position);
/// Inverts PX4's azimuthal equidistant projection; ENU components are meters.
std::optional<GlobalPosition> enu_to_global(
  const Position & position, double latitude_deg, double longitude_deg,
  double altitude_amsl_m) noexcept;
/// Projects a geographic target into the reference's ENU frame in meters.
std::optional<Position> global_to_enu(
  const GlobalPosition & position, const GlobalPosition & reference) noexcept;
/// Rotates forward/left/up offsets into ENU using counterclockwise yaw from east.
Position body_to_enu(const Position & offset, double yaw_rad) noexcept;
}  // namespace swarm_geometry
