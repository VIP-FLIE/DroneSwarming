// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <array>
#include <swarm_geometry/geography.hpp>

namespace vehicle_control
{

/// Application position ordered as east, north, and up in meters.
using Position = std::array<double, 3>;

using swarm_geometry::GlobalPosition;
using swarm_geometry::enu_to_global;
using swarm_geometry::global_to_enu;

/// Converts a local ENU position to the PX4 NED frame.
Position enu_to_ned(const Position & position) noexcept;
/// Converts a PX4 NED position to the application local ENU frame.
Position ned_to_enu(const Position & position) noexcept;
/// Returns the three-dimensional distance between two positions in meters.
double distance(const Position & first, const Position & second) noexcept;

}  // namespace vehicle_control
