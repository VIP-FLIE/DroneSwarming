// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <string>
#include <vector>

#include <swarm_interfaces/msg/trajectory_point.hpp>

#include "vehicle_control/coordinates.hpp"

namespace vehicle_control
{

/// Interpolated ENU position and yaw at one trajectory time.
struct TrajectorySample
{
  Position position_enu{};
  double yaw_enu_rad{0.0};
};

/// Checks trajectory timing, bounds, speed, acceleration, and requested yaw rate.
bool validate_trajectory(
  const std::vector<swarm_interfaces::msg::TrajectoryPoint> & points,
  bool control_yaw, std::string & failure_message);

/// Interpolates position and shortest-path yaw at elapsed seconds.
TrajectorySample interpolate_trajectory(
  const std::vector<swarm_interfaces::msg::TrajectoryPoint> & points,
  double elapsed_sec);

}  // namespace vehicle_control
