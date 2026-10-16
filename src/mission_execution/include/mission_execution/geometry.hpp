// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <geometry_msgs/msg/point.hpp>

namespace mission_execution
{

/// Adds a mission-relative waypoint offset to a local ENU origin.
geometry_msgs::msg::Point add_waypoint_offset(
  const geometry_msgs::msg::Point & origin, const geometry_msgs::msg::Point & offset);

}  // namespace mission_execution
