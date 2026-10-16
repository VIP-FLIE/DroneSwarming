// Copyright 2026 DroneSwarming maintainers.
#include "mission_execution/geometry.hpp"

namespace mission_execution
{

geometry_msgs::msg::Point add_waypoint_offset(
  const geometry_msgs::msg::Point & origin, const geometry_msgs::msg::Point & offset)
{
  geometry_msgs::msg::Point target;
  target.x = origin.x + offset.x;
  target.y = origin.y + offset.y;
  target.z = origin.z + offset.z;
  return target;
}

}  // namespace mission_execution
