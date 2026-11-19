// Copyright 2026 DroneSwarming maintainers.
#include "mission_execution/geometry.hpp"

#include <cmath>
#include <stdexcept>
#include <swarm_geometry/geography.hpp>

namespace mission_execution
{
namespace
{
/// Resolves one relative offset using the same command-start anchor for every point.
swarm_interfaces::msg::GeographicPosition resolve_offset(
  const geometry_msgs::msg::Point & point, const std::string & frame,
  const swarm_interfaces::msg::VehicleState & start)
{
  if (!start.global_position_valid) {
    throw std::invalid_argument("Relative commands require valid geographic position");
  }
  swarm_geometry::Position offset{point.x, point.y, point.z};
  constexpr double maximum_relative_distance_m = 1000.0;
  constexpr double maximum_relative_height_m = 120.0;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
    std::hypot(point.x, point.y) > maximum_relative_distance_m ||
    std::abs(point.z) > maximum_relative_height_m)
  {
    throw std::invalid_argument("Relative command exceeds the 1000 m horizontal or 120 m vertical bound");
  }
  if (frame == "relative_body") {
    if (!start.yaw_valid || !std::isfinite(start.yaw_rad)) {
      throw std::invalid_argument("relative_body requires valid command-start yaw");
    }
    offset = swarm_geometry::body_to_enu(offset, static_cast<double>(start.yaw_rad));
  } else if (!frame.empty() && frame != "relative_enu") {
    throw std::invalid_argument("Unsupported relative coordinate frame");
  }
  const std::optional<swarm_geometry::GlobalPosition> global = swarm_geometry::enu_to_global(
    offset, start.geographic_position.latitude_deg, start.geographic_position.longitude_deg,
    start.geographic_position.altitude_amsl_m);
  if (!global) {
    throw std::invalid_argument("Relative destination or geographic anchor is invalid");
  }
  return swarm_geometry::to_message(*global);
}
}  // namespace

swarm_interfaces::action::NavigateTo::Goal resolve_navigation(
  const swarm_interfaces::action::NavigateTo::Goal & goal,
  const swarm_interfaces::msg::VehicleState & start)
{
  if (goal.frame_id == "wgs84" && !start.global_position_valid) {
    throw std::invalid_argument("Geographic navigation requires valid geographic vehicle position");
  }
  swarm_interfaces::action::NavigateTo::Goal resolved = goal;
  if (goal.frame_id == "relative_enu" || goal.frame_id == "relative_body") {
    resolved.geographic_target = resolve_offset(goal.target_position, goal.frame_id, start);
    if (goal.look_at_target) {
      resolved.look_at_geographic_position = resolve_offset(goal.look_at_position, goal.frame_id, start);
    }
    resolved.frame_id = "wgs84";
    resolved.target_position = geometry_msgs::msg::Point();
    resolved.look_at_position = geometry_msgs::msg::Point();
  }
  return resolved;
}

swarm_interfaces::action::ExecuteWaypointMission::Goal resolve_mission(
  const swarm_interfaces::action::ExecuteWaypointMission::Goal & goal,
  const swarm_interfaces::msg::VehicleState & start)
{
  swarm_interfaces::action::ExecuteWaypointMission::Goal resolved = goal;
  if (!start.global_position_valid) {
    throw std::invalid_argument("Mission requires valid geographic position");
  }
  if (goal.frame_id == "wgs84") {
    if (!goal.waypoint_offsets.empty()) {
      throw std::invalid_argument("wgs84 missions do not accept relative waypoint offsets");
    }
    return resolved;
  }
  if (!goal.waypoint_positions.empty()) {
    throw std::invalid_argument("Relative missions do not accept geographic waypoints");
  }
  for (const geometry_msgs::msg::Point & point : goal.waypoint_offsets) {
    resolved.waypoint_positions.push_back(resolve_offset(point, goal.frame_id, start));
  }
  if (goal.look_at_point_of_interest) {
    resolved.look_at_geographic_position = resolve_offset(goal.look_at_offset, goal.frame_id, start);
  }
  if (goal.orbit_after_waypoints) {
    resolved.orbit_center_geographic = resolve_offset(goal.orbit_center_offset, goal.frame_id, start);
  }
  resolved.frame_id = "wgs84";
  resolved.waypoint_offsets.clear();
  resolved.look_at_offset = geometry_msgs::msg::Point();
  resolved.orbit_center_offset = geometry_msgs::msg::Point();
  return resolved;
}

geometry_msgs::msg::Point local_position(
  const swarm_interfaces::msg::GeographicPosition & position,
  const swarm_interfaces::msg::VehicleState & state)
{
  const std::optional<swarm_geometry::Position> local = swarm_geometry::global_to_enu(
    swarm_geometry::from_message(position), {state.reference_latitude_deg,
      state.reference_longitude_deg, state.reference_altitude_amsl_m});
  if (!state.global_reference_valid || !local) {
    throw std::invalid_argument("Local operation requires a valid geographic estimator reference");
  }
  geometry_msgs::msg::Point point;
  point.x = (*local)[0];
  point.y = (*local)[1];
  point.z = (*local)[2];
  return point;
}

}  // namespace mission_execution
