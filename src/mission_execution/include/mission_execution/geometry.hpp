// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <swarm_interfaces/action/execute_waypoint_mission.hpp>
#include <swarm_interfaces/action/navigate_to.hpp>
#include <swarm_interfaces/msg/vehicle_state.hpp>

namespace mission_execution
{

/// Resolves relative navigation once using a captured geographic position and optional yaw.
swarm_interfaces::action::NavigateTo::Goal resolve_navigation(
  const swarm_interfaces::action::NavigateTo::Goal & goal,
  const swarm_interfaces::msg::VehicleState & start);
/// Compiles public mission input into fixed geographic destinations and clears offsets.
swarm_interfaces::action::ExecuteWaypointMission::Goal resolve_mission(
  const swarm_interfaces::action::ExecuteWaypointMission::Goal & goal,
  const swarm_interfaces::msg::VehicleState & start);
/// Projects a geographic position into the current estimator frame for a local operation.
geometry_msgs::msg::Point local_position(
  const swarm_interfaces::msg::GeographicPosition & position,
  const swarm_interfaces::msg::VehicleState & state);

}  // namespace mission_execution
