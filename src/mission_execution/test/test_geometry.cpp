// Copyright 2026 DroneSwarming maintainers.
#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>
#include <swarm_geometry/geography.hpp>
#include "mission_execution/geometry.hpp"

namespace
{
swarm_interfaces::msg::VehicleState starting_state()
{
  swarm_interfaces::msg::VehicleState state;
  state.global_position_valid = true;
  state.global_reference_valid = true;
  state.yaw_valid = true;
  state.yaw_rad = static_cast<float>(std::acos(-1.0) / 2.0);
  state.geographic_position = swarm_geometry::to_message({47.0, 8.0, 500.0});
  state.reference_latitude_deg = 47.0;
  state.reference_longitude_deg = 8.0;
  state.reference_altitude_amsl_m = 500.0;
  return state;
}
}

TEST(MissionGeometry, ForwardUsesCapturedHeadingAndLeftUsesItsPerpendicular)
{
  const swarm_interfaces::msg::VehicleState state = starting_state();
  swarm_interfaces::action::NavigateTo::Goal goal;
  goal.frame_id = "relative_body";
  goal.target_position.x = 3.0;
  goal.target_position.y = 2.0;
  goal.target_position.z = 1.0;
  const swarm_interfaces::action::NavigateTo::Goal resolved = mission_execution::resolve_navigation(goal, state);
  const geometry_msgs::msg::Point target = mission_execution::local_position(resolved.geographic_target, state);
  EXPECT_EQ(resolved.frame_id, "wgs84");
  EXPECT_NEAR(target.x, -2.0, 1.0e-6);
  EXPECT_NEAR(target.y, 3.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(target.z, 1.0);
}

TEST(MissionGeometry, EastNorthOffsetsIgnoreVehicleHeading)
{
  swarm_interfaces::action::ExecuteWaypointMission::Goal goal;
  goal.frame_id = "relative_enu";
  geometry_msgs::msg::Point point;
  point.x = 3.0;
  point.z = 2.0;
  goal.waypoint_offsets.push_back(point);
  point.y = 4.0;
  goal.waypoint_offsets.push_back(point);
  const swarm_interfaces::action::ExecuteWaypointMission::Goal resolved =
    mission_execution::resolve_mission(goal, starting_state());
  ASSERT_EQ(resolved.waypoint_positions.size(), 2U);
  EXPECT_TRUE(resolved.waypoint_offsets.empty());
  const geometry_msgs::msg::Point target = mission_execution::local_position(
    resolved.waypoint_positions[1], starting_state());
  EXPECT_NEAR(target.x, 3.0, 1.0e-8);
  EXPECT_NEAR(target.y, 4.0, 1.0e-8);
  EXPECT_DOUBLE_EQ(target.z, 2.0);
}

TEST(MissionGeometry, GeographicTargetsArePreservedExactlyAcrossOriginChanges)
{
  swarm_interfaces::action::ExecuteWaypointMission::Goal goal;
  goal.frame_id = "wgs84";
  goal.waypoint_positions.push_back(swarm_geometry::to_message({47.000123456789, 8.000234567891, 507.0}));
  swarm_interfaces::msg::VehicleState state = starting_state();
  state.reference_longitude_deg += 0.001;
  ++state.local_frame_revision;
  const swarm_interfaces::action::ExecuteWaypointMission::Goal resolved =
    mission_execution::resolve_mission(goal, state);
  EXPECT_EQ(resolved.waypoint_positions, goal.waypoint_positions);
}

TEST(MissionGeometry, RelativeCommandsRejectMissingPositionOrHeading)
{
  swarm_interfaces::msg::VehicleState state = starting_state();
  swarm_interfaces::action::NavigateTo::Goal goal;
  goal.frame_id = "relative_body";
  state.yaw_valid = false;
  EXPECT_THROW(mission_execution::resolve_navigation(goal, state), std::invalid_argument);
  state.yaw_valid = true;
  state.global_position_valid = false;
  EXPECT_THROW(mission_execution::resolve_navigation(goal, state), std::invalid_argument);
}

TEST(MissionGeometry, RejectsMixedGeographicAndRelativeMissionFields)
{
  swarm_interfaces::action::ExecuteWaypointMission::Goal goal;
  goal.frame_id = "wgs84";
  goal.waypoint_offsets.resize(1);
  EXPECT_THROW(mission_execution::resolve_mission(goal, starting_state()), std::invalid_argument);
  goal.frame_id = "relative_enu";
  goal.waypoint_positions.resize(1);
  EXPECT_THROW(mission_execution::resolve_mission(goal, starting_state()), std::invalid_argument);
}

TEST(MissionGeometry, RejectsOversizedRelativeOffsetsBeforeGeographicWrapping)
{
  swarm_interfaces::action::NavigateTo::Goal goal;
  goal.frame_id = "relative_enu";
  goal.target_position.x = 40030176.0;
  EXPECT_THROW(mission_execution::resolve_navigation(goal, starting_state()), std::invalid_argument);
}
