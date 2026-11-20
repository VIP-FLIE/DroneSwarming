// Copyright 2026 DroneSwarming maintainers.
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <mavsdk/mavlink_include.hpp>

#include "vehicle_control/onboard_mission.hpp"

namespace vehicle_control
{
namespace
{
VehicleTelemetry reference()
{
  VehicleTelemetry telemetry;
  telemetry.position_valid = true;
  telemetry.xy_global = true;
  telemetry.z_global = true;
  telemetry.reference_latitude_deg = 47.0;
  telemetry.reference_longitude_deg = 8.0;
  telemetry.reference_altitude_amsl_m = 500.0;
  telemetry.position_enu = {10.0, 20.0, 1.0};
  telemetry.local_frame_revision = 7;
  return telemetry;
}

swarm_interfaces::action::FlyMission::Goal route()
{
  swarm_interfaces::action::FlyMission::Goal goal;
  goal.starting_position = swarm_geometry::to_message(*global_position(reference()));
  goal.takeoff_position = goal.starting_position;
  goal.takeoff_position.altitude_amsl_m += 3.0;
  goal.waypoint_timeout_sec = 30.0F;
  goal.position_tolerance_m = 0.5F;
  goal.waypoint_positions.push_back(goal.takeoff_position);
  goal.waypoint_positions.back() = swarm_geometry::to_message(*enu_to_global(
    {3.0, 0.0, 3.0}, goal.starting_position.latitude_deg,
    goal.starting_position.longitude_deg, goal.starting_position.altitude_amsl_m));
  goal.land_after_waypoints = true;
  return goal;
}
}

TEST(OnboardPlan, EncodesGlobalTakeoffWaypointsAndLandingWithAmslAltitude)
{
  const OnboardPlan plan = build_onboard_plan(route(), reference(), 0.5);
  ASSERT_EQ(plan.items.size(), 3U);
  EXPECT_EQ(plan.items[0].command, MAV_CMD_NAV_TAKEOFF);
  EXPECT_EQ(plan.items[1].command, MAV_CMD_NAV_WAYPOINT);
  EXPECT_EQ(plan.items[2].command, MAV_CMD_NAV_LAND);
  EXPECT_EQ(plan.waypoint_indexes, std::vector<std::int32_t>({1}));
  for (std::size_t index = 0; index < plan.items.size(); ++index) {
    EXPECT_EQ(plan.items[index].seq, index);
    EXPECT_EQ(plan.items[index].frame, MAV_FRAME_GLOBAL);
    EXPECT_FLOAT_EQ(plan.items[index].z, 504.0F);
    EXPECT_EQ(plan.items[index].current, index == 0 ? 1U : 0U);
    EXPECT_EQ(plan.items[index].autocontinue, 1U);
  }
  const std::optional<GlobalPosition> expected = std::optional<GlobalPosition>(swarm_geometry::from_message(route().waypoint_positions[0]));
  ASSERT_TRUE(expected.has_value());
  EXPECT_NEAR(static_cast<double>(plan.items[1].x) / 1.0e7, expected->latitude_deg, 1.0e-7);
  EXPECT_NEAR(static_cast<double>(plan.items[1].y) / 1.0e7, expected->longitude_deg, 1.0e-7);
  EXPECT_FLOAT_EQ(plan.items[1].param2, 0.5F);
  EXPECT_TRUE(std::isnan(plan.items[1].param4));
}

TEST(OnboardPlan, RoiIsClearedBeforeLandingAndProgressIndexesExcludeRoiItems)
{
  swarm_interfaces::action::FlyMission::Goal goal = route();
  goal.look_at_point_of_interest = true;
  goal.look_at_position = goal.takeoff_position;
  const OnboardPlan plan = build_onboard_plan(goal, reference(), 0.5);
  ASSERT_EQ(plan.items.size(), 5U);
  EXPECT_EQ(plan.items[1].command, MAV_CMD_DO_SET_ROI_LOCATION);
  EXPECT_EQ(plan.items[3].command, MAV_CMD_DO_SET_ROI_NONE);
  EXPECT_EQ(plan.items[4].command, MAV_CMD_NAV_LAND);
  EXPECT_EQ(plan.items[4].x, plan.items[2].x);
  EXPECT_EQ(plan.items[4].y, plan.items[2].y);
  EXPECT_EQ(plan.waypoint_indexes, std::vector<std::int32_t>({2}));
}

TEST(OnboardPlan, TakeoffOnlyRouteSupportsFiniteOrbitHandoffWithoutLanding)
{
  swarm_interfaces::action::FlyMission::Goal goal = route();
  goal.waypoint_positions.clear();
  goal.land_after_waypoints = false;
  const OnboardPlan plan = build_onboard_plan(goal, reference(), 0.5);
  ASSERT_EQ(plan.items.size(), 1U);
  EXPECT_TRUE(plan.waypoint_indexes.empty());
  EXPECT_EQ(plan.items[0].command, MAV_CMD_NAV_TAKEOFF);
}

TEST(OnboardPlan, RejectsMissingGlobalReferenceAndAcceptsLocalRevisionChanges)
{
  VehicleTelemetry telemetry = reference();
  telemetry.xy_global = false;
  EXPECT_THROW(build_onboard_plan(route(), telemetry, 0.5), std::invalid_argument);
  telemetry = reference();
  ++telemetry.local_frame_revision;
  EXPECT_NO_THROW(build_onboard_plan(route(), telemetry, 0.5));
  telemetry = reference();
  telemetry.reference_latitude_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(build_onboard_plan(route(), telemetry, 0.5), std::invalid_argument);
}

TEST(OnboardPlan, RejectsNonfiniteAndOutOfBoundsInputs)
{
  swarm_interfaces::action::FlyMission::Goal goal = route();
  goal.waypoint_positions[0].latitude_deg = std::numeric_limits<double>::infinity();
  EXPECT_THROW(build_onboard_plan(goal, reference(), 0.5), std::invalid_argument);
  goal = route();
  goal.waypoint_positions[0].latitude_deg = 48.0;
  EXPECT_THROW(build_onboard_plan(goal, reference(), 0.5), std::invalid_argument);
  goal = route();
  goal.takeoff_position.altitude_amsl_m = 500.0;
  EXPECT_THROW(build_onboard_plan(goal, reference(), 0.5), std::invalid_argument);
  goal = route();
  goal.waypoint_timeout_sec = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(build_onboard_plan(goal, reference(), 0.5), std::invalid_argument);
  goal = route();
  goal.waypoint_positions.resize(257, goal.takeoff_position);
  EXPECT_THROW(build_onboard_plan(goal, reference(), 0.5), std::invalid_argument);
}

TEST(OnboardPlan, EncodedTargetsRemainIdenticalAfterEstimatorOriginChanges)
{
  const swarm_interfaces::action::FlyMission::Goal goal = route();
  const OnboardPlan original = build_onboard_plan(goal, reference(), 0.5);
  VehicleTelemetry shifted = reference();
  shifted.reference_longitude_deg += 0.001;
  ++shifted.local_frame_revision;
  const std::optional<Position> shifted_local = global_to_enu(
    swarm_geometry::from_message(goal.starting_position),
    {shifted.reference_latitude_deg, shifted.reference_longitude_deg, shifted.reference_altitude_amsl_m});
  ASSERT_TRUE(shifted_local.has_value());
  shifted.position_enu = *shifted_local;
  const OnboardPlan rebuilt = build_onboard_plan(goal, shifted, 0.5);
  ASSERT_EQ(original.items.size(), rebuilt.items.size());
  for (std::size_t index = 0; index < original.items.size(); ++index) {
    EXPECT_EQ(original.items[index].x, rebuilt.items[index].x);
    EXPECT_EQ(original.items[index].y, rebuilt.items[index].y);
    EXPECT_EQ(original.items[index].z, rebuilt.items[index].z);
  }
}

TEST(OnboardPlan, MovementInvalidatesTheCapturedGeographicStart)
{
  VehicleTelemetry moved = reference();
  moved.position_enu[0] += 2.0;
  EXPECT_THROW(build_onboard_plan(route(), moved, 0.5), std::invalid_argument);
}

TEST(OnboardPlan, ProjectedTargetsAtTheExactDistanceLimitRemainAdmissible)
{
  swarm_interfaces::action::FlyMission::Goal goal = route();
  const std::optional<GlobalPosition> boundary = enu_to_global({1000.0, 0.0, 3.0},
    goal.starting_position.latitude_deg, goal.starting_position.longitude_deg,
    goal.starting_position.altitude_amsl_m);
  ASSERT_TRUE(boundary.has_value());
  goal.waypoint_positions[0] = swarm_geometry::to_message(*boundary);
  EXPECT_NO_THROW(build_onboard_plan(goal, reference(), 0.5));
  const std::optional<GlobalPosition> outside = enu_to_global({1000.01, 0.0, 3.0},
    goal.starting_position.latitude_deg, goal.starting_position.longitude_deg,
    goal.starting_position.altitude_amsl_m);
  ASSERT_TRUE(outside.has_value());
  goal.waypoint_positions[0] = swarm_geometry::to_message(*outside);
  EXPECT_THROW(build_onboard_plan(goal, reference(), 0.5), std::invalid_argument);
}

}  // namespace vehicle_control
