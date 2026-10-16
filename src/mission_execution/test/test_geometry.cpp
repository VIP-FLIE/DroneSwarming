// Copyright 2026 DroneSwarming maintainers.
#include <gtest/gtest.h>

#include "mission_execution/geometry.hpp"

TEST(MissionGeometry, AppliesOffsetToMissionOrigin)
{
  geometry_msgs::msg::Point origin;
  origin.x = 10.0;
  origin.y = -4.0;
  origin.z = 2.0;
  geometry_msgs::msg::Point offset;
  offset.x = 3.0;
  offset.y = 0.0;
  offset.z = 2.0;

  const geometry_msgs::msg::Point target =
    mission_execution::add_waypoint_offset(origin, offset);

  EXPECT_DOUBLE_EQ(target.x, 13.0);
  EXPECT_DOUBLE_EQ(target.y, -4.0);
  EXPECT_DOUBLE_EQ(target.z, 4.0);
}

TEST(MissionGeometry, DoesNotChangeTheOrigin)
{
  geometry_msgs::msg::Point origin;
  origin.x = 1.0;
  origin.y = 2.0;
  origin.z = 3.0;
  const geometry_msgs::msg::Point offset{};

  (void)mission_execution::add_waypoint_offset(origin, offset);

  EXPECT_DOUBLE_EQ(origin.x, 1.0);
  EXPECT_DOUBLE_EQ(origin.y, 2.0);
  EXPECT_DOUBLE_EQ(origin.z, 3.0);
}
