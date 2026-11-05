// Copyright 2026 DroneSwarming maintainers.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "vehicle_control/trajectory.hpp"

namespace vehicle_control
{
namespace
{

using TrajectoryPoint = swarm_interfaces::msg::TrajectoryPoint;

TrajectoryPoint point(
  const double east_m, const double north_m, const double up_m,
  const float time_sec, const float yaw_rad = 0.0F)
{
  TrajectoryPoint trajectory_point{};
  trajectory_point.position.x = east_m;
  trajectory_point.position.y = north_m;
  trajectory_point.position.z = up_m;
  trajectory_point.time_from_start_sec = time_sec;
  trajectory_point.yaw_rad = yaw_rad;
  return trajectory_point;
}

TEST(Trajectory, AcceptsBoundedSearchLaneSchedule)
{
  const std::vector<TrajectoryPoint> points{
    point(0.0, 0.0, 3.0, 0.0F),
    point(12.0, 0.0, 3.0, 4.0F),
    point(12.0, 0.0, 3.0, 5.0F),
    point(12.0, 4.0, 3.0, 7.0F),
    point(12.0, 4.0, 3.0, 8.0F),
    point(0.0, 4.0, 3.0, 12.0F),
    point(0.0, 4.0, 3.0, 13.0F),
    point(0.0, 8.0, 3.0, 15.0F),
    point(0.0, 8.0, 3.0, 16.0F),
    point(12.0, 8.0, 3.0, 20.0F),
    point(12.0, 8.0, 3.0, 22.0F),
  };
  std::string failure_message;

  EXPECT_TRUE(validate_trajectory(points, failure_message)) << failure_message;
  EXPECT_TRUE(failure_message.empty());
}

TEST(Trajectory, RejectsShortUnorderedAndNonzeroStartTimes)
{
  std::string failure_message;
  EXPECT_FALSE(validate_trajectory({point(0.0, 0.0, 0.0, 0.0F)}, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("at least two"));

  EXPECT_FALSE(validate_trajectory(
    {point(0.0, 0.0, 0.0, 0.0F), point(1.0, 0.0, 0.0, 0.05F)}, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("at least 0.1"));

  EXPECT_FALSE(validate_trajectory(
    {point(0.0, 0.0, 0.0, 1.0F), point(1.0, 0.0, 0.0, 2.0F)}, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("time zero"));
}

TEST(Trajectory, RejectsUnsafeEnvelopeAndSegmentSpeed)
{
  std::string failure_message;
  EXPECT_FALSE(validate_trajectory(
    {point(0.0, 0.0, 0.0, 0.0F), point(21.0, 0.0, 0.0, 10.0F)}, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("envelope"));

  EXPECT_FALSE(validate_trajectory(
    {point(0.0, 0.0, 0.0, 0.0F), point(5.0, 0.0, 0.0, 1.0F)}, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("segment-speed"));
}

TEST(Trajectory, RejectsExcessiveAccelerationAndYawRate)
{
  std::string failure_message;
  EXPECT_FALSE(validate_trajectory(
    {
      point(0.0, 0.0, 0.0, 0.0F),
      point(3.5, 0.0, 0.0, 1.0F),
      point(3.5, 0.0, 0.0, 2.0F),
      point(3.5, 0.0, 0.0, 3.0F),
    }, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("acceleration"));

  EXPECT_FALSE(validate_trajectory(
    {point(0.0, 0.0, 0.0, 0.0F), point(0.0, 0.0, 0.0, 1.0F, 3.0F)}, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("yaw-rate"));
}

TEST(Trajectory, InterpolatesPositionAndShortestYawAcrossWrap)
{
  constexpr float pi = 3.14159265358979323846F;
  const std::vector<TrajectoryPoint> points{
    point(0.0, 0.0, 1.0, 0.0F, pi - 0.1F),
    point(4.0, 2.0, 3.0, 2.0F, -pi + 0.1F),
  };

  const TrajectorySample midpoint = interpolate_trajectory(points, 1.0);
  EXPECT_NEAR(2.0, midpoint.position_enu[0], 1.0e-6);
  EXPECT_NEAR(1.0, midpoint.position_enu[1], 1.0e-6);
  EXPECT_NEAR(2.0, midpoint.position_enu[2], 1.0e-6);
  EXPECT_NEAR(pi, std::abs(midpoint.yaw_enu_rad), 1.0e-5);
  EXPECT_DOUBLE_EQ(0.0, interpolate_trajectory(points, -1.0).position_enu[0]);
  EXPECT_DOUBLE_EQ(4.0, interpolate_trajectory(points, 10.0).position_enu[0]);
}

TEST(Trajectory, RejectsNonFiniteValues)
{
  std::string failure_message;
  EXPECT_FALSE(validate_trajectory(
    {
      point(0.0, 0.0, 0.0, 0.0F),
      point(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0F),
    }, failure_message));
  EXPECT_NE(std::string::npos, failure_message.find("non-finite"));
}

}  // namespace
}  // namespace vehicle_control
