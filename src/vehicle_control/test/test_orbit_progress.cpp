// Copyright 2026 DroneSwarming maintainers.
#include <cmath>

#include <gtest/gtest.h>

#include "vehicle_control/orbit_progress.hpp"

namespace vehicle_control
{
namespace
{

constexpr double pi = 3.14159265358979323846;
constexpr double radial_tolerance_m = 0.5;
constexpr double altitude_tolerance_m = 0.5;

Position position_at_angle(const double angle_rad, const double radius_m = 5.0)
{
  return {radius_m * std::cos(angle_rad), radius_m * std::sin(angle_rad), 4.0};
}

}  // namespace

TEST(OrbitProgress, CountsClockwiseAcrossAngleWrap)
{
  OrbitProgress progress({0.0, 0.0, 4.0}, 5.0, true, radial_tolerance_m,
    altitude_tolerance_m);
  progress.update(position_at_angle(-pi + 0.1));
  const OrbitProgressSample sample = progress.update(position_at_angle(pi - 0.1));

  EXPECT_TRUE(sample.in_orbit_band);
  EXPECT_NEAR(0.2, sample.angular_step_rad, 1.0e-9);
  EXPECT_NEAR(0.2 / (2.0 * pi), sample.completed_revolutions, 1.0e-9);
}

TEST(OrbitProgress, CountsCounterclockwiseAcrossAngleWrap)
{
  OrbitProgress progress({0.0, 0.0, 4.0}, 5.0, false, radial_tolerance_m,
    altitude_tolerance_m);
  progress.update(position_at_angle(pi - 0.1));
  const OrbitProgressSample sample = progress.update(position_at_angle(-pi + 0.1));

  EXPECT_TRUE(sample.in_orbit_band);
  EXPECT_NEAR(0.2, sample.angular_step_rad, 1.0e-9);
  EXPECT_NEAR(0.2 / (2.0 * pi), sample.completed_revolutions, 1.0e-9);
}

TEST(OrbitProgress, OppositeDirectionTravelDoesNotCountAsProgress)
{
  OrbitProgress progress({0.0, 0.0, 4.0}, 5.0, true, radial_tolerance_m,
    altitude_tolerance_m);
  progress.update(position_at_angle(0.0));
  const OrbitProgressSample sample = progress.update(position_at_angle(0.2));

  EXPECT_LT(sample.angular_step_rad, 0.0);
  EXPECT_DOUBLE_EQ(0.0, sample.completed_revolutions);
}

TEST(OrbitProgress, RebaselinesAfterLeavingOrbitBand)
{
  OrbitProgress progress({0.0, 0.0, 4.0}, 5.0, true, radial_tolerance_m,
    altitude_tolerance_m);
  progress.update(position_at_angle(0.0));
  progress.update(position_at_angle(-pi / 2.0));
  const OrbitProgressSample outside_band = progress.update({0.0, 0.0, 4.0});
  EXPECT_FALSE(outside_band.in_orbit_band);

  progress.update(position_at_angle(pi));
  const OrbitProgressSample next_sample = progress.update(position_at_angle(pi - 0.1));
  EXPECT_NEAR(0.1, next_sample.angular_step_rad, 1.0e-9);
}

TEST(OrbitProgress, RequiresBothRadiusAndAltitudeToBeInBand)
{
  OrbitProgress progress({0.0, 0.0, 4.0}, 5.0, true, radial_tolerance_m,
    altitude_tolerance_m);
  const OrbitProgressSample sample = progress.update({5.0, 0.0, 5.0});

  EXPECT_FALSE(sample.in_orbit_band);
  EXPECT_DOUBLE_EQ(0.0, sample.completed_revolutions);
}

}  // namespace vehicle_control
