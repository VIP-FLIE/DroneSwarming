// Copyright 2026 DroneSwarming maintainers.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "vehicle_control/coordinates.hpp"

namespace vehicle_control
{

TEST(Coordinates, ConvertsEastNorthUpAxesToNorthEastDown)
{
  EXPECT_EQ((Position{0.0, 3.0, 0.0}), enu_to_ned({3.0, 0.0, 0.0}));
  EXPECT_EQ((Position{3.0, 0.0, 0.0}), enu_to_ned({0.0, 3.0, 0.0}));
  EXPECT_EQ((Position{0.0, 0.0, -2.0}), enu_to_ned({0.0, 0.0, 2.0}));
}

TEST(Coordinates, ConvertsNorthEastDownAxesToEastNorthUp)
{
  EXPECT_EQ((Position{3.0, 0.0, 0.0}), ned_to_enu({0.0, 3.0, 0.0}));
  EXPECT_EQ((Position{0.0, 3.0, 0.0}), ned_to_enu({3.0, 0.0, 0.0}));
  EXPECT_EQ((Position{0.0, 0.0, 2.0}), ned_to_enu({0.0, 0.0, -2.0}));
}

TEST(Coordinates, ComputesThreeDimensionalDistance)
{
  EXPECT_DOUBLE_EQ(5.0, distance({0.0, 0.0, 0.0}, {3.0, 4.0, 0.0}));
  EXPECT_DOUBLE_EQ(3.0, distance({1.0, 2.0, 3.0}, {1.0, 2.0, 0.0}));
}

TEST(Coordinates, ConvertsLocalOriginToGlobalReference)
{
  const std::optional<GlobalPosition> global = enu_to_global(
    {0.0, 0.0, 0.0}, 42.0, -71.0, 18.5);

  ASSERT_TRUE(global.has_value());
  EXPECT_NEAR(42.0, global->latitude_deg, 1.0e-12);
  EXPECT_NEAR(-71.0, global->longitude_deg, 1.0e-12);
  EXPECT_NEAR(18.5, global->altitude_amsl_m, 1.0e-12);
}

TEST(Coordinates, ConvertsEastAndNorthUsingPX4Projection)
{
  constexpr double earth_radius_m = 6371000.0;
  constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
  const std::optional<GlobalPosition> east = enu_to_global(
    {100.0, 0.0, 0.0}, 0.0, 0.0, 0.0);
  const std::optional<GlobalPosition> north = enu_to_global(
    {0.0, 100.0, 0.0}, 0.0, 0.0, 0.0);

  ASSERT_TRUE(east.has_value());
  ASSERT_TRUE(north.has_value());
  EXPECT_NEAR(0.0, east->latitude_deg, 1.0e-12);
  EXPECT_NEAR(100.0 / earth_radius_m * radians_to_degrees, east->longitude_deg, 1.0e-12);
  EXPECT_NEAR(100.0 / earth_radius_m * radians_to_degrees, north->latitude_deg, 1.0e-12);
  EXPECT_NEAR(0.0, north->longitude_deg, 1.0e-12);
}

TEST(Coordinates, WrapsLongitudeAtDateLine)
{
  const std::optional<GlobalPosition> global = enu_to_global(
    {500.0, 0.0, 0.0}, 0.0, 179.999, 0.0);

  ASSERT_TRUE(global.has_value());
  EXPECT_GE(global->longitude_deg, -180.0);
  EXPECT_LT(global->longitude_deg, -179.99);
}

TEST(Coordinates, ConvertsEastOffsetAtHighLatitude)
{
  constexpr double earth_radius_m = 6371000.0;
  constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
  const std::optional<GlobalPosition> global = enu_to_global(
    {100.0, 0.0, 0.0}, 60.0, 10.0, 0.0);

  ASSERT_TRUE(global.has_value());
  EXPECT_NEAR(60.0, global->latitude_deg, 2.0e-8);
  EXPECT_NEAR(100.0 / earth_radius_m / std::cos(3.14159265358979323846 / 3.0) *
    radians_to_degrees, global->longitude_deg - 10.0, 1.0e-8);
}

TEST(Coordinates, RejectsInvalidReferencesAndUnrepresentableAltitude)
{
  EXPECT_FALSE(enu_to_global({0.0, 0.0, 0.0}, 91.0, 0.0, 0.0).has_value());
  EXPECT_FALSE(enu_to_global({0.0, 0.0, 0.0}, 0.0, 181.0, 0.0).has_value());
  EXPECT_FALSE(enu_to_global(
      {0.0, 0.0, 0.0}, 0.0, 0.0, std::numeric_limits<double>::infinity()).has_value());
  EXPECT_FALSE(enu_to_global(
      {0.0, 0.0, 0.0}, 0.0, 0.0,
      static_cast<double>(std::numeric_limits<float>::max()) * 2.0).has_value());
  EXPECT_FALSE(enu_to_global(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 0.0, 0.0, 0.0).has_value());
}

}  // namespace vehicle_control
