// Copyright 2026 DroneSwarming maintainers.
#include <gtest/gtest.h>

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

}  // namespace vehicle_control
