// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/coordinates.hpp"

#include <cmath>

namespace vehicle_control
{

Position enu_to_ned(const Position & position) noexcept
{
  return {position[1], position[0], -position[2]};
}

Position ned_to_enu(const Position & position) noexcept
{
  return {position[1], position[0], -position[2]};
}

double distance(const Position & first, const Position & second) noexcept
{
  return std::hypot(
    std::hypot(first[0] - second[0], first[1] - second[1]), first[2] - second[2]);
}

}  // namespace vehicle_control
