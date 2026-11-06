// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include "vehicle_control/coordinates.hpp"

namespace vehicle_control
{

/// Progress measured from fresh positions while the vehicle is in the orbit annulus.
struct OrbitProgressSample
{
  bool in_orbit_band{false};
  double radial_error_m{0.0};
  double altitude_error_m{0.0};
  double angular_step_rad{0.0};
  double completed_revolutions{0.0};
};

/// Unwraps signed angular travel around a fixed local ENU center.
class OrbitProgress
{
public:
  /// Initializes the orbit center, commanded radius, direction, and tolerances.
  OrbitProgress(
    Position center_enu, double radius_m, bool clockwise,
    double radial_tolerance_m, double altitude_tolerance_m);

  /// Updates progress from one new local ENU vehicle position.
  OrbitProgressSample update(const Position & position_enu) noexcept;

private:
  Position center_enu_;
  double radius_m_;
  double direction_sign_;
  double radial_tolerance_m_;
  double altitude_tolerance_m_;
  double accumulated_angle_rad_{0.0};
  double previous_angle_rad_{0.0};
  bool has_previous_angle_{false};
};

}  // namespace vehicle_control
