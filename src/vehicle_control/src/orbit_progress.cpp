// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/orbit_progress.hpp"

#include <algorithm>
#include <cmath>

namespace vehicle_control
{
namespace
{

constexpr double two_pi = 6.28318530717958647692;

}

OrbitProgress::OrbitProgress(
  Position center_enu, const double radius_m, const bool clockwise,
  const double radial_tolerance_m, const double altitude_tolerance_m)
: center_enu_(center_enu), radius_m_(radius_m), direction_sign_(clockwise ? -1.0 : 1.0),
  radial_tolerance_m_(radial_tolerance_m), altitude_tolerance_m_(altitude_tolerance_m)
{
}

OrbitProgressSample OrbitProgress::update(const Position & position_enu) noexcept
{
  OrbitProgressSample sample{};
  const double east_offset = position_enu[0] - center_enu_[0];
  const double north_offset = position_enu[1] - center_enu_[1];
  const double radius = std::hypot(east_offset, north_offset);
  sample.radial_error_m = std::abs(radius - radius_m_);
  sample.altitude_error_m = std::abs(position_enu[2] - center_enu_[2]);
  sample.in_orbit_band = sample.radial_error_m <= radial_tolerance_m_ &&
    sample.altitude_error_m <= altitude_tolerance_m_;
  if (!sample.in_orbit_band) {
    has_previous_angle_ = false;
    sample.completed_revolutions = std::max(0.0, accumulated_angle_rad_ / two_pi);
    return sample;
  }

  const double angle_rad = std::atan2(north_offset, east_offset);
  if (has_previous_angle_) {
    const double raw_angle_delta = angle_rad - previous_angle_rad_;
    const double wrapped_angle_delta = std::atan2(
      std::sin(raw_angle_delta), std::cos(raw_angle_delta));
    sample.angular_step_rad = direction_sign_ * wrapped_angle_delta;
    accumulated_angle_rad_ += sample.angular_step_rad;
  }
  previous_angle_rad_ = angle_rad;
  has_previous_angle_ = true;
  sample.completed_revolutions = std::max(0.0, accumulated_angle_rad_ / two_pi);
  return sample;
}

}  // namespace vehicle_control
