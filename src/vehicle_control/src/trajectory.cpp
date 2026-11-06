// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/trajectory.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <stdexcept>

namespace vehicle_control
{
namespace
{

constexpr std::size_t maximum_trajectory_points = 512;
constexpr double maximum_trajectory_duration_sec = 180.0;
constexpr double minimum_segment_duration_sec = 0.1;
constexpr double maximum_horizontal_extent_m = 20.0;
constexpr double maximum_vertical_extent_m = 8.0;
constexpr double maximum_segment_speed_mps = 4.0;
constexpr double maximum_acceleration_mps2 = 3.0;
constexpr double maximum_yaw_rate_radps = 2.0;

double wrap_angle(const double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

Position point_position(const swarm_interfaces::msg::TrajectoryPoint & point)
{
  return {point.position.x, point.position.y, point.position.z};
}

Position segment_velocity(
  const swarm_interfaces::msg::TrajectoryPoint & start,
  const swarm_interfaces::msg::TrajectoryPoint & end)
{
  const double duration_sec = static_cast<double>(end.time_from_start_sec) -
    static_cast<double>(start.time_from_start_sec);
  return {
    (end.position.x - start.position.x) / duration_sec,
    (end.position.y - start.position.y) / duration_sec,
    (end.position.z - start.position.z) / duration_sec};
}

}  // namespace

bool validate_trajectory(
  const std::vector<swarm_interfaces::msg::TrajectoryPoint> & points,
  const bool control_yaw, std::string & failure_message)
{
  if (points.size() < 2) {
    failure_message = "trajectory requires at least two points";
    return false;
  }
  if (points.size() > maximum_trajectory_points) {
    failure_message = "trajectory exceeds the maximum point count";
    return false;
  }
  if (!std::isfinite(points.front().time_from_start_sec) ||
    std::abs(static_cast<double>(points.front().time_from_start_sec)) > 1.0e-5)
  {
    failure_message = "trajectory must start at time zero";
    return false;
  }

  const Position start_position = point_position(points.front());
  if (!std::isfinite(start_position[0]) || !std::isfinite(start_position[1]) ||
    !std::isfinite(start_position[2]) ||
    (control_yaw && !std::isfinite(points.front().yaw_rad)))
  {
    failure_message = "trajectory contains a non-finite first point";
    return false;
  }
  if (!std::isfinite(points.back().time_from_start_sec) ||
    static_cast<double>(points.back().time_from_start_sec) <= 0.0 ||
    static_cast<double>(points.back().time_from_start_sec) > maximum_trajectory_duration_sec)
  {
    failure_message = "trajectory duration must be positive and at most 180 seconds";
    return false;
  }

  std::vector<Position> velocities;
  velocities.reserve(points.size() - 1);
  for (std::size_t index = 1; index < points.size(); ++index) {
    const auto & previous = points[index - 1];
    const auto & point = points[index];
    const double previous_time = static_cast<double>(previous.time_from_start_sec);
    const double time = static_cast<double>(point.time_from_start_sec);
    const Position position = point_position(point);
    if (!std::isfinite(time) || !std::isfinite(position[0]) || !std::isfinite(position[1]) ||
      !std::isfinite(position[2]) || (control_yaw && !std::isfinite(point.yaw_rad)))
    {
      failure_message = "trajectory contains a non-finite point";
      return false;
    }
    const double duration_sec = time - previous_time;
    if (duration_sec < minimum_segment_duration_sec) {
      failure_message = "trajectory point times must increase by at least 0.1 seconds";
      return false;
    }
    const double horizontal_extent_m = std::hypot(
      position[0] - start_position[0], position[1] - start_position[1]);
    if (horizontal_extent_m > maximum_horizontal_extent_m ||
      std::abs(position[2] - start_position[2]) > maximum_vertical_extent_m)
    {
      failure_message = "trajectory point exceeds the 20 m horizontal or 8 m vertical envelope";
      return false;
    }
    if (control_yaw) {
      const double yaw_rate_radps =
        std::abs(wrap_angle(
          static_cast<double>(point.yaw_rad) - static_cast<double>(previous.yaw_rad))) /
        duration_sec;
      if (yaw_rate_radps > maximum_yaw_rate_radps) {
        failure_message = "trajectory exceeds the 2 rad/s yaw-rate limit";
        return false;
      }
    }
    const Position velocity = segment_velocity(previous, point);
    const double speed_mps = std::hypot(
      std::hypot(velocity[0], velocity[1]), velocity[2]);
    if (speed_mps > maximum_segment_speed_mps) {
      failure_message = "trajectory exceeds the 4 m/s segment-speed limit";
      return false;
    }
    velocities.push_back(velocity);
  }

  for (std::size_t index = 1; index < velocities.size(); ++index) {
    const auto & previous = points[index - 1];
    const auto & point = points[index];
    const double previous_duration = static_cast<double>(
      point.time_from_start_sec - previous.time_from_start_sec);
    const double next_duration = static_cast<double>(
      points[index + 1].time_from_start_sec - point.time_from_start_sec);
    const double midpoint_interval_sec = (previous_duration + next_duration) / 2.0;
    const Position & previous_velocity = velocities[index - 1];
    const Position & velocity = velocities[index];
    const double acceleration_mps2 = std::hypot(
      std::hypot(velocity[0] - previous_velocity[0],
      velocity[1] - previous_velocity[1]), velocity[2] - previous_velocity[2]) /
      midpoint_interval_sec;
    if (acceleration_mps2 > maximum_acceleration_mps2) {
      failure_message = "trajectory exceeds the 3 m/s^2 waypoint acceleration limit";
      return false;
    }
  }

  failure_message.clear();
  return true;
}

TrajectorySample interpolate_trajectory(
  const std::vector<swarm_interfaces::msg::TrajectoryPoint> & points,
  const double elapsed_sec)
{
  if (points.size() < 2 || !std::isfinite(elapsed_sec)) {
    throw std::invalid_argument("trajectory interpolation needs points and finite elapsed time");
  }
  if (elapsed_sec <= static_cast<double>(points.front().time_from_start_sec)) {
    return {point_position(points.front()), static_cast<double>(points.front().yaw_rad)};
  }
  for (std::size_t index = 1; index < points.size(); ++index) {
    const auto & start = points[index - 1];
    const auto & end = points[index];
    const double start_time = static_cast<double>(start.time_from_start_sec);
    const double end_time = static_cast<double>(end.time_from_start_sec);
    if (elapsed_sec > end_time) {
      continue;
    }
    if (end_time <= start_time) {
      throw std::invalid_argument("trajectory interpolation requires increasing point times");
    }
    const double fraction = std::clamp(
      (elapsed_sec - start_time) / (end_time - start_time), 0.0, 1.0);
    const Position start_position = point_position(start);
    const Position end_position = point_position(end);
    const double yaw_delta = wrap_angle(
      static_cast<double>(end.yaw_rad) - static_cast<double>(start.yaw_rad));
    const double yaw = wrap_angle(static_cast<double>(start.yaw_rad) + fraction * yaw_delta);
    Position position{};
    for (std::size_t axis = 0; axis < position.size(); ++axis) {
      position[axis] = start_position[axis] +
        fraction * (end_position[axis] - start_position[axis]);
    }
    return {position, yaw};
  }
  return {point_position(points.back()), static_cast<double>(points.back().yaw_rad)};
}

}  // namespace vehicle_control
