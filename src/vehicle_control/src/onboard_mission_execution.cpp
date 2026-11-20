// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/vehicle_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace vehicle_control
{
namespace
{
constexpr std::chrono::milliseconds mission_poll_interval(50);
constexpr float custom_mode_enabled = 1.0F;
constexpr float auto_mode = 4.0F;
constexpr float mission_submode = 4.0F;
constexpr float hold_submode = 3.0F;
constexpr double landing_timeout_sec = 90.0;
constexpr double start_position_tolerance_m = 1.0;
}

bool VehicleControlNode::stop_onboard_mission(std::string & failure_message)
{
  const std::function<bool()> is_shutdown = []() {return !rclcpp::ok();};
  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  if (!has_fresh_control_telemetry(telemetry, telemetry.landed)) {
    failure_message = "Cannot confirm mission stop with stale PX4 telemetry";
    return false;
  }
  if (telemetry.failsafe) {
    failure_message = "PX4 failsafe is active; leaving recovery under PX4 control";
    return false;
  }
  if (telemetry.landed) {
    return !telemetry.armed || request_transition(
      "mission ground disarm", px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      0.0F, 0.0F, 0.0F, [](const VehicleTelemetry & current) {return !current.armed;},
      is_shutdown, failure_message);
  }
  if (telemetry.navigation_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
    return true;
  }
  if (telemetry.navigation_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION)
  {
    failure_message = "PX4 mode was changed externally; leaving the selected mode in control";
    return false;
  }
  return request_transition(
    "mission Hold", px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    custom_mode_enabled, auto_mode, hold_submode,
    [](const VehicleTelemetry & current) {
      return current.navigation_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
    }, is_shutdown, failure_message);
}

void VehicleControlNode::execute_onboard_mission(
  const std::shared_ptr<FlyMissionHandle> & goal_handle)
{
  const std::shared_ptr<const FlyMission::Goal> goal = goal_handle->get_goal();
  std::shared_ptr<FlyMission::Result> result = std::make_shared<FlyMission::Result>();
  const std::function<bool()> is_cancelled = [goal_handle]() {
      return goal_handle->is_canceling() || !rclcpp::ok();
    };
  const std::function<bool()> is_shutdown = []() {return !rclcpp::ok();};
  bool mission_started = false;
  bool arm_requested = false;
  std::string failure_message;
  try {
    const VehicleTelemetry reference = px4_adapter_->telemetry();
    if (!onboard_mission_ || !has_fresh_control_telemetry(reference, true) ||
      reference.armed || !reference.landed || is_cancelled())
    {
      throw std::runtime_error("Onboard mission requires fresh landed, disarmed telemetry");
    }
    const OnboardPlan plan = build_onboard_plan(*goal, reference, default_position_tolerance_m_);
    onboard_mission_->upload(plan, is_cancelled);
    const VehicleTelemetry after_upload = px4_adapter_->telemetry();
    const std::optional<GlobalPosition> after_upload_global = global_position(after_upload);
    const std::optional<Position> movement = after_upload_global ? global_to_enu(
      *after_upload_global, swarm_geometry::from_message(goal->starting_position)) : std::nullopt;
    if (is_cancelled() || !has_fresh_control_telemetry(after_upload, true) ||
      !movement || after_upload.armed || !after_upload.landed ||
      distance(*movement, {0.0, 0.0, 0.0}) > start_position_tolerance_m)
    {
      throw std::runtime_error("Vehicle state or coordinate reference changed before mission start");
    }
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      stream_requested_ = false;
      stream_mode_established_ = false;
    }
    RCLCPP_INFO(get_logger(), "Uploaded %zu PX4 mission items for %zu waypoints", plan.items.size(),
      goal->waypoint_positions.size());
    arm_requested = true;
    if (!request_transition(
        "mission arming", px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0F, 0.0F, 0.0F, [](const VehicleTelemetry & current) {return current.armed;},
        is_shutdown, failure_message))
    {
      throw std::runtime_error(failure_message);
    }
    if (is_cancelled()) {
      throw std::runtime_error("Onboard mission canceled after arming confirmation");
    }
    mission_started = true;
    if (!request_transition(
        "Auto Mission", px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        custom_mode_enabled, auto_mode, mission_submode,
        [](const VehicleTelemetry & current) {
          return current.navigation_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION;
        }, is_shutdown, failure_message))
    {
      throw std::runtime_error(failure_message);
    }
    std::int32_t previous_item = -1;
    std::chrono::steady_clock::time_point item_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(static_cast<double>(goal->waypoint_timeout_sec)));
    while (!is_cancelled()) {
      const VehicleTelemetry telemetry = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(telemetry, false) || !onboard_mission_->is_connected() ||
        telemetry.failsafe || !global_position(telemetry)) {
        throw std::runtime_error("PX4 mission monitoring lost fresh DDS or MAVLink telemetry");
      }
      const std::int32_t current = onboard_mission_->current_item();
      result->completed_waypoints = static_cast<std::uint32_t>(std::count_if(
        plan.waypoint_indexes.begin(), plan.waypoint_indexes.end(),
        [current](const std::int32_t index) {return current > index;}));
      const bool finished = current >= static_cast<std::int32_t>(plan.items.size());
      const bool in_landing = goal->land_after_waypoints &&
        current == static_cast<std::int32_t>(plan.items.size()) - 1;
      if (finished && (!goal->land_after_waypoints ||
        (telemetry.landed && has_fresh_control_telemetry(telemetry, true)))) {
        if (!stop_onboard_mission(failure_message)) {
          throw std::runtime_error(failure_message);
        }
        result->success = true;
        result->message = goal->land_after_waypoints ?
          "PX4 onboard mission completed, landed, and disarmed" :
          "PX4 onboard mission completed in native Hold";
        goal_handle->succeed(result);
        finish_goal();
        return;
      }
      if (telemetry.navigation_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION) {
        throw std::runtime_error("PX4 left Mission mode before confirmed mission completion");
      }
      if (current != previous_item) {
        const double timeout_sec = in_landing ? landing_timeout_sec :
          static_cast<double>(goal->waypoint_timeout_sec);
        item_deadline = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(timeout_sec));
        previous_item = current;
      }
      if (std::chrono::steady_clock::now() >= item_deadline) {
        throw std::runtime_error("PX4 mission item exceeded its progress deadline");
      }
      std::shared_ptr<FlyMission::Feedback> feedback = std::make_shared<FlyMission::Feedback>();
      feedback->completed_waypoints = result->completed_waypoints;
      feedback->distance_to_target_m = std::numeric_limits<float>::quiet_NaN();
      if (result->completed_waypoints < goal->waypoint_positions.size() && current > 0) {
        feedback->current_waypoint = result->completed_waypoints + 1;
        const std::optional<GlobalPosition> current_global = global_position(telemetry);
        const std::optional<Position> offset = current_global ? global_to_enu(
          swarm_geometry::from_message(goal->waypoint_positions[result->completed_waypoints]),
          *current_global) : std::nullopt;
        if (offset) {
          feedback->distance_to_target_m = static_cast<float>(distance(*offset, {0.0, 0.0, 0.0}));
        }
      }
      feedback->status = in_landing ? "PX4 landing" : "PX4 executing uploaded mission";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(mission_poll_interval);
    }
    throw std::runtime_error("Onboard mission canceled");
  } catch (const std::exception & exception) {
    result->message = exception.what();
    const bool stopped = (!mission_started && !arm_requested) || stop_onboard_mission(failure_message);
    if (!stopped) {
      result->message += "; mission stop unconfirmed: " + failure_message;
    }
    if (goal_handle->is_canceling() && stopped) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    finish_goal();
  }
}

}  // namespace vehicle_control
