// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/vehicle_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace vehicle_control
{
namespace
{

constexpr std::uint8_t command_accepted = 0;
constexpr int maximum_command_attempts = 3;
constexpr std::chrono::milliseconds worker_poll_interval(50);
constexpr std::chrono::milliseconds feedback_interval(200);
constexpr double minimum_setpoint_rate_hz = 2.0;
constexpr double maximum_setpoint_rate_hz = 100.0;
constexpr double prestream_duration_sec = 1.0;
constexpr float offboard_custom_mode = 6.0F;
constexpr float custom_mode_enabled = 1.0F;
constexpr float arm_command_value = 1.0F;

/// Checks that all three position coordinates are finite.
bool is_finite_position(const Position & position)
{
  return std::isfinite(position[0]) && std::isfinite(position[1]) && std::isfinite(position[2]);
}

/// Builds a steady-clock deadline duration_sec into the future.
std::chrono::steady_clock::time_point deadline_after(const double duration_sec)
{
  return std::chrono::steady_clock::now() +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(duration_sec));
}

/// Returns the PX4 command acknowledgment result name for action error messages.
std::string command_acknowledgement_result_name(const std::uint8_t result)
{
  using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;
  switch (result) {
    case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:
      return "accepted";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
      return "temporarily rejected";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:
      return "denied";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:
      return "unsupported";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:
      return "failed";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:
      return "in progress";
    case VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED:
      return "cancelled";
    default:
      return "unknown";
  }
}

}  // namespace

VehicleControlNode::VehicleControlNode()
: Node("vehicle_control")
{
  enable_flight_commands_ = declare_parameter<bool>("enable_flight_commands", false);
  const std::string topic_prefix = declare_parameter<std::string>("px4_topic_prefix", "/fmu");
  const std::int64_t configured_system_id = declare_parameter<std::int64_t>("px4_system_id", 1);
  setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 20.0);
  telemetry_timeout_sec_ = declare_parameter<double>("telemetry_timeout_sec", 1.0);
  default_navigation_timeout_sec_ = declare_parameter<double>(
    "default_navigation_timeout_sec", 30.0);
  default_position_tolerance_m_ = declare_parameter<double>("default_position_tolerance_m", 0.5);
  command_ack_timeout_sec_ = declare_parameter<double>("command_ack_timeout_sec", 2.0);
  transition_timeout_sec_ = declare_parameter<double>("transition_timeout_sec", 5.0);
  takeoff_timeout_sec_ = declare_parameter<double>("takeoff_timeout_sec", 30.0);

  if (configured_system_id < 1 || configured_system_id > 255) {
    throw std::invalid_argument("px4_system_id must be between 1 and 255");
  }
  px4_system_id_ = static_cast<std::uint8_t>(configured_system_id);
  if (!std::isfinite(setpoint_rate_hz_) || setpoint_rate_hz_ < minimum_setpoint_rate_hz ||
    setpoint_rate_hz_ > maximum_setpoint_rate_hz)
  {
    throw std::invalid_argument("setpoint_rate_hz must be between 2 and 100");
  }
  if (!std::isfinite(telemetry_timeout_sec_) || telemetry_timeout_sec_ <= 0.0 ||
    !std::isfinite(default_navigation_timeout_sec_) || default_navigation_timeout_sec_ <= 0.0 ||
    !std::isfinite(default_position_tolerance_m_) || default_position_tolerance_m_ <= 0.0 ||
    !std::isfinite(command_ack_timeout_sec_) || command_ack_timeout_sec_ <= 0.0 ||
    !std::isfinite(transition_timeout_sec_) || transition_timeout_sec_ <= 0.0 ||
    !std::isfinite(takeoff_timeout_sec_) || takeoff_timeout_sec_ <= 0.0)
  {
    throw std::invalid_argument(
            "vehicle-control timeouts and default tolerance must be positive and finite");
  }

  px4_adapter_ = std::make_shared<Px4Adapter>(*this, topic_prefix, px4_system_id_);
  state_publisher_ = create_publisher<swarm_interfaces::msg::VehicleState>("vehicle_state", 10);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / setpoint_rate_hz_)),
    std::bind(&VehicleControlNode::publish_control_setpoint, this));
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&VehicleControlNode::publish_state, this));

  takeoff_server_ = rclcpp_action::create_server<Takeoff>(
    this, "takeoff",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Takeoff::Goal> goal) {
      return reserve_goal(
        GoalKind::Takeoff, std::isfinite(goal->altitude_m) && goal->altitude_m > 0.0F) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<TakeoffHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<TakeoffHandle> handle) {
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        target_position_ = reserved_start_position_;
        stream_requested_ = true;
      }
      enqueue([this, handle]() {execute_takeoff(handle);});
    });

  navigation_server_ = rclcpp_action::create_server<NavigateTo>(
    this, "navigate_to",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateTo::Goal> goal) {
      const bool is_valid = goal->frame_id == "local_enu" &&
        std::isfinite(goal->target_position.x) && std::isfinite(goal->target_position.y) &&
        std::isfinite(goal->target_position.z) && std::isfinite(goal->position_tolerance_m) &&
        goal->position_tolerance_m >= 0.0F && std::isfinite(goal->timeout_sec) &&
        goal->timeout_sec >= 0.0F;
      return reserve_goal(GoalKind::Navigate, is_valid) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<NavigateHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<NavigateHandle> handle) {
      const std::shared_ptr<const NavigateTo::Goal> goal = handle->get_goal();
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        target_position_ = {
          goal->target_position.x, goal->target_position.y, goal->target_position.z};
      }
      enqueue([this, handle]() {execute_navigation(handle);});
    });

  land_server_ = rclcpp_action::create_server<Land>(
    this, "land",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal> goal) {
      const bool is_valid = std::isfinite(goal->timeout_sec) && goal->timeout_sec >= 0.0F;
      return reserve_goal(GoalKind::Land, is_valid) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [this](const std::shared_ptr<LandHandle>) {
      std::lock_guard<std::mutex> lock(control_mutex_);
      return landing_command_started_ ? rclcpp_action::CancelResponse::REJECT :
        rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<LandHandle> handle) {
      enqueue([this, handle]() {execute_land(handle);});
    });

  worker_thread_ = std::thread(&VehicleControlNode::worker_loop, this);
  RCLCPP_INFO(
    get_logger(), "PX4 adapter uses %s; flight actions are %s.",
    normalize_px4_topic_prefix(topic_prefix).c_str(),
    enable_flight_commands_ ? "enabled" : "disabled");
}

VehicleControlNode::~VehicleControlNode()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_stopping_ = true;
  }
  worker_condition_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool VehicleControlNode::reserve_goal(const GoalKind kind, const bool is_goal_valid)
{
  const char * goal_name = "flight";
  switch (kind) {
    case GoalKind::Takeoff:
      goal_name = "takeoff";
      break;
    case GoalKind::Navigate:
      goal_name = "navigation";
      break;
    case GoalKind::Land:
      goal_name = "landing";
      break;
    case GoalKind::None:
      break;
  }

  if (!enable_flight_commands_) {
    RCLCPP_WARN(get_logger(), "Rejecting %s goal: flight commands are disabled", goal_name);
    return false;
  }
  if (!is_goal_valid) {
    RCLCPP_WARN(get_logger(), "Rejecting %s goal: goal values are invalid", goal_name);
    return false;
  }

  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  if (!has_fresh_control_telemetry(telemetry, true)) {
    RCLCPP_WARN(
      get_logger(), "Rejecting %s goal: PX4 position, status, or land telemetry is stale",
      goal_name);
    return false;
  }

  std::lock_guard<std::mutex> lock(control_mutex_);
  if (goal_active_) {
    RCLCPP_WARN(get_logger(), "Rejecting %s goal: another flight goal is active", goal_name);
    return false;
  }
  if ((kind == GoalKind::Takeoff && (telemetry.armed || !telemetry.landed || telemetry.offboard)) ||
    (kind == GoalKind::Navigate && (!telemetry.armed || !telemetry.offboard)) ||
    (kind == GoalKind::Land && !telemetry.armed && !telemetry.landed))
  {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting %s goal: incompatible PX4 state (armed=%s, offboard=%s, landed=%s, nav_state=%u)",
      goal_name, telemetry.armed ? "true" : "false", telemetry.offboard ? "true" : "false",
      telemetry.landed ? "true" : "false",
      static_cast<unsigned int>(telemetry.navigation_state));
    return false;
  }

  goal_active_ = true;
  reserved_start_position_ = telemetry.position_enu;
  if (telemetry.offboard) {
    stream_mode_established_ = true;
  }
  RCLCPP_INFO(
    get_logger(), "Accepted ROS %s goal; PX4 state: armed=%s, offboard=%s, landed=%s",
    goal_name, telemetry.armed ? "true" : "false", telemetry.offboard ? "true" : "false",
    telemetry.landed ? "true" : "false");
  return true;
}

bool VehicleControlNode::has_fresh_control_telemetry(
  const VehicleTelemetry & telemetry, const bool require_land_state) const
{
  const std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
  const std::function<bool(const std::chrono::steady_clock::time_point &)> is_recent =
    [this, &current_time](
      const std::chrono::steady_clock::time_point & received) {
      const std::chrono::steady_clock::duration age = current_time - received;
      return age >= std::chrono::steady_clock::duration::zero() &&
             age <= std::chrono::duration<double>(telemetry_timeout_sec_);
    };
  return has_fresh_position(telemetry) && telemetry.has_status &&
         is_recent(telemetry.status_received) &&
         (!require_land_state ||
         (telemetry.has_land_state && is_recent(telemetry.land_state_received)));
}

bool VehicleControlNode::has_fresh_position(const VehicleTelemetry & telemetry) const
{
  const std::chrono::steady_clock::duration age =
    std::chrono::steady_clock::now() - telemetry.position_received;
  return telemetry.has_position && telemetry.position_valid &&
         is_finite_position(telemetry.position_enu) &&
         age >= std::chrono::steady_clock::duration::zero() &&
         age <= std::chrono::duration<double>(telemetry_timeout_sec_);
}

bool VehicleControlNode::wait_until(
  const std::function<bool(const VehicleTelemetry &)> & predicate,
  const std::chrono::steady_clock::time_point & deadline,
  const std::function<bool()> & is_cancelled, const bool require_land_state,
  std::string & failure_message)
{
  while (std::chrono::steady_clock::now() < deadline) {
    if (is_cancelled()) {
      failure_message = "goal canceled";
      return false;
    }
    const VehicleTelemetry telemetry = px4_adapter_->telemetry();
    if (!has_fresh_control_telemetry(telemetry, require_land_state)) {
      failure_message = "PX4 telemetry became stale or invalid";
      return false;
    }
    if (predicate(telemetry)) {
      return true;
    }
    std::this_thread::sleep_for(worker_poll_interval);
  }
  failure_message = "transition timed out";
  return false;
}

bool VehicleControlNode::wait_for_command_acknowledgement(
  const std::uint32_t command, const std::uint64_t generation,
  const std::chrono::steady_clock::time_point & deadline,
  const std::function<bool()> & is_cancelled, std::string & failure_message)
{
  while (std::chrono::steady_clock::now() < deadline) {
    if (is_cancelled()) {
      failure_message = "goal canceled";
      return false;
    }
    const std::optional<CommandAcknowledgement> acknowledgement =
      px4_adapter_->acknowledgement_after(command, generation);
    if (acknowledgement) {
      if (acknowledgement->result != command_accepted) {
        failure_message = "PX4 command acknowledgement result=" +
          std::to_string(acknowledgement->result) + " (" +
          command_acknowledgement_result_name(acknowledgement->result) + "), result_param1=" +
          std::to_string(acknowledgement->result_parameter_1) + ", result_param2=" +
          std::to_string(acknowledgement->result_parameter_2);
        return false;
      }
      return true;
    }
    std::this_thread::sleep_for(worker_poll_interval);
  }
  failure_message = "command acknowledgement timed out";
  return false;
}

bool VehicleControlNode::request_transition(
  const std::string & transition_name, const std::uint32_t command,
  const float parameter_1, const float parameter_2,
  const std::function<bool(const VehicleTelemetry &)> & predicate,
  const std::function<bool()> & is_cancelled, std::string & failure_message)
{
  RCLCPP_INFO(
    get_logger(), "Requesting PX4 %s transition (command=%u, max_attempts=%d)",
    transition_name.c_str(), static_cast<unsigned int>(command), maximum_command_attempts);
  for (int attempt = 0; attempt < maximum_command_attempts; ++attempt) {
    if (is_cancelled()) {
      failure_message = "goal canceled before requesting PX4 " + transition_name + " transition";
      return false;
    }
    RCLCPP_DEBUG(
      get_logger(), "Publishing PX4 %s command attempt %d/%d", transition_name.c_str(),
      attempt + 1, maximum_command_attempts);
    const std::int64_t timestamp_us = get_clock()->now().nanoseconds() / 1000;
    const std::uint64_t generation = px4_adapter_->publish_vehicle_command(
      command, parameter_1, parameter_2, timestamp_us);
    const std::chrono::steady_clock::time_point acknowledgement_deadline =
      deadline_after(command_ack_timeout_sec_);
    if (!wait_for_command_acknowledgement(
        command, generation, acknowledgement_deadline, is_cancelled, failure_message))
    {
      if (failure_message == "command acknowledgement timed out") {
        if (attempt + 1 == maximum_command_attempts) {
          failure_message = "PX4 did not acknowledge " + transition_name + " transition (command " +
            std::to_string(command) + ") after " + std::to_string(maximum_command_attempts) +
            " attempts";
          RCLCPP_ERROR(get_logger(), "%s", failure_message.c_str());
          return false;
        }
        RCLCPP_WARN(
          get_logger(), "No PX4 acknowledgement for %s (command=%u, attempt=%d/%d); retrying",
          transition_name.c_str(), static_cast<unsigned int>(command), attempt + 1,
          maximum_command_attempts);
        continue;
      }
      if (failure_message != "goal canceled") {
        failure_message = "PX4 did not accept " + transition_name + " transition (command " +
          std::to_string(command) + "): " + failure_message;
        RCLCPP_ERROR(get_logger(), "%s", failure_message.c_str());
      } else {
        failure_message = "goal canceled while waiting for PX4 " + transition_name +
          " acknowledgement";
      }
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "PX4 acknowledged %s command (command=%u, attempt=%d/%d)",
      transition_name.c_str(), static_cast<unsigned int>(command), attempt + 1,
      maximum_command_attempts);

    const std::chrono::steady_clock::time_point transition_deadline =
      deadline_after(transition_timeout_sec_);
    if (wait_until(predicate, transition_deadline, is_cancelled, false, failure_message)) {
      RCLCPP_INFO(
        get_logger(), "PX4 confirmed %s transition (command=%u)", transition_name.c_str(),
        static_cast<unsigned int>(command));
      return true;
    }
    if (failure_message != "transition timed out") {
      if (failure_message != "goal canceled") {
        failure_message = "PX4 " + transition_name + " transition failed: " + failure_message;
        RCLCPP_ERROR(get_logger(), "%s", failure_message.c_str());
      } else {
        failure_message = "goal canceled while waiting for PX4 " + transition_name + " transition";
      }
      return false;
    }
    if (attempt + 1 == maximum_command_attempts) {
      failure_message = "PX4 acknowledged " + transition_name + " transition command " +
        std::to_string(command) + " but telemetry did not confirm the transition after " +
        std::to_string(maximum_command_attempts) + " attempts";
      RCLCPP_ERROR(get_logger(), "%s", failure_message.c_str());
      return false;
    }
    RCLCPP_WARN(
      get_logger(), "PX4 acknowledged %s but telemetry did not confirm the transition within %.1f s; "
      "retrying (%d/%d)", transition_name.c_str(), transition_timeout_sec_, attempt + 1,
      maximum_command_attempts);
  }
  failure_message = "PX4 " + transition_name + " transition failed after bounded retries";
  return false;
}

bool VehicleControlNode::wait_for_prestream(
  const std::chrono::steady_clock::time_point & deadline,
  const std::function<bool()> & is_cancelled, std::string & failure_message)
{
  while (std::chrono::steady_clock::now() < deadline) {
    if (is_cancelled()) {
      failure_message = "goal canceled";
      return false;
    }
    if (!has_fresh_control_telemetry(px4_adapter_->telemetry(), false)) {
      failure_message = "PX4 telemetry became stale or invalid before Offboard entry";
      return false;
    }
    std::this_thread::sleep_for(worker_poll_interval);
  }
  return true;
}

void VehicleControlNode::hold_current_position()
{
  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  std::lock_guard<std::mutex> lock(control_mutex_);
  if (telemetry.has_position && telemetry.position_valid &&
    is_finite_position(telemetry.position_enu))
  {
    target_position_ = telemetry.position_enu;
  }
}

void VehicleControlNode::publish_state()
{
  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  swarm_interfaces::msg::VehicleState state{};
  state.header.stamp = now();
  state.header.frame_id = "local_enu";
  state.position.x = telemetry.position_enu[0];
  state.position.y = telemetry.position_enu[1];
  state.position.z = telemetry.position_enu[2];
  state.velocity.x = telemetry.velocity_enu[0];
  state.velocity.y = telemetry.velocity_enu[1];
  state.velocity.z = telemetry.velocity_enu[2];
  const std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
  const std::chrono::steady_clock::duration status_age = current_time - telemetry.status_received;
  const std::chrono::steady_clock::duration land_age = current_time - telemetry.land_state_received;
  const std::chrono::duration<double> freshness(telemetry_timeout_sec_);
  state.position_valid = has_fresh_position(telemetry);
  state.armed = telemetry.has_status && status_age <= freshness && telemetry.armed;
  state.offboard = telemetry.has_status && status_age <= freshness && telemetry.offboard;
  state.landed = telemetry.has_land_state && land_age <= freshness && telemetry.landed;
  state.battery_available = telemetry.battery_available;
  state.battery_percentage = telemetry.battery_percentage;
  state_publisher_->publish(state);
}

void VehicleControlNode::publish_control_setpoint()
{
  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  Position position{};
  bool should_publish = false;
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (telemetry.offboard) {
      stream_mode_established_ = true;
    }
    if (stream_requested_ && stream_mode_established_ && telemetry.has_status &&
      telemetry.offboard == false && has_fresh_control_telemetry(telemetry, false))
    {
      stream_requested_ = false;
      stream_mode_established_ = false;
    }
    should_publish = stream_requested_ || telemetry.offboard;
    position = target_position_;
  }
  if (should_publish) {
    const std::int64_t timestamp_us = get_clock()->now().nanoseconds() / 1000;
    px4_adapter_->publish_position_setpoint(position, timestamp_us);
  }
}

void VehicleControlNode::finish_goal()
{
  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  std::lock_guard<std::mutex> lock(control_mutex_);
  goal_active_ = false;
  landing_command_started_ = false;
  if (telemetry.has_status && telemetry.offboard == false &&
    has_fresh_control_telemetry(telemetry, false))
  {
    stream_requested_ = false;
    stream_mode_established_ = false;
  }
}

void VehicleControlNode::enqueue(std::function<void()> job)
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_jobs_.push_back(std::move(job));
  }
  worker_condition_.notify_one();
}

void VehicleControlNode::worker_loop()
{
  for (;;) {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_condition_.wait(lock, [this]() {return worker_stopping_ || !worker_jobs_.empty();});
      if (worker_stopping_ && worker_jobs_.empty()) {
        return;
      }
      job = std::move(worker_jobs_.front());
      worker_jobs_.pop_front();
    }
    try {
      job();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Unhandled flight-action error: %s", exception.what());
      finish_goal();
    }
  }
}

void VehicleControlNode::execute_takeoff(const std::shared_ptr<TakeoffHandle> & goal_handle)
{
  std::shared_ptr<Takeoff::Result> result = std::make_shared<Takeoff::Result>();
  const std::shared_ptr<const Takeoff::Goal> goal = goal_handle->get_goal();
  const std::function<bool()> is_cancelled =
    [goal_handle]() {return goal_handle->is_canceling() || !rclcpp::ok();};
  const std::chrono::steady_clock::time_point goal_deadline = deadline_after(takeoff_timeout_sec_);
  std::string failure_message;
  RCLCPP_INFO(
    get_logger(), "Starting takeoff execution (target_altitude=%.2f m)",
    static_cast<double>(goal->altitude_m));
  try {
    Position start_position{};
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      start_position = reserved_start_position_;
    }
    const std::chrono::steady_clock::time_point prestream_deadline =
      deadline_after(prestream_duration_sec);
    if (!wait_for_prestream(prestream_deadline, is_cancelled, failure_message)) {
      throw std::runtime_error(failure_message);
    }

    if (!request_transition(
        "Offboard mode",
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        custom_mode_enabled, offboard_custom_mode,
        [](const VehicleTelemetry & telemetry) {return telemetry.offboard;},
        is_cancelled, failure_message))
    {
      throw std::runtime_error(failure_message);
    }
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      stream_mode_established_ = true;
    }

    if (!request_transition(
        "arming",
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        arm_command_value, 0.0F,
        [](const VehicleTelemetry & telemetry) {return telemetry.armed;},
        is_cancelled, failure_message))
    {
      throw std::runtime_error(failure_message);
    }

    Position target_position = start_position;
    target_position[2] += static_cast<double>(goal->altitude_m);
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      target_position_ = target_position;
    }

    std::chrono::steady_clock::time_point last_feedback = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < goal_deadline) {
      if (is_cancelled()) {
        failure_message = "goal canceled";
        throw std::runtime_error(failure_message);
      }
      const VehicleTelemetry telemetry = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(telemetry, false)) {
        failure_message = "PX4 telemetry became stale or invalid during takeoff";
        throw std::runtime_error(failure_message);
      }
      if (distance(telemetry.position_enu, target_position) <= default_position_tolerance_m_) {
        result->success = true;
        result->message = "Takeoff altitude reached";
        RCLCPP_INFO(
          get_logger(), "Takeoff succeeded at target altitude %.2f m",
          static_cast<double>(goal->altitude_m));
        goal_handle->succeed(result);
        finish_goal();
        return;
      }
      if (std::chrono::steady_clock::now() - last_feedback >= feedback_interval) {
        std::shared_ptr<Takeoff::Feedback> feedback = std::make_shared<Takeoff::Feedback>();
        feedback->current_altitude_m = static_cast<float>(
          telemetry.position_enu[2] - start_position[2]);
        feedback->target_altitude_m = goal->altitude_m;
        goal_handle->publish_feedback(feedback);
        last_feedback = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_for(worker_poll_interval);
    }
    failure_message = "takeoff timed out before reaching the requested altitude";
    throw std::runtime_error(failure_message);
  } catch (const std::exception & exception) {
    hold_current_position();
    result->success = false;
    result->message = exception.what();
    const bool was_cancelled = is_cancelled();
    if (was_cancelled) {
      RCLCPP_WARN(get_logger(), "Takeoff canceled: %s", exception.what());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Takeoff failed: %s", exception.what());
      goal_handle->abort(result);
    }
    finish_goal();
  }
}

void VehicleControlNode::execute_navigation(const std::shared_ptr<NavigateHandle> & goal_handle)
{
  std::shared_ptr<NavigateTo::Result> result = std::make_shared<NavigateTo::Result>();
  result->final_distance_m = std::numeric_limits<float>::quiet_NaN();
  const std::shared_ptr<const NavigateTo::Goal> goal = goal_handle->get_goal();
  const double tolerance = goal->position_tolerance_m == 0.0F ?
    default_position_tolerance_m_ : static_cast<double>(goal->position_tolerance_m);
  const double timeout = goal->timeout_sec == 0.0F ?
    default_navigation_timeout_sec_ : static_cast<double>(goal->timeout_sec);
  const Position target_position{
    goal->target_position.x, goal->target_position.y, goal->target_position.z};
  const std::chrono::steady_clock::time_point deadline = deadline_after(timeout);
  const std::function<bool()> is_cancelled =
    [goal_handle]() {return goal_handle->is_canceling() || !rclcpp::ok();};
  std::chrono::steady_clock::time_point last_feedback = std::chrono::steady_clock::now();
  std::string failure_message;
  RCLCPP_INFO(
    get_logger(), "Starting navigation (target_enu=[%.2f, %.2f, %.2f] m, tolerance=%.2f m, "
    "timeout=%.1f s)", target_position[0], target_position[1], target_position[2], tolerance,
    timeout);

  try {
    while (std::chrono::steady_clock::now() < deadline) {
      if (is_cancelled()) {
        hold_current_position();
        result->success = false;
        result->message = "Navigation canceled; holding the current position";
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        const VehicleTelemetry telemetry = px4_adapter_->telemetry();
        if (telemetry.position_valid) {
          result->final_distance_m = static_cast<float>(
            distance(telemetry.position_enu, target_position));
        }
        goal_handle->canceled(result);
        finish_goal();
        return;
      }
      const VehicleTelemetry telemetry = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(telemetry, false)) {
        failure_message = "PX4 telemetry became stale or invalid during navigation";
        throw std::runtime_error(failure_message);
      }
      const double target_distance = distance(telemetry.position_enu, target_position);
      result->final_distance_m = static_cast<float>(target_distance);
      if (target_distance <= tolerance) {
        result->success = true;
        result->message = "Navigation target reached";
        RCLCPP_INFO(
          get_logger(), "Navigation succeeded (remaining_distance=%.2f m)", target_distance);
        goal_handle->succeed(result);
        finish_goal();
        return;
      }
      if (std::chrono::steady_clock::now() - last_feedback >= feedback_interval) {
        std::shared_ptr<NavigateTo::Feedback> feedback =
          std::make_shared<NavigateTo::Feedback>();
        feedback->distance_to_target_m = static_cast<float>(target_distance);
        goal_handle->publish_feedback(feedback);
        last_feedback = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_for(worker_poll_interval);
    }
    failure_message = "navigation timed out before reaching the target";
    throw std::runtime_error(failure_message);
  } catch (const std::exception & exception) {
    hold_current_position();
    result->success = false;
    result->message = exception.what();
    const VehicleTelemetry telemetry = px4_adapter_->telemetry();
    if (telemetry.position_valid && is_finite_position(telemetry.position_enu)) {
      result->final_distance_m = static_cast<float>(
        distance(telemetry.position_enu, target_position));
    }
    const bool was_cancelled = is_cancelled();
    if (was_cancelled) {
      RCLCPP_WARN(get_logger(), "Navigation canceled: %s", exception.what());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Navigation failed: %s", exception.what());
      goal_handle->abort(result);
    }
    finish_goal();
  }
}

void VehicleControlNode::execute_land(const std::shared_ptr<LandHandle> & goal_handle)
{
  std::shared_ptr<Land::Result> result = std::make_shared<Land::Result>();
  result->landed = false;
  const std::shared_ptr<const Land::Goal> goal = goal_handle->get_goal();
  const double timeout = goal->timeout_sec == 0.0F ?
    transition_timeout_sec_ * 6.0 : static_cast<double>(goal->timeout_sec);
  const std::chrono::steady_clock::time_point deadline = deadline_after(timeout);
  const std::function<bool()> is_cancelled =
    [goal_handle]() {return goal_handle->is_canceling() || !rclcpp::ok();};
  std::string failure_message;
  RCLCPP_INFO(get_logger(), "Starting landing action (timeout=%.1f s)", timeout);

  try {
    VehicleTelemetry telemetry = px4_adapter_->telemetry();
    if (telemetry.landed) {
      result->success = true;
      result->landed = true;
      result->message = "Vehicle is already landed";
      RCLCPP_INFO(get_logger(), "Landing succeeded: PX4 already reports landed");
      goal_handle->succeed(result);
      finish_goal();
      return;
    }
    if (is_cancelled()) {
      failure_message = "goal canceled";
      throw std::runtime_error(failure_message);
    }
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      landing_command_started_ = true;
    }
    const bool already_in_auto_land =
      telemetry.navigation_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
    if (!already_in_auto_land && !request_transition(
        "Auto Land mode",
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0F, 0.0F,
        [](const VehicleTelemetry & current) {
          return current.navigation_state ==
                 px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
        }, is_cancelled, failure_message))
    {
      throw std::runtime_error(failure_message);
    }

    while (std::chrono::steady_clock::now() < deadline) {
      if (is_cancelled()) {
        failure_message = "goal canceled before PX4 accepted the landing transition";
        throw std::runtime_error(failure_message);
      }
      telemetry = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(telemetry, true)) {
        failure_message = "PX4 telemetry became stale or invalid during landing";
        throw std::runtime_error(failure_message);
      }
      std::shared_ptr<Land::Feedback> feedback = std::make_shared<Land::Feedback>();
      feedback->status = telemetry.landed ? "landed" : "PX4 landing in progress";
      goal_handle->publish_feedback(feedback);
      if (telemetry.landed) {
        result->success = true;
        result->landed = true;
        result->message = "PX4 reported the vehicle landed";
        RCLCPP_INFO(get_logger(), "Landing succeeded: PX4 land detector reports landed");
        goal_handle->succeed(result);
        finish_goal();
        return;
      }
      std::this_thread::sleep_for(feedback_interval);
    }
    failure_message = "landing timed out before PX4 reported landed";
    throw std::runtime_error(failure_message);
  } catch (const std::exception & exception) {
    result->success = false;
    result->landed = false;
    result->message = exception.what();
    const bool was_cancelled = is_cancelled();
    if (was_cancelled) {
      RCLCPP_WARN(get_logger(), "Landing canceled: %s", exception.what());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Landing failed: %s", exception.what());
      goal_handle->abort(result);
    }
    finish_goal();
  }
}

}  // namespace vehicle_control
