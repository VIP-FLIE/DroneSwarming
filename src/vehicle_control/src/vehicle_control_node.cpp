// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/vehicle_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "vehicle_control/orbit_progress.hpp"

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
constexpr double minimum_trajectory_settle_duration_sec = 0.5;
constexpr double minimum_navigation_yaw_settle_duration_sec = 0.5;
constexpr double minimum_look_at_distance_m = 0.1;
constexpr double maximum_look_at_yaw_rate_radps = 2.0;
constexpr double maximum_trajectory_yaw_tolerance_rad = 1.0;
constexpr double default_orbit_yaw_tolerance_rad = 0.5235987755982988;
constexpr std::chrono::seconds orbit_yaw_error_duration(2);
constexpr double maximum_trajectory_position_tolerance_m = 2.0;
constexpr double maximum_trajectory_goal_timeout_sec = 240.0;
constexpr double max_orbit_centripetal_acceleration_mps2 = 2.0;
constexpr double orbit_radial_tolerance_floor_m = 0.5;
constexpr double orbit_relative_radial_tolerance = 0.1;
constexpr double orbit_altitude_tolerance_m = 0.5;
constexpr double orbit_hold_velocity_tolerance_mps = 0.3;
constexpr double orbit_maximum_sample_angle_rad = 1.57079632679489661923;
constexpr std::uint8_t orbit_yaw_face_center =
  px4_msgs::msg::VehicleCommand::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER;
constexpr std::uint8_t orbit_yaw_uncontrolled =
  px4_msgs::msg::VehicleCommand::ORBIT_YAW_BEHAVIOUR_UNCONTROLLED;
constexpr std::chrono::milliseconds orbit_mode_confirmation_duration(500);
constexpr std::chrono::milliseconds orbit_hold_settle_duration(500);
constexpr float offboard_custom_mode = 6.0F;
constexpr float auto_custom_mode = 4.0F;
constexpr float auto_loiter_sub_mode = 3.0F;
constexpr float custom_mode_enabled = 1.0F;
constexpr float arm_command_value = 1.0F;
constexpr float disarm_command_value = 0.0F;
//PX4 Auto Loiter uses the AUTO main mode and LOITER sub-mode.

/// Checks that all three position coordinates are finite.
bool is_finite_position(const Position & position)
{
  return std::isfinite(position[0]) && std::isfinite(position[1]) && std::isfinite(position[2]);
}

/// Returns the shortest signed difference between two ENU yaw angles.
double angular_error(const double target_yaw_rad, const double current_yaw_rad)
{
  return std::atan2(
    std::sin(target_yaw_rad - current_yaw_rad),
    std::cos(target_yaw_rad - current_yaw_rad));
}

/// Returns the ENU yaw toward a point when its horizontal bearing is defined.
std::optional<double> yaw_toward_position(
  const Position & position_enu, const Position & look_at_position_enu)
{
  const double east_offset_m = look_at_position_enu[0] - position_enu[0];
  const double north_offset_m = look_at_position_enu[1] - position_enu[1];
  //a point directly overhead has no horizontal yaw bearing.
  if (std::hypot(east_offset_m, north_offset_m) <= minimum_look_at_distance_m) {
    return std::nullopt;
  }
  return std::atan2(north_offset_m, east_offset_m);
}

/// Returns the horizontal speed of a local ENU velocity vector.
double horizontal_speed(const Position & velocity)
{
  return std::hypot(velocity[0], velocity[1]);
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
  max_orbit_radius_m_ = declare_parameter<double>("max_orbit_radius_m", 20.0);
  max_orbit_speed_mps_ = declare_parameter<double>("max_orbit_speed_mps", 3.0);
  max_orbit_center_distance_m_ = declare_parameter<double>(
    "max_orbit_center_distance_m", 100.0);
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
    !std::isfinite(max_orbit_radius_m_) || max_orbit_radius_m_ < 1.0 ||
    !std::isfinite(max_orbit_speed_mps_) || max_orbit_speed_mps_ <= 0.0 ||
    !std::isfinite(max_orbit_center_distance_m_) || max_orbit_center_distance_m_ <= 0.0 ||
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
      if (!std::isfinite(goal->altitude_m) || goal->altitude_m <= 0.0F) {
        RCLCPP_WARN(
          get_logger(), "Rejecting takeoff: altitude must be finite and positive "
          "(requested_altitude=%.2f m)", static_cast<double>(goal->altitude_m));
        return rclcpp_action::GoalResponse::REJECT;
      }
      return reserve_goal(GoalKind::Takeoff) ?
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
      const Position target_position{
        goal->target_position.x, goal->target_position.y, goal->target_position.z};
      const Position look_at_position{
        goal->look_at_position.x, goal->look_at_position.y, goal->look_at_position.z};
      const bool has_valid_look_at_position = !goal->look_at_target ||
        (is_finite_position(look_at_position) &&
        std::hypot(look_at_position[0] - target_position[0],
        look_at_position[1] - target_position[1]) > minimum_look_at_distance_m);
      const bool is_valid = goal->frame_id == "local_enu" &&
        std::isfinite(goal->target_position.x) && std::isfinite(goal->target_position.y) &&
        std::isfinite(goal->target_position.z) && std::isfinite(goal->position_tolerance_m) &&
        goal->position_tolerance_m >= 0.0F && std::isfinite(goal->timeout_sec) &&
        goal->timeout_sec >= 0.0F && has_valid_look_at_position;
      if (!is_valid) {
        RCLCPP_WARN(
          get_logger(), "Rejecting navigation: invalid goal (frame=%s, target=[%.2f, %.2f, "
          "%.2f] m, position_tolerance=%.2f m, look_at=%s, look_at_position=[%.2f, %.2f, "
          "%.2f] m, horizontal_look_at_distance=%.2f m, timeout=%.2f s)",
          goal->frame_id.c_str(), target_position[0], target_position[1], target_position[2],
          static_cast<double>(goal->position_tolerance_m),
          goal->look_at_target ? "true" : "false", look_at_position[0], look_at_position[1],
          look_at_position[2], std::hypot(look_at_position[0] - target_position[0],
          look_at_position[1] - target_position[1]), static_cast<double>(goal->timeout_sec));
        return rclcpp_action::GoalResponse::REJECT;
      }
      return reserve_goal(GoalKind::Navigate, goal->look_at_target) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<NavigateHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<NavigateHandle> handle) {
      const std::shared_ptr<const NavigateTo::Goal> goal = handle->get_goal();
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        target_position_ = {
          goal->target_position.x, goal->target_position.y, goal->target_position.z};
        if (goal->look_at_target) {
          look_at_position_enu_ = Position{
            goal->look_at_position.x, goal->look_at_position.y, goal->look_at_position.z};
          target_yaw_rad_ = reserved_start_yaw_rad_;
          look_at_setpoint_updated_at_ = std::chrono::steady_clock::now();
        }
      }
      enqueue([this, handle]() {execute_navigation(handle);});
    });

  follow_trajectory_server_ = rclcpp_action::create_server<FollowTrajectory>(
    this, "follow_trajectory",
    [this](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowTrajectory::Goal> goal) {
      std::string validation_error;
      const double timeout_sec = goal->timeout_sec == 0.0F ? default_navigation_timeout_sec_ :
        static_cast<double>(goal->timeout_sec);
      const bool is_trajectory_valid = validate_trajectory(
        goal->points, goal->control_yaw, validation_error);
      const bool has_valid_parameters =
        std::isfinite(goal->position_tolerance_m) && goal->position_tolerance_m >= 0.0F &&
        static_cast<double>(goal->position_tolerance_m) <=
        maximum_trajectory_position_tolerance_m &&
        std::isfinite(goal->yaw_tolerance_rad) && goal->yaw_tolerance_rad >= 0.0F &&
        (goal->control_yaw || goal->yaw_tolerance_rad == 0.0F) &&
        static_cast<double>(goal->yaw_tolerance_rad) <= maximum_trajectory_yaw_tolerance_rad &&
        std::isfinite(goal->timeout_sec) && goal->timeout_sec >= 0.0F &&
        timeout_sec <= maximum_trajectory_goal_timeout_sec &&
        (goal->points.empty() ||
        timeout_sec >= static_cast<double>(goal->points.back().time_from_start_sec) +
        minimum_trajectory_settle_duration_sec);
      const VehicleTelemetry telemetry = px4_adapter_->telemetry();
      const Position first_position{
        goal->points.empty() ? 0.0 : goal->points.front().position.x,
        goal->points.empty() ? 0.0 : goal->points.front().position.y,
        goal->points.empty() ? 0.0 : goal->points.front().position.z};
      const bool starts_at_vehicle = goal->points.empty() ||
        (has_fresh_position(telemetry) &&
        distance(telemetry.position_enu, first_position) <= 1.0);
      const bool has_valid_yaw = telemetry.yaw_valid && std::isfinite(telemetry.yaw_enu_rad);
      if (!is_trajectory_valid || !has_valid_parameters || !starts_at_vehicle ||
        (goal->control_yaw && !has_valid_yaw))
      {
        const std::string reason = !is_trajectory_valid ? validation_error :
          (!has_valid_parameters ? "goal parameters are out of bounds" :
          (!starts_at_vehicle ? "first trajectory point must match current position within 1 m" :
          "requested yaw telemetry is invalid"));
        const double start_distance_m = goal->points.empty() || !has_fresh_position(telemetry) ?
          std::numeric_limits<double>::quiet_NaN() :
          distance(telemetry.position_enu, first_position);
        RCLCPP_WARN(
          get_logger(), "Rejecting FollowTrajectory: %s (points=%zu, start_distance=%.2f m, "
          "position_tolerance=%.2f m, control_yaw=%s, yaw_tolerance=%.3f rad, "
          "yaw_valid=%s, measured_yaw=%.3f rad, duration=%.2f s, timeout=%.2f s)",
          reason.c_str(), goal->points.size(), start_distance_m,
          static_cast<double>(goal->position_tolerance_m), goal->control_yaw ? "true" : "false",
          static_cast<double>(goal->yaw_tolerance_rad), telemetry.yaw_valid ? "true" : "false",
          telemetry.yaw_enu_rad,
          goal->points.empty() ? std::numeric_limits<double>::quiet_NaN() :
          static_cast<double>(goal->points.back().time_from_start_sec), timeout_sec);
        return rclcpp_action::GoalResponse::REJECT;
      }
      return reserve_goal(GoalKind::FollowTrajectory, goal->control_yaw) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<FollowTrajectoryHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<FollowTrajectoryHandle> handle) {
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        target_position_ = reserved_start_position_;
        if (handle->get_goal()->control_yaw) {
          target_yaw_rad_ = reserved_start_yaw_rad_;
        }
      }
      enqueue([this, handle]() {execute_follow_trajectory(handle);});
    });

  orbit_server_ = rclcpp_action::create_server<OrbitPoint>(
    this, "orbit_point",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const OrbitPoint::Goal> goal) {
      const double radius_m = static_cast<double>(goal->radius_m);
      const double speed_mps = static_cast<double>(goal->tangential_speed_mps);
      const double center_distance_m = std::hypot(
        goal->center_position.x, goal->center_position.y);
      const bool is_valid = goal->frame_id == "local_enu" &&
        std::isfinite(goal->center_position.x) && std::isfinite(goal->center_position.y) &&
        std::isfinite(goal->center_position.z) &&
        center_distance_m <= max_orbit_center_distance_m_ &&
        std::isfinite(radius_m) && radius_m >= 1.0 && radius_m <= max_orbit_radius_m_ &&
        std::isfinite(speed_mps) && speed_mps > 0.0 && speed_mps <= max_orbit_speed_mps_ &&
        speed_mps * speed_mps / radius_m < max_orbit_centripetal_acceleration_mps2 &&
        std::isfinite(goal->yaw_tolerance_rad) && goal->yaw_tolerance_rad >= 0.0F &&
        static_cast<double>(goal->yaw_tolerance_rad) <= maximum_trajectory_yaw_tolerance_rad &&
        (goal->face_center || goal->yaw_tolerance_rad == 0.0F) &&
        std::isfinite(goal->revolutions) && goal->revolutions > 0.0F &&
        std::isfinite(goal->timeout_sec) && goal->timeout_sec > 0.0F;
      if (!is_valid) {
        RCLCPP_WARN(
          get_logger(), "Rejecting OrbitPoint: invalid goal (frame=%s, center=[%.2f, %.2f, "
          "%.2f] m, center_distance=%.2f m, radius=%.2f m, speed=%.2f m/s, "
          "centripetal_acceleration=%.2f m/s^2, face_center=%s, yaw_tolerance=%.3f rad, "
          "revolutions=%.2f, timeout=%.2f s)",
          goal->frame_id.c_str(), goal->center_position.x, goal->center_position.y,
          goal->center_position.z, center_distance_m, radius_m, speed_mps,
          radius_m > 0.0 ? speed_mps * speed_mps / radius_m :
          std::numeric_limits<double>::quiet_NaN(),
          goal->face_center ? "true" : "false",
          static_cast<double>(goal->yaw_tolerance_rad), static_cast<double>(goal->revolutions),
          static_cast<double>(goal->timeout_sec));
        return rclcpp_action::GoalResponse::REJECT;
      }
      return reserve_goal(GoalKind::Orbit, goal->face_center) ?
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<OrbitHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<OrbitHandle> handle) {
      enqueue([this, handle]() {execute_orbit(handle);});
    });

  land_server_ = rclcpp_action::create_server<Land>(
    this, "land",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal> goal) {
      if (!std::isfinite(goal->timeout_sec) || goal->timeout_sec < 0.0F) {
        RCLCPP_WARN(
          get_logger(), "Rejecting landing: timeout must be finite and nonnegative "
          "(timeout=%.2f s)", static_cast<double>(goal->timeout_sec));
        return rclcpp_action::GoalResponse::REJECT;
      }
      return reserve_goal(GoalKind::Land) ?
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

bool VehicleControlNode::reserve_goal(
  const GoalKind kind, const bool require_yaw)
{
  [[maybe_unused]] const char * goal_name = "flight";
  switch (kind) {
    case GoalKind::Takeoff:
      goal_name = "takeoff";
      break;
    case GoalKind::Navigate:
      goal_name = "navigation";
      break;
    case GoalKind::FollowTrajectory:
      goal_name = "trajectory";
      break;
    case GoalKind::Orbit:
      goal_name = "orbit";
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
  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  if (!has_fresh_control_telemetry(telemetry, true)) {
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    const double position_age_sec = telemetry.has_position ?
      std::chrono::duration<double>(now - telemetry.position_received).count() :
      std::numeric_limits<double>::quiet_NaN();
    const double status_age_sec = telemetry.has_status ?
      std::chrono::duration<double>(now - telemetry.status_received).count() :
      std::numeric_limits<double>::quiet_NaN();
    const double land_age_sec = telemetry.has_land_state ?
      std::chrono::duration<double>(now - telemetry.land_state_received).count() :
      std::numeric_limits<double>::quiet_NaN();
    RCLCPP_WARN(
      get_logger(), "Rejecting %s: telemetry unavailable or stale (position_valid=%s, "
      "position_age=%.2f s, status_age=%.2f s, land_age=%.2f s, limit=%.2f s)",
      goal_name, telemetry.position_valid ? "true" : "false", position_age_sec,
      status_age_sec, land_age_sec, telemetry_timeout_sec_);
    return false;
  }
  if (require_yaw && (!telemetry.yaw_valid || !std::isfinite(telemetry.yaw_enu_rad))) {
    RCLCPP_WARN(
      get_logger(), "Rejecting %s: requested yaw requires valid telemetry "
      "(yaw_valid=%s, measured_yaw=%.3f rad, position=[%.2f, %.2f, %.2f] m, landed=%s)",
      goal_name, telemetry.yaw_valid ? "true" : "false", telemetry.yaw_enu_rad,
      telemetry.position_enu[0], telemetry.position_enu[1], telemetry.position_enu[2],
      telemetry.landed ? "true" : "false");
    return false;
  }
  if (kind == GoalKind::Orbit &&
    (!telemetry.velocity_valid || !std::isfinite(telemetry.velocity_enu[0]) ||
    !std::isfinite(telemetry.velocity_enu[1]) || !std::isfinite(telemetry.velocity_enu[2]) ||
    !telemetry.xy_global || !telemetry.z_global ||
    !std::isfinite(telemetry.reference_latitude_deg) ||
    !std::isfinite(telemetry.reference_longitude_deg) ||
    !std::isfinite(telemetry.reference_altitude_amsl_m) ||
    telemetry.reference_latitude_deg < -90.0 || telemetry.reference_latitude_deg > 90.0 ||
    telemetry.reference_longitude_deg < -180.0 || telemetry.reference_longitude_deg > 180.0))
  {
    RCLCPP_WARN(get_logger(), "Rejecting orbit goal: PX4 global reference or velocity is invalid");
    return false;
  }

  std::lock_guard<std::mutex> lock(control_mutex_);
  if (goal_active_) {
    RCLCPP_WARN(get_logger(), "Rejecting %s goal: another flight goal is active", goal_name);
    return false;
  }
  if ((kind == GoalKind::Takeoff && (telemetry.armed || !telemetry.landed)) ||
    (kind == GoalKind::Navigate && (!telemetry.armed || !telemetry.offboard)) ||
    (kind == GoalKind::FollowTrajectory &&
    (!telemetry.armed || !telemetry.offboard || telemetry.landed)) ||
    (kind == GoalKind::Orbit && (!telemetry.armed || !telemetry.offboard || telemetry.landed ||
    telemetry.in_transition_mode || telemetry.vehicle_type !=
    px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING)) ||
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
  reserved_start_yaw_rad_ = telemetry.yaw_enu_rad;
  look_at_position_enu_.reset();
  look_at_setpoint_updated_at_ = std::chrono::steady_clock::time_point{};
  target_yaw_rad_.reset();
  if (telemetry.offboard) {
    stream_mode_established_ = true;
  }
  RCLCPP_INFO(
    get_logger(), "Accepted ROS %s goal; PX4 state: armed=%s, offboard=%s, landed=%s, "
    "yaw_valid=%s, yaw=%.3f rad, position=[%.2f, %.2f, %.2f] m",
    goal_name, telemetry.armed ? "true" : "false", telemetry.offboard ? "true" : "false",
    telemetry.landed ? "true" : "false", telemetry.yaw_valid ? "true" : "false",
    telemetry.yaw_enu_rad, telemetry.position_enu[0], telemetry.position_enu[1],
    telemetry.position_enu[2]);
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

bool VehicleControlNode::has_same_local_reference(
  const VehicleTelemetry & current, const VehicleTelemetry & reference) const
{
  constexpr double latitude_tolerance_deg = 1.0e-8;
  constexpr double longitude_tolerance_deg = 1.0e-8;
  constexpr double altitude_tolerance_m = 0.01;
  return current.xy_global && current.z_global && reference.xy_global && reference.z_global &&
         current.reference_timestamp == reference.reference_timestamp &&
         current.xy_reset_counter == reference.xy_reset_counter &&
         current.z_reset_counter == reference.z_reset_counter &&
         std::abs(current.reference_latitude_deg - reference.reference_latitude_deg) <=
         latitude_tolerance_deg &&
         std::abs(current.reference_longitude_deg - reference.reference_longitude_deg) <=
         longitude_tolerance_deg &&
         std::abs(current.reference_altitude_amsl_m - reference.reference_altitude_amsl_m) <=
         altitude_tolerance_m;
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
  const float parameter_1, const float parameter_2, const float parameter_3,
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
      command, parameter_1, parameter_2, parameter_3, timestamp_us);
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
  target_yaw_rad_.reset();
  look_at_position_enu_.reset();
  look_at_setpoint_updated_at_ = std::chrono::steady_clock::time_point{};
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
  state.position_valid = has_fresh_position(telemetry);
  state.velocity_valid = telemetry.velocity_valid && state.position_valid;
  state.yaw_rad = static_cast<float>(telemetry.yaw_enu_rad);
  state.yaw_valid = telemetry.yaw_valid && state.position_valid;
  state.setpoint_publication_count = telemetry.setpoint_publication_count;
  const std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
  const std::chrono::steady_clock::duration status_age = current_time - telemetry.status_received;
  const std::chrono::steady_clock::duration land_age = current_time - telemetry.land_state_received;
  const std::chrono::duration<double> freshness(telemetry_timeout_sec_);
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
  const bool landed_and_disarmed = telemetry.has_land_state && telemetry.landed &&
    telemetry.has_status && !telemetry.armed;
  Position position{};
  std::optional<double> yaw_enu_rad;
  bool should_publish = false;
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    if (stream_requested_ && telemetry.offboard) {
      stream_mode_established_ = true;
    }
    if (landing_command_started_ && landed_and_disarmed) {
      stream_requested_ = false;
      stream_mode_established_ = false;
    }
    if (stream_requested_ && stream_mode_established_ && telemetry.has_status &&
      telemetry.offboard == false && has_fresh_control_telemetry(telemetry, false))
    {
      stream_requested_ = false;
      stream_mode_established_ = false;
    }
    should_publish = stream_requested_ || (telemetry.offboard && !landed_and_disarmed);
    position = target_position_;
    if (look_at_position_enu_.has_value() && has_fresh_position(telemetry) &&
      is_finite_position(telemetry.position_enu))
    {
      const std::optional<double> look_at_yaw_rad = yaw_toward_position(
        telemetry.position_enu, *look_at_position_enu_);
      if (look_at_yaw_rad.has_value() && target_yaw_rad_.has_value()) {
        const std::chrono::steady_clock::time_point current_time =
          std::chrono::steady_clock::now();
        const double elapsed_sec = std::max(0.0, std::chrono::duration<double>(
            current_time - look_at_setpoint_updated_at_).count());
        const double maximum_yaw_change_rad = maximum_look_at_yaw_rate_radps * elapsed_sec;
        const double requested_yaw_change_rad = angular_error(
          *look_at_yaw_rad, *target_yaw_rad_);
        const double applied_yaw_change_rad = std::clamp(
          requested_yaw_change_rad, -maximum_yaw_change_rad, maximum_yaw_change_rad);
        const double next_yaw_rad = *target_yaw_rad_ + applied_yaw_change_rad;
        target_yaw_rad_ = std::atan2(std::sin(next_yaw_rad), std::cos(next_yaw_rad));
        look_at_setpoint_updated_at_ = current_time;
      }
    }
    yaw_enu_rad = target_yaw_rad_;
  }
  if (should_publish) {
    const std::int64_t timestamp_us = get_clock()->now().nanoseconds() / 1000;
    px4_adapter_->publish_position_setpoint(position, timestamp_us, yaw_enu_rad);
  }
}

void VehicleControlNode::finish_goal()
{
  const VehicleTelemetry telemetry = px4_adapter_->telemetry();
  std::lock_guard<std::mutex> lock(control_mutex_);
  goal_active_ = false;
  landing_command_started_ = false;
  target_yaw_rad_.reset();
  look_at_position_enu_.reset();
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
  Position start_position{};
  RCLCPP_INFO(
    get_logger(), "Starting takeoff execution (target_altitude=%.2f m)",
    static_cast<double>(goal->altitude_m));
  try {
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      start_position = reserved_start_position_;
    }
    const std::chrono::steady_clock::time_point prestream_deadline =
      deadline_after(prestream_duration_sec);
    if (!wait_for_prestream(prestream_deadline, is_cancelled, failure_message)) {
      throw std::runtime_error(failure_message);
    }

    VehicleTelemetry mode_telemetry = px4_adapter_->telemetry();
    if (!has_fresh_control_telemetry(mode_telemetry, false)) {
      throw std::runtime_error("PX4 telemetry became stale after Offboard prestream");
    }
    if (mode_telemetry.offboard) {
      RCLCPP_INFO(get_logger(), "PX4 is already in Offboard after the one-second prestream");
      if (!wait_until(
          [](const VehicleTelemetry & current) {return current.offboard;},
          deadline_after(transition_timeout_sec_), is_cancelled, false, failure_message))
      {
        throw std::runtime_error("Offboard confirmation failed: " + failure_message);
      }
    } else if (!request_transition(
        "Offboard mode",
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        custom_mode_enabled, offboard_custom_mode, 0.0F,
        [](const VehicleTelemetry & current) {return current.offboard;},
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
        arm_command_value, 0.0F, 0.0F,
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
    const VehicleTelemetry telemetry = px4_adapter_->telemetry();
    const double climbed_m = telemetry.position_valid &&
      is_finite_position(telemetry.position_enu) ?
      telemetry.position_enu[2] - start_position[2] :
      std::numeric_limits<double>::quiet_NaN();
    std::ostringstream diagnostic;
    diagnostic << exception.what() << " (climbed=" << climbed_m
               << " m, requested_altitude=" << goal->altitude_m
               << " m, position_tolerance=" << default_position_tolerance_m_
               << " m, armed=" << (telemetry.armed ? "true" : "false")
               << ", offboard=" << (telemetry.offboard ? "true" : "false")
               << ", landed=" << (telemetry.landed ? "true" : "false")
               << ", nav_state=" << static_cast<unsigned int>(telemetry.navigation_state)
               << ')';
    result->message = diagnostic.str();
    const bool was_cancelled = is_cancelled();
    if (was_cancelled) {
      RCLCPP_WARN(get_logger(), "Takeoff canceled: %s", result->message.c_str());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Takeoff failed: %s", result->message.c_str());
      goal_handle->abort(result);
    }
    finish_goal();
  }
}

void VehicleControlNode::execute_navigation(const std::shared_ptr<NavigateHandle> & goal_handle)
{
  std::shared_ptr<NavigateTo::Result> result = std::make_shared<NavigateTo::Result>();
  result->final_distance_m = std::numeric_limits<float>::quiet_NaN();
  result->final_yaw_error_rad = std::numeric_limits<float>::quiet_NaN();
  const std::shared_ptr<const NavigateTo::Goal> goal = goal_handle->get_goal();
  const double tolerance = goal->position_tolerance_m == 0.0F ?
    default_position_tolerance_m_ : static_cast<double>(goal->position_tolerance_m);
  const double timeout = goal->timeout_sec == 0.0F ?
    default_navigation_timeout_sec_ : static_cast<double>(goal->timeout_sec);
  const Position target_position{
    goal->target_position.x, goal->target_position.y, goal->target_position.z};
  const Position look_at_position{
    goal->look_at_position.x, goal->look_at_position.y, goal->look_at_position.z};
  const std::chrono::steady_clock::time_point deadline = deadline_after(timeout);
  const std::function<bool()> is_cancelled =
    [goal_handle]() {return goal_handle->is_canceling() || !rclcpp::ok();};
  std::chrono::steady_clock::time_point last_feedback = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point yaw_settled_since{};
  std::string failure_message;
  RCLCPP_INFO(
    get_logger(), "Starting navigation (target_enu=[%.2f, %.2f, %.2f] m, tolerance=%.2f m, "
    "timeout=%.1f s)", target_position[0], target_position[1], target_position[2], tolerance,
    timeout);

  try {
    while (std::chrono::steady_clock::now() < deadline) {
      if (is_cancelled()) {
        result->success = false;
        result->message = "Navigation canceled; holding the current position";
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        const VehicleTelemetry telemetry = px4_adapter_->telemetry();
        if (telemetry.position_valid) {
          result->final_distance_m = static_cast<float>(
            distance(telemetry.position_enu, target_position));
        }
        if (goal->look_at_target && telemetry.position_valid && telemetry.yaw_valid &&
          is_finite_position(telemetry.position_enu) && std::isfinite(telemetry.yaw_enu_rad))
        {
          const std::optional<double> expected_yaw_rad = yaw_toward_position(
            telemetry.position_enu, look_at_position);
          if (expected_yaw_rad.has_value()) {
            result->final_yaw_error_rad = static_cast<float>(std::abs(angular_error(
                *expected_yaw_rad, telemetry.yaw_enu_rad)));
          }
        }
        hold_current_position();
        goal_handle->canceled(result);
        finish_goal();
        return;
      }
      const VehicleTelemetry telemetry = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(telemetry, false) ||
        (goal->look_at_target && (!telemetry.yaw_valid ||
        !std::isfinite(telemetry.yaw_enu_rad))))
      {
        failure_message = "PX4 telemetry became stale or invalid during navigation";
        throw std::runtime_error(failure_message);
      }
      const double target_distance = distance(telemetry.position_enu, target_position);
      result->final_distance_m = static_cast<float>(target_distance);
      bool is_yaw_settled = !goal->look_at_target;
      if (goal->look_at_target) {
        const std::optional<double> expected_yaw_rad = yaw_toward_position(
          telemetry.position_enu, look_at_position);
        double target_yaw_rad = telemetry.yaw_enu_rad;
        if (expected_yaw_rad.has_value()) {
          target_yaw_rad = *expected_yaw_rad;
        } else {
          std::lock_guard<std::mutex> lock(control_mutex_);
          if (target_yaw_rad_.has_value()) {
            target_yaw_rad = *target_yaw_rad_;
          }
        }
        const double yaw_error_rad = std::abs(angular_error(
            target_yaw_rad, telemetry.yaw_enu_rad));
        result->final_yaw_error_rad = static_cast<float>(yaw_error_rad);
        is_yaw_settled = target_distance <= tolerance &&
          yaw_error_rad <= default_yaw_tolerance_rad_;
        if (is_yaw_settled) {
          if (yaw_settled_since == std::chrono::steady_clock::time_point{}) {
            yaw_settled_since = std::chrono::steady_clock::now();
          } else if (std::chrono::steady_clock::now() - yaw_settled_since >=
            std::chrono::duration<double>(minimum_navigation_yaw_settle_duration_sec))
          {
            result->success = true;
            result->message = "Navigation target reached and look-at yaw settled";
            RCLCPP_INFO(
              get_logger(), "Navigation succeeded (distance=%.2f m, yaw_error=%.2f rad)",
              target_distance, yaw_error_rad);
            goal_handle->succeed(result);
            finish_goal();
            return;
          }
        } else {
          yaw_settled_since = std::chrono::steady_clock::time_point{};
        }
      }
      if (!goal->look_at_target && target_distance <= tolerance) {
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
    result->success = false;
    const VehicleTelemetry telemetry = px4_adapter_->telemetry();
    double expected_yaw_rad = std::numeric_limits<double>::quiet_NaN();
    if (telemetry.position_valid && is_finite_position(telemetry.position_enu)) {
      result->final_distance_m = static_cast<float>(
        distance(telemetry.position_enu, target_position));
    }
    if (goal->look_at_target && telemetry.yaw_valid && std::isfinite(telemetry.yaw_enu_rad) &&
      telemetry.position_valid && is_finite_position(telemetry.position_enu))
    {
      const std::optional<double> target_yaw_rad = yaw_toward_position(
        telemetry.position_enu, look_at_position);
      if (target_yaw_rad.has_value()) {
        expected_yaw_rad = *target_yaw_rad;
        result->final_yaw_error_rad = static_cast<float>(std::abs(angular_error(
            *target_yaw_rad, telemetry.yaw_enu_rad)));
      }
    }
    std::ostringstream diagnostic;
    diagnostic << exception.what() << " (distance=" << result->final_distance_m
               << " m, position_tolerance=" << tolerance << " m, look_at="
               << (goal->look_at_target ? "true" : "false");
    if (goal->look_at_target) {
      diagnostic << ", yaw_valid=" << (telemetry.yaw_valid ? "true" : "false")
                 << ", target_yaw=" << expected_yaw_rad << " rad"
                 << ", measured_yaw=" << telemetry.yaw_enu_rad
                 << " rad, yaw_error=" << result->final_yaw_error_rad
                 << " rad, yaw_tolerance=" << default_yaw_tolerance_rad_ << " rad";
    }
    diagnostic << ')';
    result->message = diagnostic.str();
    hold_current_position();
    const bool was_cancelled = is_cancelled();
    if (was_cancelled) {
      RCLCPP_WARN(get_logger(), "Navigation canceled: %s", result->message.c_str());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Navigation failed: %s", result->message.c_str());
      goal_handle->abort(result);
    }
    finish_goal();
  }
}

void VehicleControlNode::execute_follow_trajectory(
  const std::shared_ptr<FollowTrajectoryHandle> & goal_handle)
{
  std::shared_ptr<FollowTrajectory::Result> result =
    std::make_shared<FollowTrajectory::Result>();
  result->final_distance_m = std::numeric_limits<float>::quiet_NaN();
  result->final_yaw_error_rad = std::numeric_limits<float>::quiet_NaN();
  const std::shared_ptr<const FollowTrajectory::Goal> goal = goal_handle->get_goal();
  const double position_tolerance_m = goal->position_tolerance_m == 0.0F ?
    default_position_tolerance_m_ : static_cast<double>(goal->position_tolerance_m);
  const double yaw_tolerance_rad = goal->yaw_tolerance_rad == 0.0F ?
    default_yaw_tolerance_rad_ : static_cast<double>(goal->yaw_tolerance_rad);
  const double timeout_sec = goal->timeout_sec == 0.0F ?
    default_navigation_timeout_sec_ : static_cast<double>(goal->timeout_sec);
  const double trajectory_duration_sec =
    static_cast<double>(goal->points.back().time_from_start_sec);
  const std::chrono::steady_clock::time_point started_at = std::chrono::steady_clock::now();
  const std::chrono::steady_clock::time_point deadline = deadline_after(timeout_sec);
  const std::function<bool()> is_cancelled =
    [goal_handle]() {return goal_handle->is_canceling() || !rclcpp::ok();};
  std::chrono::steady_clock::time_point last_feedback = started_at;
  std::chrono::steady_clock::time_point stable_since{};
  std::string failure_message;

  RCLCPP_INFO(
    get_logger(), "Starting FollowTrajectory (points=%zu, duration=%.1f s, "
    "position_tolerance=%.2f m, control_yaw=%s, yaw_tolerance=%.2f rad, timeout=%.1f s)",
    goal->points.size(), trajectory_duration_sec, position_tolerance_m,
    goal->control_yaw ? "true" : "false", yaw_tolerance_rad, timeout_sec);

  try {
    VehicleTelemetry telemetry = px4_adapter_->telemetry();
    if (!has_fresh_control_telemetry(telemetry, false) || !telemetry.armed ||
      !telemetry.offboard || telemetry.landed ||
      (goal->control_yaw && (!telemetry.yaw_valid ||
      !std::isfinite(telemetry.yaw_enu_rad))))
    {
      throw std::runtime_error(
              "PX4 state changed before FollowTrajectory execution");
    }
    const Position first_position{
      goal->points.front().position.x,
      goal->points.front().position.y,
      goal->points.front().position.z};
    if (distance(telemetry.position_enu, first_position) > 1.0) {
      throw std::runtime_error(
              "vehicle moved more than 1 m from the first trajectory point before execution");
    }

    while (std::chrono::steady_clock::now() < deadline) {
      if (is_cancelled()) {
        hold_current_position();
        result->success = false;
        result->message = "Trajectory canceled; holding current position";
        telemetry = px4_adapter_->telemetry();
        if (telemetry.position_valid && is_finite_position(telemetry.position_enu)) {
          const Position final_position{
            goal->points.back().position.x,
            goal->points.back().position.y,
            goal->points.back().position.z};
          result->final_distance_m = static_cast<float>(
            distance(telemetry.position_enu, final_position));
        }
        if (goal->control_yaw && telemetry.yaw_valid && std::isfinite(telemetry.yaw_enu_rad)) {
          const TrajectorySample last_target =
            interpolate_trajectory(goal->points, trajectory_duration_sec);
          result->final_yaw_error_rad = static_cast<float>(
            std::abs(angular_error(last_target.yaw_enu_rad, telemetry.yaw_enu_rad)));
        }
        goal_handle->canceled(result);
        finish_goal();
        return;
      }

      telemetry = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(telemetry, false) || !telemetry.armed ||
        !telemetry.offboard || telemetry.landed ||
        (goal->control_yaw && (!telemetry.yaw_valid ||
        !std::isfinite(telemetry.yaw_enu_rad))))
      {
        throw std::runtime_error(
                "PX4 left armed Offboard flight or telemetry became invalid during FollowTrajectory");
      }

      const std::chrono::steady_clock::time_point current_time =
        std::chrono::steady_clock::now();
      const double elapsed_sec = std::chrono::duration<double>(current_time - started_at).count();
      const TrajectorySample target = interpolate_trajectory(goal->points, elapsed_sec);
      {
        std::lock_guard<std::mutex> lock(control_mutex_);
        target_position_ = target.position_enu;
        if (goal->control_yaw) {
          target_yaw_rad_ = target.yaw_enu_rad;
        }
      }

      const double position_error_m = distance(telemetry.position_enu, target.position_enu);
      const double yaw_error_rad = goal->control_yaw ?
        std::abs(angular_error(target.yaw_enu_rad, telemetry.yaw_enu_rad)) :
        std::numeric_limits<double>::quiet_NaN();
      const bool at_final_point = elapsed_sec >= trajectory_duration_sec;
      if (at_final_point && position_error_m <= position_tolerance_m &&
        (!goal->control_yaw || yaw_error_rad <= yaw_tolerance_rad))
      {
        if (stable_since == std::chrono::steady_clock::time_point{}) {
          stable_since = current_time;
        } else if (current_time - stable_since >= std::chrono::duration<double>(
          minimum_trajectory_settle_duration_sec))
        {
          result->success = true;
          result->final_distance_m = static_cast<float>(position_error_m);
          if (goal->control_yaw) {
            result->final_yaw_error_rad = static_cast<float>(yaw_error_rad);
          }
          result->message = goal->control_yaw ?
            "Timed trajectory completed and final position and yaw settled" :
            "Timed trajectory completed and final position settled";
          RCLCPP_INFO(
            get_logger(), "FollowTrajectory succeeded (distance=%.2f m, yaw_controlled=%s)",
            position_error_m, goal->control_yaw ? "true" : "false");
          goal_handle->succeed(result);
          finish_goal();
          return;
        }
      } else {
        stable_since = std::chrono::steady_clock::time_point{};
      }

      if (current_time - last_feedback >= feedback_interval) {
        std::shared_ptr<FollowTrajectory::Feedback> feedback =
          std::make_shared<FollowTrajectory::Feedback>();
        feedback->elapsed_sec = static_cast<float>(elapsed_sec);
        feedback->distance_to_target_m = static_cast<float>(position_error_m);
        feedback->yaw_error_rad = static_cast<float>(yaw_error_rad);
        feedback->status = at_final_point ? "settling" : "tracking";
        goal_handle->publish_feedback(feedback);
        last_feedback = current_time;
      }
      std::this_thread::sleep_for(worker_poll_interval);
    }
    failure_message = goal->control_yaw ?
      "trajectory timed out before final position and yaw settled" :
      "trajectory timed out before final position settled";
    throw std::runtime_error(failure_message);
  } catch (const std::exception & exception) {
    hold_current_position();
    result->success = false;
    const VehicleTelemetry telemetry = px4_adapter_->telemetry();
    if (telemetry.position_valid && is_finite_position(telemetry.position_enu)) {
      const Position final_position{
        goal->points.back().position.x,
        goal->points.back().position.y,
        goal->points.back().position.z};
      result->final_distance_m = static_cast<float>(
        distance(telemetry.position_enu, final_position));
    }
    if (goal->control_yaw && telemetry.yaw_valid && std::isfinite(telemetry.yaw_enu_rad)) {
      const double final_yaw_rad = static_cast<double>(goal->points.back().yaw_rad);
      result->final_yaw_error_rad = static_cast<float>(
        std::abs(angular_error(final_yaw_rad, telemetry.yaw_enu_rad)));
    }
    std::ostringstream diagnostic;
    diagnostic << exception.what() << " (final_distance=" << result->final_distance_m
               << " m, position_tolerance=" << position_tolerance_m
               << " m, control_yaw=" << (goal->control_yaw ? "true" : "false");
    if (goal->control_yaw) {
      diagnostic << ", yaw_valid=" << (telemetry.yaw_valid ? "true" : "false")
                 << ", target_yaw=" << goal->points.back().yaw_rad
                 << " rad, measured_yaw=" << telemetry.yaw_enu_rad
                 << " rad, yaw_error=" << result->final_yaw_error_rad
                 << " rad, yaw_tolerance=" << yaw_tolerance_rad << " rad";
    }
    diagnostic << ')';
    result->message = diagnostic.str();
    if (is_cancelled()) {
      RCLCPP_WARN(get_logger(), "FollowTrajectory canceled: %s", result->message.c_str());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "FollowTrajectory failed: %s", result->message.c_str());
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
  const std::function<bool()> is_shutdown = []() {return !rclcpp::ok();};
  std::string failure_message;
  RCLCPP_INFO(get_logger(), "Starting landing action (timeout=%.1f s)", timeout);

  try {
    VehicleTelemetry telemetry = px4_adapter_->telemetry();
    if (is_cancelled()) {
      failure_message = "goal canceled";
      throw std::runtime_error(failure_message);
    }
    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      landing_command_started_ = true;
    }
    if (!telemetry.landed) {
      const bool already_in_auto_land =
        telemetry.navigation_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
      if (!already_in_auto_land && !request_transition(
          "Auto Land mode",
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0F, 0.0F, 0.0F,
          [](const VehicleTelemetry & current) {
            return current.navigation_state ==
                   px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
          }, is_cancelled, failure_message))
      {
        throw std::runtime_error(failure_message);
      }

      while (std::chrono::steady_clock::now() < deadline) {
        if (is_cancelled()) {
          failure_message = "goal canceled during landing";
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
          break;
        }
        std::this_thread::sleep_for(feedback_interval);
      }
      if (!telemetry.landed) {
        failure_message = "landing timed out before PX4 reported landed";
        throw std::runtime_error(failure_message);
      }
    }

    telemetry = px4_adapter_->telemetry();
    if (!has_fresh_control_telemetry(telemetry, true) || !telemetry.landed) {
      throw std::runtime_error("PX4 landing confirmation became stale");
    }
    if (telemetry.armed && !request_transition(
        "disarming after landing",
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        disarm_command_value, 0.0F, 0.0F,
        [](const VehicleTelemetry & current) {return !current.armed;},
        is_shutdown, failure_message))
    {
      throw std::runtime_error(failure_message);
    }

    telemetry = px4_adapter_->telemetry();
    const bool already_in_auto_loiter = telemetry.navigation_state ==
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
    if (!already_in_auto_loiter && !request_transition(
        "Auto Loiter hold",
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        custom_mode_enabled, auto_custom_mode, auto_loiter_sub_mode,
        [](const VehicleTelemetry & current) {
          return current.navigation_state ==
                 px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
        }, is_shutdown, failure_message))
    {
      throw std::runtime_error(failure_message);
    }

    if (!wait_until(
        [](const VehicleTelemetry & current) {
          return current.landed && !current.armed && !current.offboard &&
                 current.navigation_state ==
                 px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
        }, deadline_after(transition_timeout_sec_), is_shutdown, true, failure_message))
    {
      throw std::runtime_error("PX4 did not confirm landed, disarmed Auto Loiter: " +
        failure_message);
    }
    result->success = true;
    result->landed = true;
    result->message = "PX4 reported landed, disarmed, and in Auto Loiter";
    RCLCPP_INFO(get_logger(), "Landing succeeded: landed, disarmed, and in Auto Loiter");
    goal_handle->succeed(result);
    finish_goal();
  } catch (const std::exception & exception) {
    result->success = false;
    const VehicleTelemetry telemetry = px4_adapter_->telemetry();
    result->landed = telemetry.landed;
    const double altitude_m = telemetry.position_valid &&
      is_finite_position(telemetry.position_enu) ? telemetry.position_enu[2] :
      std::numeric_limits<double>::quiet_NaN();
    std::ostringstream diagnostic;
    diagnostic << exception.what() << " (armed=" << (telemetry.armed ? "true" : "false")
               << ", offboard=" << (telemetry.offboard ? "true" : "false")
               << ", landed=" << (telemetry.landed ? "true" : "false")
               << ", auto_loiter=" << (telemetry.navigation_state ==
      px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER ? "true" : "false")
               << ", altitude=" << altitude_m
               << " m, position_valid=" << (telemetry.position_valid ? "true" : "false")
               << ", nav_state=" << static_cast<unsigned int>(telemetry.navigation_state)
               << ", timeout=" << timeout << " s)";
    result->message = diagnostic.str();
    const bool was_cancelled = is_cancelled();
    if (was_cancelled) {
      RCLCPP_WARN(get_logger(), "Landing canceled: %s", result->message.c_str());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Landing failed: %s", result->message.c_str());
      goal_handle->abort(result);
    }
    finish_goal();
  }
}

bool VehicleControlNode::restore_offboard_after_orbit(std::string & failure_message)
{
  const std::function<bool()> is_shutdown = []() {return !rclcpp::ok();};
  VehicleTelemetry telemetry = px4_adapter_->telemetry();
  if (!has_fresh_control_telemetry(telemetry, false) || !telemetry.armed || telemetry.landed) {
    failure_message = "cannot restore Offboard hold because PX4 telemetry is stale or vehicle is not airborne";
    return false;
  }

  if (!telemetry.offboard && telemetry.navigation_state !=
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT)
  {
    failure_message = "PX4 exited Orbit into navigation state " +
      std::to_string(telemetry.navigation_state) + "; leaving that mode in control";
    return false;
  }

  Position hold_position = telemetry.position_enu;
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    target_position_ = hold_position;
    stream_requested_ = true;
    stream_mode_established_ = telemetry.offboard;
  }

  if (!telemetry.offboard) {
    RCLCPP_INFO(get_logger(), "Orbit ending; prestreaming Offboard hold at current position");
    if (!wait_for_prestream(
        deadline_after(prestream_duration_sec), is_shutdown, failure_message))
    {
      failure_message = "Offboard recovery prestream failed: " + failure_message;
      return false;
    }
    if (!request_transition(
        "Offboard recovery", px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        custom_mode_enabled, offboard_custom_mode, 0.0F,
        [](const VehicleTelemetry & current) {return current.offboard;},
        is_shutdown, failure_message))
    {
      failure_message = "Offboard recovery failed: " + failure_message;
      return false;
    }
    std::lock_guard<std::mutex> lock(control_mutex_);
    stream_mode_established_ = true;
  }

  const std::chrono::steady_clock::time_point settle_deadline = deadline_after(15.0);
  std::chrono::steady_clock::time_point stable_since{};
  while (std::chrono::steady_clock::now() < settle_deadline) {
    if (is_shutdown()) {
      failure_message = "ROS shutdown interrupted Offboard hold confirmation";
      return false;
    }
    telemetry = px4_adapter_->telemetry();
    if (!has_fresh_control_telemetry(telemetry, false) || !telemetry.armed ||
      !telemetry.offboard)
    {
      failure_message = "PX4 left Offboard or telemetry became invalid before hold was confirmed";
      return false;
    }
    const double position_error = distance(telemetry.position_enu, hold_position);
    const bool is_stable = position_error <= default_position_tolerance_m_ &&
      telemetry.velocity_valid && horizontal_speed(telemetry.velocity_enu) <=
      orbit_hold_velocity_tolerance_mps &&
      std::abs(telemetry.velocity_enu[2]) <= orbit_hold_velocity_tolerance_mps;
    if (is_stable) {
      if (stable_since == std::chrono::steady_clock::time_point{}) {
        stable_since = std::chrono::steady_clock::now();
      } else if (std::chrono::steady_clock::now() - stable_since >= orbit_hold_settle_duration) {
        RCLCPP_INFO(
          get_logger(), "Offboard position hold confirmed (position_error=%.2f m)",
          position_error);
        return true;
      }
    } else {
      stable_since = std::chrono::steady_clock::time_point{};
    }
    std::this_thread::sleep_for(worker_poll_interval);
  }
  failure_message = "Offboard mode was entered but position and velocity did not settle at hold";
  return false;
}

void VehicleControlNode::execute_orbit(const std::shared_ptr<OrbitHandle> & goal_handle)
{
  std::shared_ptr<OrbitPoint::Result> result = std::make_shared<OrbitPoint::Result>();
  result->success = false;
  result->completed_revolutions = 0.0F;
  const std::shared_ptr<const OrbitPoint::Goal> goal = goal_handle->get_goal();
  const Position center_position{
    goal->center_position.x, goal->center_position.y, goal->center_position.z};
  const double radius_m = static_cast<double>(goal->radius_m);
  const double timeout_sec = static_cast<double>(goal->timeout_sec);
  const double yaw_tolerance_rad = goal->yaw_tolerance_rad == 0.0F ?
    default_orbit_yaw_tolerance_rad : static_cast<double>(goal->yaw_tolerance_rad);
  const double radial_tolerance_m = std::max(
    orbit_radial_tolerance_floor_m, radius_m * orbit_relative_radial_tolerance);
  OrbitProgress orbit_progress(
    center_position, radius_m, goal->clockwise, radial_tolerance_m,
    orbit_altitude_tolerance_m);
  const std::chrono::steady_clock::time_point goal_deadline = deadline_after(timeout_sec);
  const std::function<bool()> is_cancelled =
    [goal_handle]() {return goal_handle->is_canceling() || !rclcpp::ok();};
  std::string failure_message;
  bool recovery_attempted = false;
  bool recovery_confirmed = false;
  double completed_revolutions = 0.0;
  double radial_error_m = std::numeric_limits<double>::quiet_NaN();
  double yaw_error_rad = std::numeric_limits<double>::quiet_NaN();
  std::chrono::steady_clock::time_point yaw_outside_tolerance_since{};
  std::string feedback_status = "entering_orbit";

  RCLCPP_INFO(
    get_logger(), "Starting OrbitPoint (center_enu=[%.2f, %.2f, %.2f] m, radius=%.2f m, "
    "direction=%s, speed=%.2f m/s, revolutions=%.2f, timeout=%.1f s)",
    center_position[0], center_position[1], center_position[2], radius_m,
    goal->clockwise ? "clockwise" : "counterclockwise",
    static_cast<double>(goal->tangential_speed_mps), static_cast<double>(goal->revolutions),
    timeout_sec);
  std::shared_ptr<OrbitPoint::Feedback> initial_feedback =
    std::make_shared<OrbitPoint::Feedback>();
  initial_feedback->completed_revolutions = 0.0F;
  initial_feedback->radial_error_m = std::numeric_limits<float>::quiet_NaN();
  initial_feedback->yaw_error_rad = std::numeric_limits<float>::quiet_NaN();
  initial_feedback->status = feedback_status;
  goal_handle->publish_feedback(initial_feedback);

  try {
    VehicleTelemetry reference_telemetry = px4_adapter_->telemetry();
    if (!has_fresh_control_telemetry(reference_telemetry, true) ||
      !reference_telemetry.armed || !reference_telemetry.offboard ||
      reference_telemetry.landed || reference_telemetry.in_transition_mode ||
      (goal->face_center && (!reference_telemetry.yaw_valid ||
      !std::isfinite(reference_telemetry.yaw_enu_rad))) ||
      reference_telemetry.vehicle_type !=
      px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING ||
      !reference_telemetry.velocity_valid || !reference_telemetry.xy_global ||
      !reference_telemetry.z_global)
    {
      throw std::runtime_error("PX4 state changed before OrbitPoint execution");
    }
    const std::optional<GlobalPosition> global_center = enu_to_global(
      center_position, reference_telemetry.reference_latitude_deg,
      reference_telemetry.reference_longitude_deg,
      reference_telemetry.reference_altitude_amsl_m);
    if (!global_center) {
      throw std::runtime_error("Orbit center cannot be represented as a PX4 global position");
    }

    {
      std::lock_guard<std::mutex> lock(control_mutex_);
      target_position_ = reference_telemetry.position_enu;
      stream_requested_ = false;
      stream_mode_established_ = false;
    }

    if (is_cancelled()) {
      throw std::runtime_error("goal canceled before publishing the Orbit command");
    }
    const double signed_radius_m = goal->clockwise ? radius_m : -radius_m;
    const std::uint8_t yaw_behavior = goal->face_center ?
      orbit_yaw_face_center : orbit_yaw_uncontrolled;
    const std::uint64_t command_generation = px4_adapter_->publish_orbit_command(
      signed_radius_m, goal->tangential_speed_mps, yaw_behavior, *global_center,
      get_clock()->now().nanoseconds() / 1000);
    RCLCPP_INFO(
      get_logger(), "Published PX4 DO_ORBIT command (radius=%.2f m, center=[%.7f, %.7f, "
      "%.2f] WGS84)", signed_radius_m, global_center->latitude_deg,
      global_center->longitude_deg, global_center->altitude_amsl_m);

    const std::chrono::steady_clock::time_point acknowledgement_deadline =
      std::min(goal_deadline, deadline_after(command_ack_timeout_sec_));
    if (!wait_for_command_acknowledgement(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_ORBIT, command_generation,
        acknowledgement_deadline, is_cancelled, failure_message))
    {
      if (failure_message == "command acknowledgement timed out") {
        const VehicleTelemetry current = px4_adapter_->telemetry();
        if (!has_fresh_control_telemetry(current, false) || current.navigation_state !=
          px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT)
        {
          throw std::runtime_error(
                  "PX4 did not acknowledge DO_ORBIT and telemetry did not confirm Orbit mode");
        }
        RCLCPP_WARN(
          get_logger(), "PX4 did not acknowledge DO_ORBIT; telemetry confirms Orbit mode, "
          "so the command will not be resent");
      } else {
        throw std::runtime_error(failure_message);
      }
    }

    const std::function<bool(const VehicleTelemetry &)> is_orbit_mode =
      [](const VehicleTelemetry & current) {
        return current.navigation_state ==
               px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT;
      };
    const std::chrono::steady_clock::time_point mode_deadline =
      std::min(goal_deadline, deadline_after(transition_timeout_sec_));
    if (!wait_until(is_orbit_mode, mode_deadline, is_cancelled, false, failure_message)) {
      throw std::runtime_error(
              failure_message == "transition timed out" ?
              "PX4 acknowledged DO_ORBIT but did not enter Orbit mode" : failure_message);
    }
    RCLCPP_INFO(get_logger(), "PX4 confirmed Orbit navigation state");

    const std::chrono::steady_clock::time_point confirmation_deadline =
      std::min(goal_deadline, deadline_after(
        std::chrono::duration<double>(orbit_mode_confirmation_duration).count()));
    while (std::chrono::steady_clock::now() < confirmation_deadline) {
      if (is_cancelled()) {
        throw std::runtime_error("goal canceled while confirming PX4 Orbit mode");
      }
      const VehicleTelemetry current = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(current, false) || !current.armed ||
        current.navigation_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT ||
        !current.velocity_valid || !has_same_local_reference(current, reference_telemetry))
      {
        throw std::runtime_error(
                "PX4 Orbit mode was interrupted or local position reference changed");
      }
      std::this_thread::sleep_for(worker_poll_interval);
    }

    feedback_status = "approaching_circle";
    std::shared_ptr<OrbitPoint::Feedback> approach_feedback =
      std::make_shared<OrbitPoint::Feedback>();
    approach_feedback->completed_revolutions = 0.0F;
    approach_feedback->radial_error_m = std::numeric_limits<float>::quiet_NaN();
    approach_feedback->yaw_error_rad = std::numeric_limits<float>::quiet_NaN();
    approach_feedback->status = feedback_status;
    goal_handle->publish_feedback(approach_feedback);
    std::chrono::steady_clock::time_point last_feedback = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_position_received =
      reference_telemetry.position_received;
    RCLCPP_INFO(
      get_logger(), "Orbit mode stable; waiting for commanded radius %.2f m and altitude %.2f m",
      radius_m, center_position[2]);
    while (std::chrono::steady_clock::now() < goal_deadline) {
      if (is_cancelled()) {
        throw std::runtime_error("goal canceled during OrbitPoint");
      }
      const VehicleTelemetry current = px4_adapter_->telemetry();
      if (!has_fresh_control_telemetry(current, false) || !current.armed || current.landed ||
        current.navigation_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ORBIT ||
        current.in_transition_mode || !current.velocity_valid ||
        (goal->face_center && (!current.yaw_valid || !std::isfinite(current.yaw_enu_rad))))
      {
        throw std::runtime_error("PX4 Orbit mode was interrupted or telemetry became invalid");
      }
      if (!has_same_local_reference(current, reference_telemetry)) {
        throw std::runtime_error("PX4 local position reference or reset counter changed during Orbit");
      }
      if (current.position_received != last_position_received) {
        const OrbitProgressSample sample = orbit_progress.update(current.position_enu);
        last_position_received = current.position_received;
        radial_error_m = sample.radial_error_m;
        completed_revolutions = sample.completed_revolutions;
        feedback_status = sample.in_orbit_band ? "orbiting" : "approaching_circle";
        if (std::abs(sample.angular_step_rad) > orbit_maximum_sample_angle_rad) {
          throw std::runtime_error("PX4 position samples were discontinuous during Orbit");
        }
        if (goal->face_center && sample.in_orbit_band) {
          const std::optional<double> expected_yaw_rad = yaw_toward_position(
            current.position_enu, center_position);
          if (!expected_yaw_rad.has_value()) {
            throw std::runtime_error("Orbit center has no horizontal yaw bearing");
          }
          yaw_error_rad = std::abs(angular_error(
            *expected_yaw_rad, current.yaw_enu_rad));
          if (yaw_error_rad > yaw_tolerance_rad) {
            if (yaw_outside_tolerance_since == std::chrono::steady_clock::time_point{}) {
              yaw_outside_tolerance_since = std::chrono::steady_clock::now();
            } else if (std::chrono::steady_clock::now() - yaw_outside_tolerance_since >=
              orbit_yaw_error_duration)
            {
              std::ostringstream diagnostic;
              diagnostic << "Orbit yaw remained outside tolerance (target_yaw="
                         << *expected_yaw_rad << " rad, measured_yaw="
                         << current.yaw_enu_rad << " rad, error=" << yaw_error_rad
                         << " rad, tolerance=" << yaw_tolerance_rad << " rad, duration="
                         << orbit_yaw_error_duration.count() << " s)";
              throw std::runtime_error(diagnostic.str());
            }
          } else {
            yaw_outside_tolerance_since = std::chrono::steady_clock::time_point{};
          }
        } else {
          yaw_error_rad = std::numeric_limits<double>::quiet_NaN();
          yaw_outside_tolerance_since = std::chrono::steady_clock::time_point{};
        }
      }
      if (std::chrono::steady_clock::now() - last_feedback >= feedback_interval) {
        std::shared_ptr<OrbitPoint::Feedback> feedback =
          std::make_shared<OrbitPoint::Feedback>();
        feedback->completed_revolutions = static_cast<float>(completed_revolutions);
        feedback->radial_error_m = std::isfinite(radial_error_m) ?
          static_cast<float>(radial_error_m) : std::numeric_limits<float>::quiet_NaN();
        feedback->yaw_error_rad = static_cast<float>(yaw_error_rad);
        feedback->status = feedback_status;
        goal_handle->publish_feedback(feedback);
        last_feedback = std::chrono::steady_clock::now();
      }
      if (completed_revolutions >= static_cast<double>(goal->revolutions) &&
        yaw_outside_tolerance_since == std::chrono::steady_clock::time_point{})
      {
        recovery_attempted = true;
        feedback_status = "restoring_hold";
        RCLCPP_INFO(
          get_logger(), "Orbit target reached (completed_revolutions=%.2f); restoring Offboard",
          completed_revolutions);
        std::shared_ptr<OrbitPoint::Feedback> feedback =
          std::make_shared<OrbitPoint::Feedback>();
        feedback->completed_revolutions = static_cast<float>(completed_revolutions);
        feedback->radial_error_m = static_cast<float>(radial_error_m);
        feedback->yaw_error_rad = static_cast<float>(yaw_error_rad);
        feedback->status = feedback_status;
        goal_handle->publish_feedback(feedback);
        if (!restore_offboard_after_orbit(failure_message)) {
          throw std::runtime_error(
                  "Orbit completed but Offboard hold was not confirmed: " + failure_message);
        }
        recovery_confirmed = true;
        if (is_cancelled()) {
          result->success = false;
          result->completed_revolutions = static_cast<float>(completed_revolutions);
          result->message = "Orbit canceled after Offboard position hold was confirmed";
          RCLCPP_WARN(
            get_logger(), "OrbitPoint canceled during hold recovery "
            "(completed_revolutions=%.2f)", completed_revolutions);
          goal_handle->canceled(result);
          finish_goal();
          return;
        }
        result->success = true;
        result->completed_revolutions = static_cast<float>(completed_revolutions);
        result->message = "Orbit completed and Offboard position hold confirmed";
        RCLCPP_INFO(
          get_logger(), "OrbitPoint succeeded (completed_revolutions=%.2f)",
          completed_revolutions);
        goal_handle->succeed(result);
        finish_goal();
        return;
      }
      std::this_thread::sleep_for(worker_poll_interval);
    }
    throw std::runtime_error("OrbitPoint timed out before completing the requested revolutions");
  } catch (const std::exception & exception) {
    failure_message = exception.what();
    bool cancellation_requested = is_cancelled();
    if (!recovery_attempted) {
      recovery_attempted = true;
      std::string recovery_failure;
      recovery_confirmed = restore_offboard_after_orbit(recovery_failure);
      if (!recovery_confirmed) {
        failure_message += "; " + recovery_failure;
      }
    }
    cancellation_requested = cancellation_requested || is_cancelled();
    result->success = false;
    result->completed_revolutions = static_cast<float>(completed_revolutions);
    const VehicleTelemetry telemetry = px4_adapter_->telemetry();
    std::ostringstream diagnostic;
    diagnostic << failure_message << " (completed_revolutions=" << completed_revolutions
               << ", radial_error=" << radial_error_m << " m, face_center="
               << (goal->face_center ? "true" : "false")
               << ", yaw_valid=" << (telemetry.yaw_valid ? "true" : "false")
               << ", measured_yaw=" << telemetry.yaw_enu_rad
               << " rad, nav_state=" << static_cast<unsigned int>(telemetry.navigation_state)
               << ')';
    result->message = diagnostic.str();
    if (cancellation_requested && recovery_confirmed) {
      RCLCPP_WARN(
        get_logger(), "OrbitPoint canceled after hold recovery (completed_revolutions=%.2f): %s",
        completed_revolutions, result->message.c_str());
      goal_handle->canceled(result);
    } else {
      RCLCPP_ERROR(
        get_logger(), "OrbitPoint failed (completed_revolutions=%.2f): %s",
        completed_revolutions, result->message.c_str());
      goal_handle->abort(result);
    }
    finish_goal();
  }
}

}  // namespace vehicle_control
