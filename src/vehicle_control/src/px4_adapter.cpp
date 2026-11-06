// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/px4_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace vehicle_control
{
namespace
{

/// Returns best-effort QoS settings for PX4 streaming topics.
rclcpp::QoS px4_stream_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
}

/// Returns reliable QoS settings for PX4 command topics.
rclcpp::QoS px4_command_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

}  // namespace

/// Owns PX4 endpoints and decodes subscribed messages into telemetry.
struct Px4Adapter::Implementation
{
  /// Creates publishers and subscriptions under the normalized PX4 prefix.
  Implementation(
    rclcpp::Node & node, const std::string & prefix,
    const std::uint8_t configured_system_id)
  : system_id(configured_system_id)
  {
    offboard_mode_publisher = node.create_publisher<px4_msgs::msg::OffboardControlMode>(
      prefix + "/in/offboard_control_mode", px4_stream_qos());
    setpoint_publisher = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      prefix + "/in/trajectory_setpoint", px4_stream_qos());
    command_publisher = node.create_publisher<px4_msgs::msg::VehicleCommand>(
      prefix + "/in/vehicle_command", px4_command_qos());

    local_position_subscription = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      prefix + "/out/vehicle_local_position_v1", px4_stream_qos(),
      [this](const px4_msgs::msg::VehicleLocalPosition & message) {
        const std::chrono::steady_clock::time_point received = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex);
        telemetry.position_enu = ned_to_enu({
          static_cast<double>(message.x), static_cast<double>(message.y),
          static_cast<double>(message.z)});
        telemetry.velocity_enu = ned_to_enu({
          static_cast<double>(message.vx), static_cast<double>(message.vy),
          static_cast<double>(message.vz)});
        telemetry.position_valid = message.xy_valid && message.z_valid &&
          std::isfinite(message.x) && std::isfinite(message.y) && std::isfinite(message.z);
        telemetry.velocity_valid = message.v_xy_valid && message.v_z_valid &&
          std::isfinite(message.vx) && std::isfinite(message.vy) && std::isfinite(message.vz);
        telemetry.yaw_valid = message.heading_good_for_control && std::isfinite(message.heading);
        if (telemetry.yaw_valid) {
          constexpr double half_pi = 1.57079632679489661923;
          telemetry.yaw_enu_rad = std::atan2(
            std::sin(half_pi - static_cast<double>(message.heading)),
            std::cos(half_pi - static_cast<double>(message.heading)));
        }
        telemetry.xy_global = message.xy_global;
        telemetry.z_global = message.z_global;
        telemetry.reference_latitude_deg = message.ref_lat;
        telemetry.reference_longitude_deg = message.ref_lon;
        telemetry.reference_altitude_amsl_m = message.ref_alt;
        telemetry.reference_timestamp = message.ref_timestamp;
        telemetry.xy_reset_counter = message.xy_reset_counter;
        telemetry.z_reset_counter = message.z_reset_counter;
        telemetry.has_position = true;
        telemetry.position_received = received;
      });

    status_subscription = node.create_subscription<px4_msgs::msg::VehicleStatus>(
      prefix + "/out/vehicle_status_v1", px4_stream_qos(),
      [this](const px4_msgs::msg::VehicleStatus & message) {
        std::lock_guard<std::mutex> lock(mutex);
        telemetry.armed = message.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        telemetry.offboard = message.nav_state ==
          px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        telemetry.in_transition_mode = message.in_transition_mode;
        telemetry.vehicle_type = message.vehicle_type;
        telemetry.navigation_state = message.nav_state;
        telemetry.has_status = true;
        telemetry.status_received = std::chrono::steady_clock::now();
      });

    land_state_subscription = node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
      prefix + "/out/vehicle_land_detected", px4_stream_qos(),
      [this](const px4_msgs::msg::VehicleLandDetected & message) {
        std::lock_guard<std::mutex> lock(mutex);
        telemetry.landed = message.landed;
        telemetry.has_land_state = true;
        telemetry.land_state_received = std::chrono::steady_clock::now();
      });

    battery_subscription = node.create_subscription<px4_msgs::msg::BatteryStatus>(
      prefix + "/out/battery_status_v1", px4_stream_qos(),
      [this](const px4_msgs::msg::BatteryStatus & message) {
        std::lock_guard<std::mutex> lock(mutex);
        telemetry.battery_available = std::isfinite(message.remaining) && message.remaining >= 0.0F;
        telemetry.battery_percentage = telemetry.battery_available ?
          std::clamp(message.remaining * 100.0F, 0.0F, 100.0F) : 0.0F;
      });

    acknowledgement_subscription = node.create_subscription<px4_msgs::msg::VehicleCommandAck>(
      prefix + "/out/vehicle_command_ack", px4_stream_qos(),
      [this](const px4_msgs::msg::VehicleCommandAck & message) {
       // PX4 v1.17 leaves this command-origin flag unset in acknowledgements.
        if (message.target_system != 255U || message.target_component != 1U)
        {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        ++acknowledgement_generation;
        last_acknowledgement = CommandAcknowledgement{
          message.command, message.result, message.result_param1, message.result_param2,
          acknowledgement_generation};
      });
  }

  /// Publishes the Offboard mode and a position setpoint in PX4 NED.
  void publish_position_setpoint(
    const Position & position_enu, const std::int64_t timestamp,
    const std::optional<double> yaw_enu_rad)
  {
    const Position position_ned = enu_to_ned(position_enu);
    px4_msgs::msg::OffboardControlMode control_mode{};
    control_mode.position = true;
    control_mode.timestamp = static_cast<std::uint64_t>(timestamp);
    offboard_mode_publisher->publish(control_mode);

    const float unset = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position = {
      static_cast<float>(position_ned[0]), static_cast<float>(position_ned[1]),
      static_cast<float>(position_ned[2])};
    setpoint.velocity = {unset, unset, unset};
    setpoint.acceleration = {unset, unset, unset};
    setpoint.jerk = {unset, unset, unset};
    if (yaw_enu_rad && std::isfinite(*yaw_enu_rad)) {
      constexpr double half_pi = 1.57079632679489661923;
      const double yaw_ned_rad = std::atan2(
        std::sin(half_pi - *yaw_enu_rad), std::cos(half_pi - *yaw_enu_rad));
      setpoint.yaw = static_cast<float>(yaw_ned_rad);
    } else {
      setpoint.yaw = unset;
    }
    setpoint.yawspeed = unset;
    setpoint.timestamp = static_cast<std::uint64_t>(timestamp);
    setpoint_publisher->publish(setpoint);
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++telemetry.setpoint_publication_count;
    }
  }

  /// Publishes one PX4 command and returns the prior acknowledgment generation.
  std::uint64_t publish_vehicle_command(
    const std::uint32_t command, const float parameter_1, const float parameter_2,
    const std::int64_t timestamp)
  {
    return publish_vehicle_command(
      command, parameter_1, parameter_2, 0.0F, timestamp);
  }

  /// Publishes a PX4 command with a custom sub-mode and returns its prior acknowledgment
  /// generation.
  std::uint64_t publish_vehicle_command(
    const std::uint32_t command, const float parameter_1, const float parameter_2,
    const float parameter_3, const std::int64_t timestamp)
  {
    return publish_vehicle_command(
      command, parameter_1, parameter_2, parameter_3, 0.0F, 0.0, 0.0, 0.0F, timestamp);
  }

  /// Publishes a full PX4 command and returns the preceding acknowledgment generation.
  std::uint64_t publish_vehicle_command(
    const std::uint32_t command, const float parameter_1, const float parameter_2,
    const float parameter_3, const float parameter_4, const double parameter_5,
    const double parameter_6, const float parameter_7, const std::int64_t timestamp)
  {
    std::uint64_t generation_before_publish = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      generation_before_publish = acknowledgement_generation;
    }

    px4_msgs::msg::VehicleCommand message{};
    message.timestamp = static_cast<std::uint64_t>(timestamp);
    message.param1 = parameter_1;
    message.param2 = parameter_2;
    message.param3 = parameter_3;
    message.param4 = parameter_4;
    message.param5 = parameter_5;
    message.param6 = parameter_6;
    message.param7 = parameter_7;
    message.command = command;
    message.target_system = system_id;
    message.target_component = 1;
    message.source_system = 255;
    message.source_component = 1;
    message.from_external = true;
    command_publisher->publish(message);
    return generation_before_publish;
  }

  VehicleTelemetry telemetry{};
  mutable std::mutex mutex;
  std::uint64_t acknowledgement_generation{0};
  std::optional<CommandAcknowledgement> last_acknowledgement;
  std::uint8_t system_id;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_state_subscription;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_subscription;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr acknowledgement_subscription;
};

Px4Adapter::Px4Adapter(
  rclcpp::Node & node, std::string topic_prefix, const std::uint8_t system_id)
: implementation_(std::make_shared<Implementation>(
    node, normalize_px4_topic_prefix(topic_prefix), system_id))
{
}

VehicleTelemetry Px4Adapter::telemetry() const
{
  std::lock_guard<std::mutex> lock(implementation_->mutex);
  return implementation_->telemetry;
}

void Px4Adapter::publish_position_setpoint(
  const Position & position_enu, const std::int64_t timestamp_us,
  const std::optional<double> yaw_enu_rad)
{
  implementation_->publish_position_setpoint(position_enu, timestamp_us, yaw_enu_rad);
}

std::uint64_t Px4Adapter::publish_orbit_command(
  const double signed_radius_m, const float tangential_speed_mps,
  const std::uint8_t yaw_behavior, const GlobalPosition & center,
  const std::int64_t timestamp_us)
{
  return implementation_->publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_ORBIT,
    static_cast<float>(signed_radius_m), tangential_speed_mps,
    static_cast<float>(yaw_behavior), std::numeric_limits<float>::quiet_NaN(),
    center.latitude_deg, center.longitude_deg,
    static_cast<float>(center.altitude_amsl_m), timestamp_us);
}

std::uint64_t Px4Adapter::publish_vehicle_command(
  const std::uint32_t command, const float parameter_1, const float parameter_2,
  const std::int64_t timestamp_us)
{
  return implementation_->publish_vehicle_command(command, parameter_1, parameter_2, timestamp_us);
}

std::uint64_t Px4Adapter::publish_vehicle_command(
  const std::uint32_t command, const float parameter_1, const float parameter_2,
  const float parameter_3, const std::int64_t timestamp_us)
{
  return implementation_->publish_vehicle_command(
    command, parameter_1, parameter_2, parameter_3, timestamp_us);
}

std::optional<CommandAcknowledgement> Px4Adapter::acknowledgement_after(
  const std::uint32_t command, const std::uint64_t generation) const
{
  std::lock_guard<std::mutex> lock(implementation_->mutex);
  if (implementation_->last_acknowledgement &&
    implementation_->last_acknowledgement->generation > generation &&
    implementation_->last_acknowledgement->command == command)
  {
    return implementation_->last_acknowledgement;
  }
  return std::nullopt;
}

std::string normalize_px4_topic_prefix(const std::string & topic_prefix)
{
  const std::size_t first_segment = topic_prefix.find_first_not_of('/');
  if (first_segment == std::string::npos) {
    throw std::invalid_argument("px4_topic_prefix must include at least one topic segment");
  }
  const std::size_t last_segment = topic_prefix.find_last_not_of('/');
  const std::string normalized = topic_prefix.substr(
    first_segment, last_segment - first_segment + 1);
  if (normalized.find("//") != std::string::npos ||
    normalized.find_first_of(" \t\n") != std::string::npos)
  {
    throw std::invalid_argument("px4_topic_prefix contains an invalid topic segment");
  }
  return "/" + normalized;
}

}  // namespace vehicle_control
