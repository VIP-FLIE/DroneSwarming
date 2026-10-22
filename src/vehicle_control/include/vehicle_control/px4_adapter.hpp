// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "vehicle_control/coordinates.hpp"

#include <rclcpp/rclcpp.hpp>

namespace vehicle_control
{

/// Snapshot of PX4 telemetry decoded into application coordinates.
struct VehicleTelemetry
{
  Position position_enu{};
  Position velocity_enu{};
  bool position_valid{false};
  bool velocity_valid{false};
  bool armed{false};
  bool offboard{false};
  std::uint8_t navigation_state{0};
  bool landed{false};
  bool battery_available{false};
  float battery_percentage{0.0F};
  bool has_position{false};
  bool has_status{false};
  bool has_land_state{false};
  std::chrono::steady_clock::time_point position_received{};
  std::chrono::steady_clock::time_point status_received{};
  std::chrono::steady_clock::time_point land_state_received{};
};

/// PX4 command result paired with its receive generation.
struct CommandAcknowledgement
{
  std::uint32_t command{0};
  std::uint8_t result{0};
  std::uint8_t result_parameter_1{0};
  std::int32_t result_parameter_2{0};
  std::uint64_t generation{0};
};

/// Owns PX4 topic translation, telemetry decoding, and command publishing.
class Px4Adapter
{
public:
  /// Creates PX4 topic endpoints under topic_prefix for system_id.
  Px4Adapter(rclcpp::Node & node, std::string topic_prefix, std::uint8_t system_id);

  /// Returns a synchronized snapshot of the latest decoded telemetry.
  VehicleTelemetry telemetry() const;
  /// Publishes an ENU position target as a PX4 NED Offboard setpoint.
  void publish_position_setpoint(const Position & position_enu, std::int64_t timestamp_us);
  /// Publishes a PX4 command and returns the preceding acknowledgment generation.
  std::uint64_t publish_vehicle_command(
    std::uint32_t command, float parameter_1, float parameter_2, std::int64_t timestamp_us);
  /// Returns a matching acknowledgment received after generation, if available.
  std::optional<CommandAcknowledgement> acknowledgement_after(
    std::uint32_t command, std::uint64_t generation) const;

private:
  struct Implementation;
  std::shared_ptr<Implementation> implementation_;
};

/// Validates a PX4 topic prefix and normalizes it to one leading slash.
std::string normalize_px4_topic_prefix(const std::string & topic_prefix);

}  // namespace vehicle_control
