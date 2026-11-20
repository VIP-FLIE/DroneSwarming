// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/mission_raw/mission_raw.hpp>
#include <swarm_interfaces/action/fly_mission.hpp>

#include "vehicle_control/px4_adapter.hpp"

namespace vehicle_control
{

/// Uploaded items and the raw indexes corresponding to application waypoints.
struct OnboardPlan
{
  std::vector<mavsdk::MissionRaw::MissionItem> items;
  std::vector<std::int32_t> waypoint_indexes;
};

/// Validates bounded ENU inputs and projects one immutable mission into global items.
OnboardPlan build_onboard_plan(
  const swarm_interfaces::action::FlyMission::Goal & goal,
  const VehicleTelemetry & reference, double default_tolerance_m);

/// Owns the MAVLink mission protocol; flight-mode commands remain in the PX4 adapter.
class OnboardMission
{
public:
  /// Binds a MAVLink endpoint and selects only the configured PX4 system ID.
  OnboardMission(const std::string & connection_url, std::uint8_t system_id);
  /// Stops callbacks before destroying the SDK connection.
  ~OnboardMission();
  /// Uploads a replacement route before arming, with bounded discovery and cancellation.
  void upload(const OnboardPlan & plan, const std::function<bool()> & is_cancelled);
  /// Returns the largest observed current item for this upload, or -1 before progress.
  std::int32_t current_item() const;
  /// Returns whether the selected autopilot still has a MAVLink connection.
  bool is_connected() const;

private:
  mavsdk::Mavsdk sdk_;
  std::uint8_t system_id_;
  std::shared_ptr<mavsdk::System> system_;
  std::unique_ptr<mavsdk::MissionRaw> mission_;
  std::shared_ptr<std::atomic<std::int32_t>> current_item_;
  mavsdk::MissionRaw::MissionProgressHandle progress_handle_{};
};

}  // namespace vehicle_control
