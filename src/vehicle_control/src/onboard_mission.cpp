// Copyright 2026 DroneSwarming maintainers.
#include "vehicle_control/onboard_mission.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <mavsdk/mavlink_include.hpp>
#include <mavsdk/system.hpp>

namespace vehicle_control
{
namespace
{
constexpr std::size_t maximum_waypoints = 256;
constexpr double maximum_distance_m = 1000.0;
constexpr double maximum_altitude_m = 120.0;
constexpr double maximum_tolerance_m = 10.0;
constexpr double start_position_tolerance_m = 1.0;
constexpr float waypoint_dwell_sec = 0.5F;
constexpr double command_timeout_sec = 1.0;
constexpr float maximum_waypoint_timeout_sec = 600.0F;
constexpr double coordinate_scale = 1.0e7;
constexpr std::chrono::milliseconds protocol_poll_interval(50);
constexpr std::chrono::seconds protocol_deadline(10);

/// Encodes a fixed geographic destination without consulting the estimator's local origin.
mavsdk::MissionRaw::MissionItem global_item(
  const swarm_interfaces::msg::GeographicPosition & position, const std::uint32_t command)
{
  if (!swarm_geometry::is_valid(swarm_geometry::from_message(position))) {
    throw std::invalid_argument("Mission geographic position or AMSL altitude is invalid");
  }
  mavsdk::MissionRaw::MissionItem item{};
  item.frame = MAV_FRAME_GLOBAL;
  item.command = command;
  item.autocontinue = 1;
  item.param4 = std::numeric_limits<float>::quiet_NaN();
  item.x = static_cast<std::int32_t>(std::llround(position.latitude_deg * coordinate_scale));
  item.y = static_cast<std::int32_t>(std::llround(position.longitude_deg * coordinate_scale));
  item.z = static_cast<float>(position.altitude_amsl_m);
  return item;
}

}  // namespace

OnboardPlan build_onboard_plan(
  const swarm_interfaces::action::FlyMission::Goal & goal,
  const VehicleTelemetry & reference, const double default_tolerance_m)
{
  const std::optional<GlobalPosition> current = global_position(reference);
  const GlobalPosition start = swarm_geometry::from_message(goal.starting_position);
  const std::optional<Position> start_offset = current ? global_to_enu(*current, start) : std::nullopt;
  const std::optional<Position> takeoff_offset = global_to_enu(
    swarm_geometry::from_message(goal.takeoff_position), start);
  if (!current || !start_offset || distance(*start_offset, {0.0, 0.0, 0.0}) > start_position_tolerance_m + swarm_geometry::projection_tolerance_m ||
    goal.waypoint_positions.size() > maximum_waypoints ||
    !std::isfinite(goal.position_tolerance_m) || goal.position_tolerance_m < 0.0F ||
    static_cast<double>(goal.position_tolerance_m) > maximum_tolerance_m ||
    !std::isfinite(default_tolerance_m) || default_tolerance_m <= 0.0 ||
    default_tolerance_m > maximum_tolerance_m ||
    !std::isfinite(goal.waypoint_timeout_sec) || goal.waypoint_timeout_sec <= 0.0F ||
    goal.waypoint_timeout_sec > maximum_waypoint_timeout_sec)
  {
    throw std::invalid_argument("Onboard mission requires a valid geographic start and bounded inputs");
  }
  if (!takeoff_offset || (*takeoff_offset)[2] <= 0.0 || (*takeoff_offset)[2] > maximum_altitude_m + swarm_geometry::projection_tolerance_m ||
    std::hypot((*takeoff_offset)[0], (*takeoff_offset)[1]) > start_position_tolerance_m + swarm_geometry::projection_tolerance_m)
  {
    throw std::invalid_argument("Mission takeoff must be a vertical climb of at most 120 m");
  }
  OnboardPlan plan;
  plan.items.push_back(global_item(goal.takeoff_position, MAV_CMD_NAV_TAKEOFF));
  if (goal.look_at_point_of_interest) {
    plan.items.push_back(global_item(goal.look_at_position, MAV_CMD_DO_SET_ROI_LOCATION));
  }
  for (const swarm_interfaces::msg::GeographicPosition & point : goal.waypoint_positions) {
    const std::optional<Position> offset = global_to_enu(swarm_geometry::from_message(point), start);
    if (!offset || (*offset)[2] <= 0.0 || (*offset)[2] > maximum_altitude_m + swarm_geometry::projection_tolerance_m ||
      std::hypot((*offset)[0], (*offset)[1]) > maximum_distance_m + swarm_geometry::projection_tolerance_m)
    {
      throw std::invalid_argument("Mission waypoint exceeds the 1000 m horizontal or 120 m vertical envelope");
    }
    mavsdk::MissionRaw::MissionItem item = global_item(point, MAV_CMD_NAV_WAYPOINT);
    item.param1 = waypoint_dwell_sec;
    item.param2 = goal.position_tolerance_m == 0.0F ?
      static_cast<float>(default_tolerance_m) : goal.position_tolerance_m;
    plan.waypoint_indexes.push_back(static_cast<std::int32_t>(plan.items.size()));
    plan.items.push_back(item);
  }
  if (goal.look_at_point_of_interest) {
    mavsdk::MissionRaw::MissionItem clear_roi{};
    clear_roi.frame = MAV_FRAME_MISSION;
    clear_roi.command = MAV_CMD_DO_SET_ROI_NONE;
    clear_roi.autocontinue = 1;
    plan.items.push_back(clear_roi);
  }
  if (goal.land_after_waypoints) {
    const swarm_interfaces::msg::GeographicPosition & last = goal.waypoint_positions.empty() ?
      goal.takeoff_position : goal.waypoint_positions.back();
    plan.items.push_back(global_item(last, MAV_CMD_NAV_LAND));
  }
  for (std::size_t index = 0; index < plan.items.size(); ++index) {
    plan.items[index].seq = static_cast<std::uint32_t>(index);
    plan.items[index].current = index == 0 ? 1U : 0U;
  }
  return plan;
}

OnboardMission::OnboardMission(const std::string & connection_url, const std::uint8_t system_id)
: sdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer}),
  system_id_(system_id), current_item_(std::make_shared<std::atomic<std::int32_t>>(-1))
{
  sdk_.set_timeout_s(command_timeout_sec);
  if (sdk_.add_any_connection(connection_url) != mavsdk::ConnectionResult::Success) {
    throw std::runtime_error("Cannot open mission MAVLink connection: " + connection_url);
  }
}

OnboardMission::~OnboardMission()
{
  if (mission_ && progress_handle_.valid()) {
    mission_->unsubscribe_mission_progress(progress_handle_);
  }
}

void OnboardMission::upload(const OnboardPlan & plan, const std::function<bool()> & is_cancelled)
{
  const std::chrono::steady_clock::time_point discovery_deadline =
    std::chrono::steady_clock::now() + protocol_deadline;
  while (!system_) {
    for (const std::shared_ptr<mavsdk::System> & candidate : sdk_.systems()) {
      if (candidate->get_system_id() == system_id_ && candidate->has_autopilot() && candidate->is_connected()) {
        system_ = candidate;
        mission_ = std::make_unique<mavsdk::MissionRaw>(system_);
        break;
      }
    }
    if (is_cancelled() || std::chrono::steady_clock::now() >= discovery_deadline) {
      throw std::runtime_error("Mission MAVLink discovery interrupted or timed out for configured system ID");
    }
    std::this_thread::sleep_for(protocol_poll_interval);
  }
  if (progress_handle_.valid()) {
    mission_->unsubscribe_mission_progress(progress_handle_);
    progress_handle_ = {};
  }
  current_item_ = std::make_shared<std::atomic<std::int32_t>>(-1);
  const std::shared_ptr<std::promise<mavsdk::MissionRaw::Result>> promise =
    std::make_shared<std::promise<mavsdk::MissionRaw::Result>>();
  std::future<mavsdk::MissionRaw::Result> result = promise->get_future();
  mission_->upload_mission_async(plan.items, [promise](const mavsdk::MissionRaw::Result outcome) {
    promise->set_value(outcome);
  });
  const std::chrono::steady_clock::time_point upload_deadline =
    std::chrono::steady_clock::now() + protocol_deadline;
  while (result.wait_for(protocol_poll_interval) != std::future_status::ready) {
    if (is_cancelled() || std::chrono::steady_clock::now() >= upload_deadline) {
      const mavsdk::MissionRaw::Result cancellation = mission_->cancel_mission_upload();
      std::ostringstream diagnostic;
      diagnostic << "Mission upload interrupted or timed out; upload cancellation: " << cancellation;
      throw std::runtime_error(diagnostic.str());
    }
  }
  const mavsdk::MissionRaw::Result outcome = result.get();
  if (outcome != mavsdk::MissionRaw::Result::Success) {
    std::ostringstream diagnostic;
    diagnostic << "Mission upload failed: " << outcome;
    throw std::runtime_error(diagnostic.str());
  }
  const mavsdk::MissionRaw::Result selection = mission_->set_current_mission_item(0);
  if (selection != mavsdk::MissionRaw::Result::Success) {
    throw std::runtime_error("Cannot select the first item of the uploaded mission");
  }
  const std::shared_ptr<std::atomic<std::int32_t>> progress = current_item_;
  const std::int32_t total = static_cast<std::int32_t>(plan.items.size());
  progress_handle_ = mission_->subscribe_mission_progress(
    [progress, total](const mavsdk::MissionRaw::MissionProgress sample) {
      if (sample.total == total && sample.current >= 0 && sample.current <= total) {
        const std::int32_t previous = progress->load();
        progress->store(std::max(previous, sample.current));
      }
    });
}

std::int32_t OnboardMission::current_item() const {return current_item_->load();}

bool OnboardMission::is_connected() const {return system_ && system_->is_connected();}

}  // namespace vehicle_control
