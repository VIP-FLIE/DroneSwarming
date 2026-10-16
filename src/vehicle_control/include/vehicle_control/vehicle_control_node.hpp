// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "vehicle_control/px4_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <swarm_interfaces/action/land.hpp>
#include <swarm_interfaces/action/navigate_to.hpp>
#include <swarm_interfaces/action/takeoff.hpp>
#include <swarm_interfaces/msg/vehicle_state.hpp>

namespace vehicle_control
{

/// Exposes vehicle actions and state while managing the PX4 Offboard stream.
class VehicleControlNode : public rclcpp::Node
{
public:
  /// Declares parameters and creates the vehicle action and telemetry interfaces.
  VehicleControlNode();
  /// Stops and joins the flight-action worker thread.
  ~VehicleControlNode() override;

private:
  using Takeoff = swarm_interfaces::action::Takeoff;
  using NavigateTo = swarm_interfaces::action::NavigateTo;
  using Land = swarm_interfaces::action::Land;
  using TakeoffHandle = rclcpp_action::ServerGoalHandle<Takeoff>;
  using NavigateHandle = rclcpp_action::ServerGoalHandle<NavigateTo>;
  using LandHandle = rclcpp_action::ServerGoalHandle<Land>;

  enum class GoalKind : std::uint8_t
  {
    None,
    Takeoff,
    Navigate,
    Land
  };

  /// Reserves one valid flight goal when telemetry and vehicle state permit it.
  bool reserve_goal(GoalKind kind, bool is_goal_valid);
  /// Checks whether position telemetry is valid, finite, and recent.
  bool has_fresh_position(const VehicleTelemetry & telemetry) const;
  /// Checks fresh control telemetry and optional landing-state telemetry.
  bool has_fresh_control_telemetry(
    const VehicleTelemetry & telemetry, bool require_land_state) const;
  /// Waits for a telemetry predicate while checking cancellation and deadline.
  bool wait_until(
    const std::function<bool(const VehicleTelemetry &)> & predicate,
    const std::chrono::steady_clock::time_point & deadline,
    const std::function<bool()> & is_cancelled,
    bool require_land_state,
    std::string & failure_message);
  /// Retries a PX4 command until its telemetry transition predicate succeeds.
  bool request_transition(
    std::uint32_t command, float parameter_1, float parameter_2,
    const std::function<bool(const VehicleTelemetry &)> & predicate,
    const std::function<bool()> & is_cancelled,
    std::string & failure_message);
  /// Waits for an accepted acknowledgment matching command and generation.
  bool wait_for_command_acknowledgement(
    std::uint32_t command, std::uint64_t generation,
    const std::chrono::steady_clock::time_point & deadline,
    const std::function<bool()> & is_cancelled,
    std::string & failure_message);
  /// Waits through prestream while checking telemetry freshness and cancellation.
  bool wait_for_prestream(
    const std::chrono::steady_clock::time_point & deadline,
    const std::function<bool()> & is_cancelled,
    std::string & failure_message);
  /// Sets the active target to the latest finite vehicle position.
  void hold_current_position();
  /// Publishes vehicle state with local ENU coordinates and freshness flags.
  void publish_state();
  /// Publishes position setpoints while requested or while PX4 reports Offboard.
  void publish_control_setpoint();
  /// Releases the active goal and clears stream state after PX4 exits Offboard.
  void finish_goal();
  /// Queues flight-action work for execution outside ROS callbacks.
  void enqueue(std::function<void()> job);
  /// Runs queued flight-action jobs until the node shuts down.
  void worker_loop();
  /// Runs takeoff through Offboard entry, arming, and reaching the target altitude.
  void execute_takeoff(const std::shared_ptr<TakeoffHandle> & goal_handle);
  /// Tracks navigation progress and holds position when navigation ends.
  void execute_navigation(const std::shared_ptr<NavigateHandle> & goal_handle);
  /// Requests PX4 landing and waits for its land detector confirmation.
  void execute_land(const std::shared_ptr<LandHandle> & goal_handle);

  std::shared_ptr<Px4Adapter> px4_adapter_;
  bool enable_flight_commands_{false};
  double setpoint_rate_hz_{20.0};
  double telemetry_timeout_sec_{1.0};
  double default_navigation_timeout_sec_{30.0};
  double default_position_tolerance_m_{0.5};
  double command_ack_timeout_sec_{2.0};
  double transition_timeout_sec_{5.0};
  double takeoff_timeout_sec_{30.0};
  std::uint8_t px4_system_id_{1};
  Position reserved_start_position_{};
  Position target_position_{};
  bool goal_active_{false};
  bool landing_command_started_{false};
  bool stream_requested_{false};
  bool stream_mode_established_{false};
  std::mutex control_mutex_;
  std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::deque<std::function<void()>> worker_jobs_;
  bool worker_stopping_{false};
  std::thread worker_thread_;

  rclcpp::Publisher<swarm_interfaces::msg::VehicleState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<NavigateTo>::SharedPtr navigation_server_;
  rclcpp_action::Server<Land>::SharedPtr land_server_;
};

}  // namespace vehicle_control
