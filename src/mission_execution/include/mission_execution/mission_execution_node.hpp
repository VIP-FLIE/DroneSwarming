// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <swarm_interfaces/action/execute_waypoint_mission.hpp>
#include <swarm_interfaces/action/execute_flight_plan.hpp>
#include <swarm_interfaces/action/follow_trajectory.hpp>
#include <swarm_interfaces/action/land.hpp>
#include <swarm_interfaces/action/navigate_to.hpp>
#include <swarm_interfaces/action/orbit_point.hpp>
#include <swarm_interfaces/action/takeoff.hpp>
#include <swarm_interfaces/msg/vehicle_state.hpp>
#include <swarm_interfaces/srv/plan_flight.hpp>

namespace mission_execution
{

/// Owns application command admission, landing preemption, and vehicle action selection.
class MissionExecutionNode : public rclcpp::Node
{
public:
  /// Creates the mission action server, vehicle clients, and worker thread.
  explicit MissionExecutionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /// Stops and joins the mission worker thread.
  ~MissionExecutionNode() override;

private:
  using Mission = swarm_interfaces::action::ExecuteWaypointMission;
  using MissionHandle = rclcpp_action::ServerGoalHandle<Mission>;
  using FlightPlan = swarm_interfaces::action::ExecuteFlightPlan;
  using FlightPlanHandle = rclcpp_action::ServerGoalHandle<FlightPlan>;
  using PlanFlight = swarm_interfaces::srv::PlanFlight;

  /// Reserves one command or schedules landing ahead of the interrupted command.
  bool reserve_command(bool is_goal_valid, bool is_landing = false);
  /// Reports shutdown or a pending landing that interrupts ordinary flight work.
  bool command_is_interrupted();
  /// Returns a diagnostic when state cannot support the selected operation.
  std::string state_error(bool is_takeoff, bool is_landing, bool requires_yaw);
  /// Captures one finite position after waiting for fresh application telemetry.
  swarm_interfaces::msg::VehicleState starting_state(
    const std::function<bool()> & is_interrupted);
  /// Rejects a changed local coordinate frame while executing translated instructions.
  std::string frame_error(const swarm_interfaces::msg::VehicleState & captured_state);
  /// Sends JSON to the planning service and executes its validated selected assignment.
  void execute_flight_plan(const std::shared_ptr<FlightPlanHandle> & goal_handle);
  /// Creates a public operation endpoint with shared command admission.
  template<typename ActionT>
  typename rclcpp_action::Server<ActionT>::SharedPtr create_operation_server(
    const std::string & name,
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    bool is_landing = false);
  /// Checks state, dispatches one operation, and preserves child feedback and result fields.
  template<typename ActionT>
  void execute_operation(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client, bool is_landing);
  /// Queues admitted command work for execution outside ROS callbacks.
  void enqueue(std::function<void()> job, bool is_landing = false);
  /// Runs queued command jobs until the node shuts down.
  void worker_loop();
  /// Executes takeoff, mission-origin waypoint navigation, and optional landing.
  template<typename ActionT>
  void execute_mission(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    const std::shared_ptr<const Mission::Goal> & goal,
    const std::optional<swarm_interfaces::msg::VehicleState> & captured_state = std::nullopt);

  double telemetry_timeout_sec_{1.0};
  double starting_position_timeout_sec_{10.0};
  double action_server_timeout_sec_{10.0};
  double child_result_timeout_sec_{90.0};
  double default_waypoint_timeout_sec_{30.0};
  double maximum_plan_altitude_m_{30.0};
  double maximum_plan_distance_m_{100.0};
  std::string vehicle_id_;
  bool command_active_{false};
  bool landing_pending_{false};
  std::atomic<bool> stopping_{false};
  std::shared_ptr<std::atomic<bool>> child_pending_{std::make_shared<std::atomic<bool>>(false)};
  std::mutex command_mutex_;
  std::mutex state_mutex_;
  std::condition_variable state_condition_;
  swarm_interfaces::msg::VehicleState latest_vehicle_state_;
  bool has_vehicle_state_{false};
  std::chrono::steady_clock::time_point vehicle_state_received_{};
  std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::deque<std::function<void()>> worker_jobs_;
  bool worker_stopping_{false};
  std::thread worker_thread_;

  rclcpp_action::Server<Mission>::SharedPtr mission_server_;
  rclcpp_action::Server<FlightPlan>::SharedPtr flight_plan_server_;
  rclcpp::Client<PlanFlight>::SharedPtr planner_client_;
  rclcpp_action::Server<swarm_interfaces::action::Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<swarm_interfaces::action::NavigateTo>::SharedPtr navigation_server_;
  rclcpp_action::Server<swarm_interfaces::action::FollowTrajectory>::SharedPtr trajectory_server_;
  rclcpp_action::Server<swarm_interfaces::action::OrbitPoint>::SharedPtr orbit_server_;
  rclcpp_action::Server<swarm_interfaces::action::Land>::SharedPtr land_server_;
  rclcpp_action::Client<swarm_interfaces::action::Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<swarm_interfaces::action::NavigateTo>::SharedPtr navigation_client_;
  rclcpp_action::Client<swarm_interfaces::action::FollowTrajectory>::SharedPtr trajectory_client_;
  rclcpp_action::Client<swarm_interfaces::action::OrbitPoint>::SharedPtr orbit_client_;
  rclcpp_action::Client<swarm_interfaces::action::Land>::SharedPtr land_client_;
  rclcpp::Subscription<swarm_interfaces::msg::VehicleState>::SharedPtr state_subscription_;
};

}  // namespace mission_execution
