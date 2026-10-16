// Copyright 2026 DroneSwarming maintainers.
#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <swarm_interfaces/action/execute_waypoint_mission.hpp>
#include <swarm_interfaces/action/land.hpp>
#include <swarm_interfaces/action/navigate_to.hpp>
#include <swarm_interfaces/action/takeoff.hpp>
#include <swarm_interfaces/msg/vehicle_state.hpp>

namespace mission_execution
{

/// Sequences vehicle actions to execute a waypoint mission.
class MissionExecutionNode : public rclcpp::Node
{
public:
  /// Creates the mission action server, vehicle clients, and worker thread.
  MissionExecutionNode();
  /// Stops and joins the mission worker thread.
  ~MissionExecutionNode() override;

private:
  using Mission = swarm_interfaces::action::ExecuteWaypointMission;
  using MissionHandle = rclcpp_action::ServerGoalHandle<Mission>;

  /// Reserves the single mission slot when the incoming goal is valid.
  bool reserve_mission(bool is_goal_valid);
  /// Queues mission work for execution outside ROS callbacks.
  void enqueue(std::function<void()> job);
  /// Runs queued mission jobs until the node shuts down.
  void worker_loop();
  /// Executes takeoff, mission-origin waypoint navigation, and optional landing.
  void execute_mission(const std::shared_ptr<MissionHandle> & goal_handle);

  double telemetry_timeout_sec_{1.0};
  double starting_position_timeout_sec_{10.0};
  double action_server_timeout_sec_{10.0};
  double child_result_timeout_sec_{90.0};
  double default_waypoint_timeout_sec_{30.0};
  bool mission_active_{false};
  std::mutex mission_mutex_;
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
  rclcpp_action::Client<swarm_interfaces::action::Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<swarm_interfaces::action::NavigateTo>::SharedPtr navigation_client_;
  rclcpp_action::Client<swarm_interfaces::action::Land>::SharedPtr land_client_;
  rclcpp::Subscription<swarm_interfaces::msg::VehicleState>::SharedPtr state_subscription_;
};

}  // namespace mission_execution
