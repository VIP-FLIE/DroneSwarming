// Copyright 2026 DroneSwarming maintainers.
#include <memory>

#include "mission_execution/mission_execution_node.hpp"

/// Starts the mission-execution node and its ROS executor.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<mission_execution::MissionExecutionNode> node =
    std::make_shared<mission_execution::MissionExecutionNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
