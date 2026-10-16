// Copyright 2026 DroneSwarming maintainers.
#include <memory>

#include "vehicle_control/vehicle_control_node.hpp"

/// Starts the vehicle-control node and its ROS executor.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<vehicle_control::VehicleControlNode> node =
    std::make_shared<vehicle_control::VehicleControlNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
