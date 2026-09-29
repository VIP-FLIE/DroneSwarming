// ===========================================================================
// main.cpp -- the entry point for `ros2 run swarm_naive waypoint_drone`.
//
// Nothing here but main(). The node itself is in waypoint_drone.cpp.
// ===========================================================================

#include <exception>
#include <memory>

#include "swarm_naive/waypoint_drone.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // make_shared, and rclcpp owns it from here. There is no new or delete in
    // this file and nothing to free by hand.
    rclcpp::spin(std::make_shared<swarm_naive::WaypointDrone>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_drone"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
