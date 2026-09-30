// ===========================================================================
// main.cpp -- the entry point for `ros2 run swarm_comms formation_drone`.
//
// Nothing here but main(). The node itself is in formation_drone.cpp.
// ===========================================================================

#include <exception>
#include <memory>

#include "swarm_comms/formation_drone.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<swarm_comms::FormationDrone>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("formation_drone"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
