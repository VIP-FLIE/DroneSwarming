// ===========================================================================
// main.cpp -- the entry point for `ros2 run cpp_single_hop hop`.
//
// Nothing here but main(). The node itself is in hop.cpp.
// ===========================================================================

#include <exception>
#include <memory>

#include "cpp_single_hop/hop.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // make_shared, and rclcpp owns it from here. There is no new or delete in
    // this program and nothing to free by hand.
    rclcpp::spin(std::make_shared<cpp_single_hop::Hop>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hop"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
