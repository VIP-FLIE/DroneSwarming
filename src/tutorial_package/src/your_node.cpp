// your_node.cpp
// The minimal C++ ROS 2 node. Every line here is required; nothing is extra.
//
// Build and run:
//   colcon build --symlink-install --packages-select tutorial_package
//   source install/setup.bash
//   ros2 run tutorial_package your_node

// rclcpp is the C++ ROS 2 client library. It provides Node, spin, init, etc.
#include <rclcpp/rclcpp.hpp>

// A ROS 2 node is a class that inherits from rclcpp::Node.
// The constructor calls the parent with the node's name. That string is what
// shows up in `ros2 node list` and in log output: "[your_node]: ...".
class YourNode : public rclcpp::Node
{
public:
  YourNode() : Node("your_node")
  {
    // RCLCPP_INFO writes a timestamped message to the ROS 2 log.
    // get_logger() returns the logger tied to this node's name.
    // Other levels: RCLCPP_WARN, RCLCPP_ERROR, RCLCPP_DEBUG.
    RCLCPP_INFO(get_logger(), "Hello from your_node.");
  }
};

// main is the entry point. It follows the same pattern in every C++ node.
int main(int argc, char ** argv)
{
  // init reads ROS 2 arguments (e.g. --ros-args -p param:=value) from argv
  // and sets up the DDS middleware. Must be called before anything else.
  rclcpp::init(argc, argv);

  // spin hands the node to the executor and blocks until Ctrl-C.
  // make_shared<YourNode>() constructs the node and calls the constructor.
  rclcpp::spin(std::make_shared<YourNode>());

  // shutdown cleans up the middleware. Called after spin returns.
  rclcpp::shutdown();
  return 0;
}
