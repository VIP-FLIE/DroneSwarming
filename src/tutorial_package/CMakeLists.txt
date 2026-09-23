# CMakeLists.txt is the build script for a C++ ROS 2 package.
# Every line that matters for a minimal node is here; nothing extra.

cmake_minimum_required(VERSION 3.8)

# The project name must match <name> in package.xml and the directory name.
project(tutorial_package)

# Writes a compile_commands.json that VS Code's IntelliSense reads.
# Without it, the editor shows red squiggles on every #include even when the
# build succeeds. Costs nothing.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Turn on common warnings. Optional, but cheap.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package locates a dependency and puts its headers and libraries on the
# path. You need one of these for every <depend> in package.xml.
# ament_cmake is always required; add rclcpp, px4_msgs, etc. as needed.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# add_executable names the binary and lists its source files.
# "your_node" is what you type in `ros2 run tutorial_package your_node`.
# Add more source files (space-separated) if you split code across files.
add_executable(your_node src/your_node.cpp)

# rclcpp needs C++17. This line sets that requirement on this target only.
target_compile_features(your_node PRIVATE cxx_std_17)

# Link the libraries. rclcpp::rclcpp is an "imported target": it already knows
# its own include directories, so you do not need a separate include_directories
# call. When you add px4_msgs, append ${px4_msgs_TARGETS} here.
target_link_libraries(your_node PRIVATE rclcpp::rclcpp)

# ros2 run searches lib/<package_name>/ for executables. Without this install
# rule the binary builds fine but `ros2 run` reports "No executable found".
install(TARGETS your_node DESTINATION lib/${PROJECT_NAME})

# Always the last line. Registers the package with the ament index so
# `ros2 pkg list` and `ros2 run` can find it.
ament_package()
