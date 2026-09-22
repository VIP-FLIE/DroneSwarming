cmake_minimum_required(VERSION 3.8)
project(swarm_naive)

# Writes build/<package>/compile_commands.json, which colcon merges into
# build/compile_commands.json. That is what .vscode/c_cpp_properties.json points
# at, so editor IntelliSense uses the real compile flags instead of a hand
# maintained include list. Costs nothing at build time.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings are the cheapest bug detector available, so they are on and loud.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake is the ROS 2 build system; the other two are what we #include.
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(waypoint_drone src/waypoint_drone.cpp)
target_compile_features(waypoint_drone PRIVATE cxx_std_17)
target_link_libraries(waypoint_drone PRIVATE rclcpp::rclcpp ${px4_msgs_TARGETS})

# ros2 run looks for executables in lib/<package>, so this is what makes
# `ros2 run swarm_naive waypoint_drone` work.
install(TARGETS waypoint_drone DESTINATION lib/${PROJECT_NAME})

# ros2 launch looks for launch files in share/<package>, so this is what makes
# `ros2 launch swarm_naive swarm.launch.py` work. Missing this line is the
# reason a launch file that plainly exists reports "file not found".
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
