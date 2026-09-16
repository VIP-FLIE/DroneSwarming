cmake_minimum_required(VERSION 3.8)
project(drone_swarm)

# Warnings are the cheapest bug detector available, so they are on and loud.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake is the ROS 2 build system; the other two are what we #include.
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hop src/hop.cpp)
target_compile_features(hop PRIVATE cxx_std_17)
target_link_libraries(hop PRIVATE rclcpp::rclcpp ${px4_msgs_TARGETS})

# ros2 run looks for executables in lib/<package>, so this is what makes
# `ros2 run drone_swarm hop` work.
install(TARGETS hop DESTINATION lib/${PROJECT_NAME})

ament_package()
