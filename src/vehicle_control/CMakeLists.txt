cmake_minimum_required(VERSION 3.8)
project(vehicle_control)

include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ProjectWarnings.cmake")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(swarm_interfaces REQUIRED)

add_library(vehicle_control_core
  src/coordinates.cpp
  src/px4_adapter.cpp
  src/vehicle_control_node.cpp
)
target_include_directories(vehicle_control_core PUBLIC include)
target_compile_features(vehicle_control_core PUBLIC cxx_std_17)
enable_project_cpp_diagnostics(vehicle_control_core)
ament_target_dependencies(vehicle_control_core SYSTEM
  geometry_msgs px4_msgs rclcpp rclcpp_action swarm_interfaces
)
target_link_libraries(vehicle_control_core ${px4_msgs_TARGETS})

add_executable(vehicle_control_node src/main.cpp)
target_compile_features(vehicle_control_node PRIVATE cxx_std_17)
enable_project_cpp_diagnostics(vehicle_control_node)
target_link_libraries(vehicle_control_node vehicle_control_core)

install(TARGETS vehicle_control_core vehicle_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_coordinates test/test_coordinates.cpp)
  target_compile_features(test_coordinates PRIVATE cxx_std_17)
  enable_project_cpp_diagnostics(test_coordinates)
  target_link_libraries(test_coordinates vehicle_control_core)

  ament_add_gtest(test_px4_adapter test/test_px4_adapter.cpp)
  target_compile_features(test_px4_adapter PRIVATE cxx_std_17)
  enable_project_cpp_diagnostics(test_px4_adapter)
  ament_target_dependencies(test_px4_adapter SYSTEM px4_msgs rclcpp)
  target_link_libraries(test_px4_adapter vehicle_control_core)
  # Keep the ROS test isolated from live PX4 and Gazebo DDS traffic.
  set_tests_properties(test_px4_adapter PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=77")
endif()

ament_package()
