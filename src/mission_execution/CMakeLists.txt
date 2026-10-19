cmake_minimum_required(VERSION 3.8)
project(mission_execution)

include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ProjectWarnings.cmake")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(swarm_interfaces REQUIRED)

add_executable(mission_execution_node
  src/geometry.cpp
  src/main.cpp
  src/mission_execution_node.cpp
)
target_include_directories(mission_execution_node PRIVATE include)
target_compile_features(mission_execution_node PRIVATE cxx_std_17)
enable_project_cpp_diagnostics(mission_execution_node)
ament_target_dependencies(mission_execution_node SYSTEM
  geometry_msgs rclcpp rclcpp_action swarm_interfaces
)

install(TARGETS mission_execution_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_geometry test/test_geometry.cpp src/geometry.cpp)
  target_compile_features(test_geometry PRIVATE cxx_std_17)
  enable_project_cpp_diagnostics(test_geometry)
  ament_target_dependencies(test_geometry SYSTEM geometry_msgs)
  target_include_directories(test_geometry PRIVATE include)
endif()

ament_package()
