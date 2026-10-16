cmake_minimum_required(VERSION 3.8)
project(vehicle_control)

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
target_compile_options(vehicle_control_core PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(vehicle_control_core
  geometry_msgs px4_msgs rclcpp rclcpp_action swarm_interfaces
)
target_link_libraries(vehicle_control_core ${px4_msgs_TARGETS})

add_executable(vehicle_control_node src/main.cpp)
target_compile_features(vehicle_control_node PRIVATE cxx_std_17)
target_compile_options(vehicle_control_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(vehicle_control_node vehicle_control_core)

install(TARGETS vehicle_control_core vehicle_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  ament_add_gtest(test_coordinates test/test_coordinates.cpp)
  target_link_libraries(test_coordinates vehicle_control_core)
endif()

ament_package()
