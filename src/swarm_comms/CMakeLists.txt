cmake_minimum_required(VERSION 3.8)
project(swarm_comms)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# --------------------------------------------------------------------------
# Our own message type.
#
# rosidl reads msg/SwarmState.msg and generates C++ (and Python, and the DDS
# type support underneath both) at build time. The generated header is
# swarm_comms/msg/swarm_state.hpp: the message name in snake_case, under the
# package name.
#
# Most tutorials put messages in a package of their own, and for a large project
# that is right, because everybody who wants to talk to you then depends on a
# tiny interface package instead of on all of your code. One message used by one
# node does not earn a second package, so it lives here.
# --------------------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwarmState.msg"
)

add_executable(formation_drone src/formation_drone.cpp)
target_compile_features(formation_drone PRIVATE cxx_std_17)

# Using a message from the SAME package it is generated in needs this extra
# step. Anywhere else you would write find_package(swarm_comms) and link the
# imported target, but that target does not exist yet while this package is
# still being built. rosidl_get_typesupport_target names the target that is
# being generated right now, so the executable can link against it directly.
#
# Leave it out and the code compiles (the headers are generated either way) and
# then fails to link, with undefined references to the message's type support.
rosidl_get_typesupport_target(swarm_state_typesupport
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(formation_drone PRIVATE
  rclcpp::rclcpp
  ${px4_msgs_TARGETS}
  ${swarm_state_typesupport}
)

install(TARGETS formation_drone DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
