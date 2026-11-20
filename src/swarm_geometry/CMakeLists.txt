cmake_minimum_required(VERSION 3.8)
project(swarm_geometry)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(swarm_interfaces REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ProjectWarnings.cmake")

add_library(swarm_geometry src/geography.cpp)
target_include_directories(swarm_geometry PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(swarm_geometry PUBLIC cxx_std_17)
enable_project_cpp_diagnostics(swarm_geometry)
ament_target_dependencies(swarm_geometry SYSTEM swarm_interfaces)
install(TARGETS swarm_geometry EXPORT swarm_geometryTargets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_python_install_package(${PROJECT_NAME})
ament_export_targets(swarm_geometryTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(swarm_interfaces)
ament_package()
