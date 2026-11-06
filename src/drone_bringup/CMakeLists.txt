cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

include(CTest)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  unset(RUFF_EXECUTABLE CACHE)
  find_program(RUFF_EXECUTABLE NAMES ruff)
  if(NOT RUFF_EXECUTABLE)
    message(FATAL_ERROR
      "Ruff is required to lint ROS launch files when BUILD_TESTING is enabled; install the project-pinned version or set BUILD_TESTING=OFF"
    )
  endif()

  add_test(
    NAME lint_launch_python
    COMMAND "${RUFF_EXECUTABLE}" check
      --config "${CMAKE_CURRENT_SOURCE_DIR}/ruff.toml"
      "${CMAKE_CURRENT_SOURCE_DIR}/launch"
  )
  add_test(
    NAME format_launch_python
    COMMAND "${RUFF_EXECUTABLE}" format --check
      --config "${CMAKE_CURRENT_SOURCE_DIR}/ruff.toml"
      "${CMAKE_CURRENT_SOURCE_DIR}/launch"
  )
  set_tests_properties(
    lint_launch_python format_launch_python
    PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
