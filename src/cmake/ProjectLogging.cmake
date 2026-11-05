if(NOT DEFINED DRONE_LOG_MIN_SEVERITY)
  set(DRONE_LOG_MIN_SEVERITY "DEBUG" CACHE STRING "Minimum compiled ROS C++ log severity")
endif()
set_property(CACHE DRONE_LOG_MIN_SEVERITY PROPERTY STRINGS DEBUG INFO WARN ERROR FATAL NONE)

string(TOUPPER "${DRONE_LOG_MIN_SEVERITY}" normalized_log_severity)
set(supported_log_severities DEBUG INFO WARN ERROR FATAL NONE)
list(FIND supported_log_severities "${normalized_log_severity}" log_severity_index)
if(log_severity_index EQUAL -1)
  message(FATAL_ERROR
    "DRONE_LOG_MIN_SEVERITY must be DEBUG, INFO, WARN, ERROR, FATAL, or NONE; "
    "received '${DRONE_LOG_MIN_SEVERITY}'")
endif()

function(enable_project_cpp_logging target_name)
  if(NOT TARGET "${target_name}")
    message(FATAL_ERROR "Cannot configure logging for missing target '${target_name}'")
  endif()

  string(TOUPPER "${DRONE_LOG_MIN_SEVERITY}" target_log_severity)
  target_compile_definitions("${target_name}" PRIVATE
    "RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_${target_log_severity}")
endfunction()
