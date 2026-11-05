# drone_bringup

`single_vehicle.launch.py` starts `vehicle_control` and `mission_execution` in an optional application namespace. It does not start PX4, Gazebo, or a DDS agent and does not start a mission. The PX4 topic prefix is configured independently. The `log_level` launch argument sets the runtime ROS log threshold for both nodes and defaults to `info`.

ROS 2 writes node log files to `ROS_LOG_DIR` when the environment variable is set. `scripts/sim-shell` points it at the current host-backed run directory. Set `log_level:=debug` to emit debug messages when the C++ build includes them.

The build-time `DRONE_LOG_MIN_SEVERITY` setting compiles out lower severity C++ macros. `DEBUG` includes all severities. `NONE` compiles out all `RCLCPP_*` log macros. Change this value in `.env` and rebuild the simulation image to apply a different minimum. A runtime `log_level` cannot restore messages removed during compilation. Set the option directly for a workspace build with `colcon build --symlink-install --cmake-args -DDRONE_LOG_MIN_SEVERITY=NONE`.

Flight actions are disabled by default. Set `enable_flight_commands:=true` only for an explicitly controlled simulation or hardware session after confirming PX4 telemetry, the system ID, and the operator's recovery path.

The orbit action limits can be configured with the `max_orbit_radius_m`, `max_orbit_speed_mps`, and `max_orbit_center_distance_m` launch arguments. Their defaults are 20 m, 3 m/s, and 100 m.
