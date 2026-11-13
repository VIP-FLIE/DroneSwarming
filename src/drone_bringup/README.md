# drone_bringup

`single_vehicle.launch.py` starts `vehicle_control`, `mission_execution`, and `mission_planner` in an optional application namespace. It does not start PX4, Gazebo, or a DDS agent and does not start a mission. The PX4 topic prefix is configured independently. The `log_level` launch argument sets the runtime ROS log threshold for all three nodes and defaults to `info`.

The launch exposes `mission_execution/execute_flight_plan`, the planning-only `mission_planner/plan_flight` service, `execute_waypoint_mission` and `mission_execution/{takeoff,navigate_to,follow_trajectory,orbit_point,land}` to callers. Internal execution endpoints are `vehicle_control/{takeoff,navigate_to,follow_trajectory,orbit_point,land}`. The namespace prefixes both sets. For `namespace:=uav_1`, public navigation is `/uav_1/mission_execution/navigate_to` and its child endpoint is `/uav_1/vehicle_control/navigate_to`. The node names and endpoint paths require no launch remappings.

ROS 2 writes node log files to `ROS_LOG_DIR` when the environment variable is set. `scripts/sim-shell` points it at the current host-backed run directory. Set `log_level:=debug` to emit debug messages when the C++ build includes them.

The build-time `DRONE_LOG_MIN_SEVERITY` setting compiles out lower severity C++ macros. `DEBUG` includes all severities. `NONE` compiles out all `RCLCPP_*` log macros. Change this value in `.env` and rebuild the simulation image to apply a different minimum. A runtime `log_level` cannot restore messages removed during compilation. Set the option directly for a workspace build with `colcon build --symlink-install --cmake-args -DDRONE_LOG_MIN_SEVERITY=NONE`.

Flight actions are disabled by default. Set `enable_flight_commands:=true` only for an explicitly controlled simulation or hardware session after confirming PX4 telemetry, the system ID, and the operator's recovery path.

The orbit action limits can be configured with the `max_orbit_radius_m`, `max_orbit_speed_mps`, and `max_orbit_center_distance_m` launch arguments. Their defaults are 20 m, 3 m/s, and 100 m.

`vehicle_id` selects the explicit fleet assignment and defaults to the namespace-derived ID, or `uav_1` at root. `maximum_plan_altitude_m` and `maximum_plan_distance_m` default to 30 and 100 meters and configure both planner and executor. See the [JSON guide](../mission_planner/README.md).
