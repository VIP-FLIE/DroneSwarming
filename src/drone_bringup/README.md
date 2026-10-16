# drone_bringup

`single_vehicle.launch.py` starts `vehicle_control` and `mission_execution` in an optional application namespace. It does not start PX4, Gazebo, or a DDS agent and does not start a mission. The PX4 topic prefix is configured independently.

Flight actions are disabled by default. Set `enable_flight_commands:=true` only for an explicitly controlled simulation or hardware session after confirming PX4 telemetry, the system ID, and the operator's recovery path.
