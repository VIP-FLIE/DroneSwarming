# vehicle_control

`vehicle_control` is the only application package that depends on `px4_msgs`. Its PX4 adapter owns DDS topic names, command routing and acknowledgements, telemetry conversion, and ENU/NED conversion. The node exposes `VehicleState` and the `takeoff`, `navigate_to`, and `land` actions.

Application positions use `local_enu` in meters from PX4's local-position origin. PX4 v1.17 local position and trajectory setpoints use NED, so the adapter maps ENU `(east, north, up)` to PX4 `(north, east, down)`.

Flight actions are rejected while `enable_flight_commands` is false. The launch default is false. Enable it only after checking the vehicle, operator control link, PX4 preflight state, configured topic prefix, and system ID. Takeoff warms the setpoint stream before requesting Offboard and arming. Navigation cancellation commands a hold at the latest valid position while the stream continues. Landing uses `VEHICLE_CMD_NAV_LAND` and reports success only after PX4's land detector reports landed.

The PX4 topic prefix is the full path through `fmu` and is independent of the ROS namespace. For example, application namespace `/uav_1` with PX4 prefix `/uav_1/fmu` resolves app actions under `/uav_1` and PX4 topics under `/uav_1/fmu/in/...` and `/uav_1/fmu/out/...`.
