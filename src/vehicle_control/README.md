# vehicle_control

`vehicle_control` is the only application package that depends on `px4_msgs`. Its PX4 adapter owns DDS topic names, command routing and acknowledgements, telemetry conversion, and ENU/NED conversion. The node publishes `VehicleState` and serves the internal `vehicle_control/takeoff`, `vehicle_control/navigate_to`, `vehicle_control/follow_trajectory`, `vehicle_control/orbit_point`, and `vehicle_control/land` actions. `mission_execution` is their sole application client. External callers use the [mission-execution interface](../mission_execution/README.md). Vehicle control validates and executes the selected operation; mission execution owns intent admission, serialization, and landing preemption.

The adapter matches command acknowledgements by command and destination system/component. PX4 v1.17 does not set `VehicleCommandAck.from_external` when it publishes acknowledgements. Transition logs identify the stage, command, attempt, acknowledgement, and observed state change. Action errors include PX4 result codes and result parameters for non-accepted acknowledgements.

Application positions use `local_enu` in meters from PX4's local-position origin. PX4 v1.17 local position and trajectory setpoints use NED, so the adapter maps ENU `(east, north, up)` to PX4 `(north, east, down)`.

`NavigateTo` leaves yaw unspecified when `look_at_target` is false. When enabled, it updates yaw at a maximum setpoint rate of 2 radians per second toward the horizontal bearing from the current position to the absolute `look_at_position` in `local_enu`. Position targets remain in ENU and do not rotate with yaw. Navigation succeeds after position is within its requested tolerance and yaw error is at most 0.2 radians for 0.5 seconds. PX4 controls the resulting attitude and rotor outputs.

`FollowTrajectory` controls position alone by default. Set `control_yaw` to true to interpolate each point's `yaw_rad`; the action then requires valid yaw telemetry and checks the final wrapped yaw error against `yaw_tolerance_rad` for 0.5 seconds. A zero tolerance selects the configured default. When `control_yaw` is false, yaw fields are ignored, `yaw_tolerance_rad` must be zero, and yaw error feedback and results are NaN. Yaw control ends when the action completes or cancels.

Flight actions are rejected while `enable_flight_commands` is false. The launch default is false. Enable it only after checking the vehicle, operator control link, PX4 preflight state, configured topic prefix, and system ID. Takeoff streams position setpoints for one second, enters Offboard when needed, confirms Offboard, and then arms. A takeoff request can reuse an already active Offboard mode. Navigation cancellation commands a hold at the latest valid position while the stream continues. Landing uses PX4's `VEHICLE_CMD_NAV_LAND`, waits for its land detector, disarms if needed, and selects Auto Loiter. The action succeeds after PX4 reports landed, disarmed, and in Auto Loiter. Position setpoints stop after touchdown and disarming. A later takeoff request starts the one-second prestream and can enter Offboard again.

`OrbitPoint` sends one PX4 `VEHICLE_CMD_DO_ORBIT` command. It accepts only an armed, airborne rotary-wing vehicle in Offboard with fresh global-reference and velocity telemetry. Its center is an absolute point in `local_enu`; the center altitude is the requested orbit altitude. PX4 performs the approach and circle. The action measures signed angular travel from position telemetry after the vehicle reaches the requested radius and altitude band. It rejects a missing or changed local reference, unsupported vehicle state, invalid radius or speed, and speeds above the 2 m/s² lateral-acceleration bound. Defaults limit the radius to 20 m, speed to 3 m/s, and center distance from the local origin to 100 m. These bounds are ROS parameters `max_orbit_radius_m`, `max_orbit_speed_mps`, and `max_orbit_center_distance_m`.

`OrbitPoint` leaves yaw uncontrolled by default. Set `face_center` to true to request PX4 center-facing yaw. The action then requires valid yaw telemetry, checks the wrapped heading error while the vehicle is on the orbit circle, and fails after two seconds outside `yaw_tolerance_rad`. It completes only while yaw is within tolerance. Zero tolerance selects 0.524 radians. A nonzero yaw tolerance requires `face_center`.

After completion or cancellation, `OrbitPoint` captures the current position, streams Offboard setpoints for one second, requests Offboard, and waits for position and velocity to settle. It reports success or cancellation only after Offboard hold is confirmed. If PX4 changes to another mode first or hold recovery fails, the action reports failure and leaves the active PX4 mode in control. A lost `DO_ORBIT` acknowledgment is not retried; the action checks `VehicleStatus.nav_state` before deciding whether Orbit began.

Vehicle lifecycle, goal rejection, and PX4 transition events use ROS logging. Rejection logs include goal values, telemetry freshness, and yaw validity. Failed action results include measured progress, PX4 state, and applicable tolerances. The control setpoint timer does not emit per-setpoint log messages.

The PX4 topic prefix is the full path through `fmu` and is independent of the ROS namespace. For example, application namespace `/uav_1` with PX4 prefix `/uav_1/fmu` resolves app actions under `/uav_1` and PX4 topics under `/uav_1/fmu/in/...` and `/uav_1/fmu/out/...`.

## Geographic planning telemetry

`VehicleState` publishes the validity-gated latitude, longitude, and AMSL altitude of the local ENU origin. `local_frame_revision` changes when PX4's reference timestamp, reference values, global-reference validity flags, or XY/Z reset counters change. Position samples without a frame change retain the same revision. The planner uses these application fields to project GPS paths; PX4-specific telemetry remains inside this package. Mission execution aborts a translated JSON mission when its captured revision changes.

## Fault injection tests

Run `./scripts/torture-tests` from a built project environment for the opt-in [flight torture suite](../../docs/testing/TORTURE-TESTS.md).
The suite tests the real application executables with synthetic PX4 telemetry, writes native ROS file logs,
and retains fault timelines and action results. Its diagnostic-report unit tests also run with the package tests.
