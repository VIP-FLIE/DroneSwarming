# Simulation

The Linux container builds PX4 SITL v1.17.0 and matching `px4_msgs` v1.17.0. The pinned toolchain image provides Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and Micro XRCE-DDS Agent v2.4.3. QGroundControl runs on the host and uses MAVLink over UDP port 14550.

## Processes

| Process | Role |
| --- | --- |
| PX4 SITL | Autopilot software, vehicle state, and failsafes |
| Gazebo Harmonic | X500 model, sensors, physics, and world |
| Micro XRCE-DDS Agent | Connects PX4's uXRCE-DDS client to ROS 2 |
| `vehicle_control` | Sends Offboard position setpoints and PX4 vehicle commands |
| `mission_planner` | Decodes JSON assignments and translates relative or geographic paths into application instructions |
| `mission_execution` | Admits all application commands, checks state, serializes operations, and executes missions |
| QGroundControl | Ground control station and MAVLink telemetry |

`drone_bringup` starts only the three ROS application nodes. Start the agent and PX4 SITL in separate terminals. The PX4 v1.17.0 SITL ROS 2 graph exposes versioned state topics `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1`, and `/fmu/out/battery_status_v1`. It exposes `/fmu/out/vehicle_land_detected` and `/fmu/out/vehicle_command_ack` without a version suffix. PX4 receives Offboard control and trajectory setpoints under `/fmu/in/`.

The public `mission_execution/land` action interrupts ordinary work and selects the internal `vehicle_control/land` operation. That operation requests PX4 Auto Land and waits for the land detector. It disarms the vehicle if needed, selects Auto Loiter, and succeeds after PX4 reports landed, disarmed, and outside Offboard. Position setpoints stop after touchdown and disarming. A later `takeoff` action streams setpoints for one second, enters Offboard when needed, confirms the mode, and arms.

## Orbit action

`mission_execution` accepts `/mission_execution/orbit_point` and selects the internal `/vehicle_control/orbit_point` operation for an airborne rotary-wing vehicle in Offboard. The action requires valid global-reference data from `VehicleLocalPosition`; it converts the requested absolute `local_enu` center to the WGS84 center used by PX4's `DO_ORBIT` command. PX4 controls approach, radius, and tangential speed. The caller sets `face_center` to request center-facing yaw; the action checks yaw telemetry and heading error when enabled. The action tracks completed revolutions from position samples, then restores and confirms Offboard hold. A missing or changed global reference rejects or aborts the action.

For Orbit validation, run only in the controlled SITL procedure with flight commands enabled. Confirm that PX4 reports Orbit state, the requested direction and radius are visible in telemetry, progress advances only after reaching the circle, cancellation returns to a confirmed Offboard hold, and landing starts only after mission Orbit succeeds.

## Start

Follow the [Linux setup](../setup/LINUX.md) and the root [simulation flight procedure](../../README.md#simulation-flight-procedure). Run `./scripts/xrce-agent` and `./scripts/px4-sitl` in separate native terminals or in the sourced container shells opened by `./scripts/tmux-session`. From native terminals, each wrapper starts the simulation service and builds its image if it is missing, then runs its process inside the container with terminal input and output attached. The wrappers save terminal output under `run-logs/<run-id>/console/` on the host. `scripts/sim-shell` points `ROS_LOG_DIR` at `run-logs/<run-id>/ros/` through the container mount. `scripts/px4-sitl` stores ULog files under `run-logs/<run-id>/px4/log/`. QGroundControl runs separately on the host. Check that it shows the simulated vehicle and updating telemetry before starting the flight-enabled application.

`scripts/tmux-session` creates one run ID for its panes. Separate host terminals use one shared ID when each terminal exports the value printed by `./scripts/new-run`. `DRONE_RUN_ID`, `DRONE_RUN_LOG_ROOT`, `DRONE_RUN_DIR`, and `DRONE_RUN_METADATA` are available to processes launched inside the simulation container.

`scripts/px4-sitl` selects the flat `vip_windy` world and project-owned `vip_x500` model by
default. The world has a marked ground texture, a cloudy sky, and 5 m/s wind without aerial
obstacles. Set `PX4_GZ_WORLD` or `PX4_GZ_MODEL` before starting PX4 to select another world or
PX4-supported model. The X500 SITL battery simulator drains linearly over 900 seconds of armed
time and resets its simulated charge while disarmed. See [Gazebo worlds](../../worlds/README.md).

PX4 SITL starts ULog recording at boot and stops it at shutdown through `SDLOG_MODE=2`. PX4 v1.17 accepts `PX4_PARAM_*` environment variables in its POSIX startup script. See the [PX4 v1.17 logging parameters](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/docs/en/dev_log/logging.md#configuration) and [PX4 v1.17 parameter environment overrides](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/ROMFS/px4fmu_common/init.d-posix/rcS#L1127-L1141).

The PX4 log mount applies to SITL. Hardware PX4 logs remain on the flight controller's storage and require transfer to the host.

The `vehicle_control` default keeps flight actions disabled. The example enables them only for the simulation procedure.
