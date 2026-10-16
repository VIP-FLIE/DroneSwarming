# Simulation

The Linux container builds PX4 SITL v1.17.0 and matching `px4_msgs` v1.17.0. The pinned toolchain image provides Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and Micro XRCE-DDS Agent v2.4.3. QGroundControl runs on the host and uses MAVLink over UDP port 14550.

## Processes

| Process | Role |
| --- | --- |
| PX4 SITL | Autopilot software, vehicle state, and failsafes |
| Gazebo Harmonic | X500 model, sensors, physics, and world |
| Micro XRCE-DDS Agent | Connects PX4's uXRCE-DDS client to ROS 2 |
| `vehicle_control` | Sends Offboard position setpoints and PX4 vehicle commands |
| `mission_execution` | Sequences application flight actions |
| QGroundControl | Ground control station and MAVLink telemetry |

`drone_bringup` starts only the two ROS application nodes. Start the agent and PX4 SITL in separate terminals. The PX4 v1.17.0 SITL ROS 2 graph exposes versioned state topics `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1`, and `/fmu/out/battery_status_v1`. It exposes `/fmu/out/vehicle_land_detected` and `/fmu/out/vehicle_command_ack` without a version suffix. PX4 receives Offboard control and trajectory setpoints under `/fmu/in/`.

## Start

Follow the [Linux setup](../setup/LINUX.md) and the root [simulation flight procedure](../../README.md#simulation-flight-procedure). Run `./scripts/xrce-agent` and `./scripts/px4-sitl` in separate native terminals. Each wrapper starts the simulation service and builds its image if it is missing, then runs its process inside the container with terminal input and output attached. Use `./scripts/sim-shell` to open a sourced interactive container shell for the application launch and ROS commands. QGroundControl runs separately on the host. Check that it shows the simulated vehicle and updating telemetry before starting the flight-enabled application.

The `vehicle_control` default keeps flight actions disabled. The example enables them only for the simulation procedure.
