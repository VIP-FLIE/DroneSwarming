# Simulation

The container builds PX4 SITL v1.17.0 and matching `px4_msgs`. The pinned toolchain provides Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and Micro XRCE-DDS Agent v2.4.3. MAVSDK 4.2.0 supplies onboard mission upload. [`.env`](../../.env) records the pinned versions.

## Processes and links

| Process | Role |
| --- | --- |
| PX4 SITL | Flight control, mission execution, state, preflight checks, and failsafes |
| Gazebo Harmonic | Project `vip_x500` model, simulated sensors, physics, and `vip_windy` world |
| Micro XRCE-DDS Agent | Bridge PX4's uXRCE-DDS client to ROS 2 |
| `vehicle_control` | DDS commands/telemetry, MAVLink mission upload, and standalone Offboard control |
| `mission_execution` | Admit commands, serialize child operations, and preempt for landing |
| `mission_planner` | Compile JSON assignments and route geometry |
| QGroundControl | Ground station on the host, using MAVLink UDP 14550 |

`drone_bringup` starts the three ROS application nodes. The DDS agent, PX4, Gazebo, and QGroundControl have separate lifetimes.

| Link | Local SITL endpoint | Purpose |
| --- | --- | --- |
| uXRCE-DDS | Agent UDP 8888 | PX4 telemetry, command acknowledgments, and application commands |
| Mission MAVLink | Application UDP 14540 | Upload and progress for the first SITL vehicle |
| Ground-station MAVLink | QGroundControl UDP 14550 | Operator telemetry and control |

PX4 publishes versioned position, status, and battery topics with a `_v1` suffix under `/fmu/out/`. Land detection and command acknowledgments have no version suffix. Offboard control and trajectory setpoints enter under `/fmu/in/`. Uploaded routes execute in Mission mode without an application Offboard stream. [Application architecture](../architecture/README.md) explains mode ownership and frame conversion.

## Start and inspect a run

Follow the [Linux setup](../setup/LINUX.md) and [simulation flight procedure](../../README.md#simulation-flight-procedure). `scripts/tmux-session start` builds the workspace and starts the DDS agent and PX4/Gazebo. Launch the ROS application in pane 2 and submit commands in pane 3. QGroundControl runs separately on the host.

The default world has a textured ground grid, cloudy sky, steady 5 m/s wind, and no aerial obstacles. The project model derives from PX4's X500 and enables body wind effects. `PX4_GZ_WORLD` and `PX4_GZ_MODEL` select alternatives before startup. [Gazebo resources](../../worlds/README.md) documents those settings and the simulated battery.

The tmux session assigns one `DRONE_RUN_ID` to its panes. Separate host terminals share a run by exporting the ID printed by `scripts/new-run`. Logs reside under `run-logs/<run-id>/`: `console/` for terminal transcripts, `ros/` for node logs, and `px4/log/` for ULogs.

PX4 SITL starts ULog recording at boot through `SDLOG_MODE=2`. Hardware logs remain on the flight controller and require separate transfer.

## Simulation limits

SITL runs the autopilot against simulated physics. It does not flash flight-controller firmware or reproduce Raspberry Pi resource limits, physical sensor errors, air-to-ground radio links, or real network delay and loss.

The application provides no obstacle detection, collision avoidance, peer discovery, automatic search allocation, or peer-failure reassignment. The default world provides no building-contact or obstacle-clearance validation.

Passing SITL does not establish hardware readiness. The [testing guide](../testing/README.md#validation-status) records completed checks and remaining limits. Land and confirm disarming before stopping the application; a stored PX4 mission can continue after ROS exits.
