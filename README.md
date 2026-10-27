# DroneSwarming

ROS 2 control software for PX4 search-and-rescue drones. The supported development environment is a Docker container with Ubuntu 24.04, ROS 2 Jazzy, PX4 SITL v1.17.0, Gazebo Harmonic, and matching `px4_msgs` v1.17.0. QGroundControl runs on the Linux host.

> **Do not push directly to `main`.** Create feature branches from `dev`, validate the changes, and open pull requests into `dev`. Promote `dev` to `main` through a release pull request after required validation and approval by at least two team members. `main` must build, run the documented SITL mission without a vehicle crash, and remain ready for project demos. Read the [Repository Rules](docs/REPOSITORY_RULES.md) and [Contributing](CONTRIBUTING.md) before changing the repository.

## Architecture

| Package | Responsibility |
| --- | --- |
| `swarm_interfaces` | Application messages and actions. Positions use `local_enu`. |
| `vehicle_control` | PX4 DDS topics, command acknowledgements, telemetry, frame conversion, and reusable takeoff, navigation, and landing actions. |
| `mission_execution` | Sequences takeoff, absolute waypoint targets derived from launch-relative offsets, and optional landing. |
| `swarm_coordination` | Empty scaffold for future peer health and task coordination. |
| `drone_bringup` | Launches the application nodes with separate application namespace and PX4 topic-prefix settings. |

PX4 retains low-level flight control, preflight checks, and failsafes. `vehicle_control` sends position setpoints in Offboard mode. `mission_execution` uses ROS 2 actions and does not depend on PX4 messages.

The application frame is `local_enu`: X is east, Y is north, and Z is up, in meters from PX4's local-position origin. PX4 v1.17 local positions and trajectory setpoints use NED, so `vehicle_control` maps `(east, north, up)` to `(north, east, down)`. The PX4 topic prefix is the full path through `fmu`; `/fmu` addresses the default single-vehicle topics and `/uav_1/fmu` addresses a namespaced PX4 instance.

## Container setup and build

From a native terminal at the repository root, enter the project container:

```bash
./scripts/sim-shell
```

The script starts the simulation service, builds its image if it is missing, and opens a shell with ROS 2, PX4 messages, and the workspace sourced. To rebuild after Dockerfile or container dependency changes, run `./scripts/sim up --build -d simulation`.

Build the workspace from the container shell:

```bash
cd /workspace
colcon build --symlink-install
source /workspace/install/setup.bash
```

With no command argument, `scripts/sim-shell` opens an interactive shell. With a command, it runs the command inside the sourced container and keeps the process attached to the native terminal. For example, `./scripts/sim-shell ros2 topic list` runs `ros2 topic list` in the container.

Run the unit tests and ROS launch-file lint checks:

```bash
colcon test --packages-select vehicle_control mission_execution drone_bringup
colcon test-result --verbose
```

The project Dockerfile builds PX4 and matching `px4_msgs` from v1.17.0. `drone_bringup` does not start PX4 or a simulator.

## Optional: launch the application without flight commands

Use this when inspecting ROS topics without sending flight commands. Do not run it before or during the flight procedure below; that procedure starts its own application process with flight commands enabled.

```bash
./scripts/sim-shell ros2 launch drone_bringup single_vehicle.launch.py enable_flight_commands:=false
```

The single-vehicle simulation uses the default PX4 topic prefix `/fmu`. Use a different `px4_topic_prefix` only when PX4 is configured to publish under that prefix. The application namespace and PX4 topic prefix are independent settings.

## Simulation flight procedure

Complete the [Linux setup](docs/setup/LINUX.md), build the workspace, and install [QGroundControl](docs/setup/LINUX.md#install-qgroundcontrol) on the host. Start QGroundControl and leave it open. With its default UDP auto-connect enabled, it listens for PX4 on UDP port 14550. If no vehicle appears, check QGroundControl's [UDP auto-connect setting](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/comm_links.html). Confirm the simulated vehicle appears and its telemetry updates before starting the flight-enabled application.

Start the tmux simulation session from a host terminal:

```bash
./scripts/tmux-session
```

The session starts the XRCE-DDS Agent and PX4 SITL with Gazebo Harmonic. It opens a sourced container shell for the application in pane 2 and a sourced ROS 2 CLI shell in pane 3. Confirm that the simulated vehicle and telemetry appear in QGroundControl before starting the flight-enabled application.

In pane 2, start the application with flight actions enabled for SITL:

```bash
ros2 launch drone_bringup single_vehicle.launch.py enable_flight_commands:=true
```

In pane 3, check one vehicle-state message and confirm the mission action is available, then send the mission:

```bash
ros2 topic echo --once /vehicle_state
ros2 action list
ros2 action send_goal /execute_waypoint_mission \
  swarm_interfaces/action/ExecuteWaypointMission \
  "{takeoff_altitude_m: 3.0, waypoint_offsets: [{x: 3.0, y: 0.0, z: 3.0}, {x: 3.0, y: 3.0, z: 3.0}, {x: 0.0, y: 3.0, z: 3.0}, {x: 0.0, y: 0.0, z: 3.0}], position_tolerance_m: 0.5, waypoint_timeout_sec: 30.0, land_after_waypoints: true}" \
  --feedback
```

The mission climbs 3 m, flies a 3 m square at 3 m relative to the position captured at mission start, then requests PX4 auto-land. Each waypoint offset is added to that same mission origin. The mission executor converts each offset to an absolute `local_enu` target, and `vehicle_control` converts it to PX4 NED setpoints.

Press `Ctrl-b`, then `d` to detach from tmux. Run `tmux attach-session -t drone-swarming` to return to the session.

## Scope and verification

`swarm_coordination` defines no peer discovery, allocation, failure detection, or flight behavior. Perception, collision avoidance, multi-drone task allocation, and payload operations are outside the initial vertical slice.

The v1.17 Offboard topic and timestamp behavior follows the [PX4 v1.17 ROS 2 Offboard example](https://docs.px4.io/v1.17/en/ros2/offboard_control). The PX4 DDS configuration is pinned to [PX4-Autopilot v1.17.0](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/uxrce_dds_client/dds_topics.yaml). In the running v1.17 SITL graph, PX4 exposes versioned position, status, and battery topics with a `_v1` suffix. ROS package and action APIs target [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/).

## Project guides

- [Architecture diagrams](docs/architecture/README.md)
- [Linux and Docker setup](docs/setup/LINUX.md)
- [Simulation architecture](docs/simulation/HOW-IT-WORKS.md)
- [Simulation fidelity](docs/simulation/SIMULATION-FIDELITY.md)
- [Gazebo world resources](worlds/README.md)
- [Swarm development stages](docs/swarm/SWARM.md)
- [Neovim LSP setup](docs/development/NEOVIM.md)
- [tmux simulation session](docs/development/TMUX.md)
- [VS Code LSP setup](docs/development/VS-CODE.md)
- [Zed LSP setup](docs/development/ZED.md)
- [Repository rules](docs/REPOSITORY_RULES.md)
- [Repository coding rules](CONTRIBUTING.md)
- [Glossary](docs/reference/GLOSSARY.md)
