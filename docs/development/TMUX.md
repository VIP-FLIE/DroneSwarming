# tmux simulation session

The `scripts/tmux-session` command starts the simulation container and opens four tmux panes for the project flight workflow.

## Requirements

- Linux host with Docker Compose and `tmux`.
- NVIDIA Container Toolkit configured for Docker.
- X11 or XWayland display access for Gazebo.
- QGroundControl running on the host before the flight-enabled application starts.
- Workspace built in the container before launching the ROS application.

Install tmux on Ubuntu with:

```bash
sudo apt install tmux
```

Build the workspace from the repository root before starting the ROS application:

```bash
./scripts/sim-shell colcon build --symlink-install
```

## Start the session

Run this command from the repository root in a host terminal:

```bash
./scripts/tmux-session
```

The launcher must run outside the project container because tmux starts on the machine that runs the script. The script reports an error when it detects a container environment.

The script starts the `simulation` Compose service through `scripts/sim`, then attaches the `drone-swarming` tmux session. Running the command again attaches to the existing session and recreates any missing project panes.

| Pane | Process or shell |
| --- | --- |
| 0 | XRCE-DDS Agent, started by `scripts/xrce-agent` |
| 1 | PX4 SITL and Gazebo Harmonic, started by `scripts/px4-sitl` |
| 2 | Sourced container shell for the ROS application |
| 3 | Sourced container shell for ROS 2 CLI commands |

The application and CLI panes start interactive shells. The script does not start the ROS application or send a mission.

## Run a flight mission

Start QGroundControl before the tmux session. Confirm that QGroundControl shows the simulated vehicle and current telemetry after PX4 starts.

In pane 2, start the application with flight commands enabled for SITL:

```bash
ros2 launch drone_bringup single_vehicle.launch.py enable_flight_commands:=true
```

In pane 3, check vehicle state and available actions, then send the mission commands in the root [simulation flight procedure](../../README.md#simulation-flight-procedure).

## Session controls

- Press `Ctrl-b`, then `d` to detach. Run `tmux attach-session -t drone-swarming` to reattach.
- Press `Ctrl-b`, then an arrow key to move between panes.
- Press `Ctrl-c` in a pane to stop its foreground process.
- Run `tmux kill-session -t drone-swarming` to close the session. Run `./scripts/sim stop` from the repository root to stop the simulation container and revoke display access.
