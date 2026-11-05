# tmux simulation session

The `scripts/tmux-session` command starts the simulation container and opens four tmux panes for the project flight workflow.

## Requirements

- Linux host with Docker Compose, `tmux`, and the util-linux `script` command.
- NVIDIA Container Toolkit configured for Docker.
- X11 or XWayland display access for Gazebo.
- QGroundControl running on the host before the flight-enabled application starts.
- Workspace built in the container before launching the ROS application.

Install tmux on Ubuntu with:

```bash
sudo apt install tmux util-linux
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

The script creates a UTC run directory, starts the `simulation` Compose service through `scripts/sim`, then attaches the `drone-swarming` tmux session. All four panes use the session's `DRONE_RUN_ID`. Running the command again reuses that ID and recreates any missing project panes.

| Pane | Process or shell |
| --- | --- |
| 0 | XRCE-DDS Agent, started by `scripts/xrce-agent` |
| 1 | PX4 SITL and Gazebo Harmonic, started by `scripts/px4-sitl` |
| 2 | Sourced container shell for the ROS application |
| 3 | Sourced container shell for ROS 2 CLI commands |

The application and CLI panes start interactive shells. The script does not start the ROS application or send a mission.

## Run logs

The host stores each run at `run-logs/<UTC timestamp>/`. The simulation container mounts the directory at `/workspace/run-logs`.

| Path | Contents |
| --- | --- |
| `run-info.txt` | Run ID, creation time, Git commit, branch, and worktree dirty state |
| `console/` | Host terminal transcripts and process exit status files |
| `ros/` | ROS 2 node log files created through `ROS_LOG_DIR` |
| `px4/log/` | PX4 SITL ULog files |
| `flight-tests/` | Per-case flight test artifacts |

The tmux launcher creates one ID for the session. Detaching and reattaching preserves the ID. Killing the session and starting a new one creates a new run directory. A session created before run ID support has no saved ID; close it with `tmux kill-session -t drone-swarming` before starting a new session. Remove a run directory manually to reclaim host disk space.

For separate host terminals, run `export DRONE_RUN_ID="$(./scripts/new-run)"` once. Copy `printf 'export DRONE_RUN_ID=%q\n' "$DRONE_RUN_ID"` output into the other terminals before starting the wrappers.

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
