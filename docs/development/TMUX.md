# tmux simulation session

The `scripts/tmux-session` command starts the simulation container and opens four tmux panes for the project flight workflow.

## Requirements

- Linux host with Docker Compose, `tmux`, and the util-linux `script` command.
- NVIDIA Container Toolkit configured for Docker.
- X11 or XWayland display access for Gazebo.
- QGroundControl running on the host before the flight-enabled application starts.

Install tmux on Ubuntu with:

```bash
sudo apt install tmux util-linux
```

## Start the session

Run this command from the repository root in a host terminal to build the workspace and open four
sourced container Bash shells:

```bash
./scripts/tmux-session
```

Start the Agent in pane 0:

```bash
./scripts/xrce-agent
```

Start PX4 in pane 1:

```bash
./scripts/px4-sitl
```

Run this command to start the XRCE-DDS Agent and PX4 SITL with Gazebo when creating the session:

```bash
./scripts/tmux-session start
```

The container shells use `PX4_GZ_WORLD=vip_windy` and `PX4_GZ_MODEL=vip_x500` by default. Set
either variable before `start` to select another world or model, for example
`PX4_GZ_WORLD=default PX4_GZ_MODEL=x500 ./scripts/tmux-session start`. In a manual pane, run
`PX4_GZ_WORLD=default PX4_GZ_MODEL=x500 ./scripts/px4-sitl`. See
[Gazebo worlds](../../worlds/README.md).

The launcher must run outside the project container because tmux starts on the machine that runs the script. The script reports an error when it detects a container environment.

When creating a session, the script creates a UTC run directory, starts the `simulation` Compose service through `scripts/sim`, and builds every workspace package with `colcon build --symlink-install` before attaching the `drone-swarming` tmux session. The build includes `flight_test`. Each `scripts/sim-shell` pane sources ROS 2, PX4 messages, and `/workspace/install/setup.bash` before opening its shell or running a command. All four panes use the session's `DRONE_RUN_ID`. Running the command without an argument when the session exists reuses its run ID and recreates any missing project panes without rebuilding. The `start` argument applies to a new session and reports an error if the session already exists.

| Pane | Process or shell |
| --- | --- |
| 0 | Sourced container Bash shell, or XRCE-DDS Agent with `start` |
| 1 | Sourced container Bash shell, or PX4 SITL and Gazebo Harmonic with `start` |
| 2 | Sourced container shell for the ROS application |
| 3 | Sourced container shell for ROS 2 CLI commands and flight test commands |

All four panes start sourced container Bash shells without an argument. Only `start` launches the
Agent and PX4. The script does not start the ROS application or send a mission.

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

Start QGroundControl before PX4. Confirm that QGroundControl shows the simulated vehicle and current telemetry after PX4 starts.

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
