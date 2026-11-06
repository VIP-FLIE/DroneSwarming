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

For the default and `start` modes, build the workspace from the repository root before starting
the ROS application:

```bash
./scripts/sim-shell colcon build --symlink-install
```

## Start the session

Run this command from the repository root in a host terminal to open four sourced container Bash
shells:

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

The container shells have `PX4_GZ_WORLD=lawn` by default. Set `PX4_GZ_WORLD` before `start` to
select another world, for example `PX4_GZ_WORLD=default ./scripts/tmux-session start`. In a
manual pane, run `PX4_GZ_WORLD=default ./scripts/px4-sitl` to select that world. See
[Gazebo worlds](../../worlds/README.md).

Run this command to open four sourced container shells, with pane 3 labeled for standalone
flight commands:

```bash
./scripts/tmux-session test
```

The `test` command builds the flight and application packages inside the simulation container
before opening the panes. Start the Agent and PX4 manually in panes 0 and 1 before running a
flight from pane 3 with `./scripts/test-flights --case FT-00`.

The launcher must run outside the project container because tmux starts on the machine that runs the script. The script reports an error when it detects a container environment.

The script creates a UTC run directory, starts the `simulation` Compose service through `scripts/sim`, then attaches the `drone-swarming` tmux session. All four panes use the session's `DRONE_RUN_ID`. Running the command without an argument again reuses that ID and recreates any missing project panes. The `start` and `test` arguments apply to a new session; they report an error if the session already exists.

| Pane | Process or shell |
| --- | --- |
| 0 | Sourced container Bash shell, or XRCE-DDS Agent with `start` |
| 1 | Sourced container Bash shell, or PX4 SITL and Gazebo Harmonic with `start` |
| 2 | Sourced container shell for the ROS application |
| 3 | Sourced container shell for ROS 2 CLI commands, labeled Flight Tests with `test` |

All four panes start sourced container Bash shells without an argument or with `test`. The
application pane remains a shell in every mode. Only `start` launches the Agent and PX4. The
script does not start the ROS application or send a mission.

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
