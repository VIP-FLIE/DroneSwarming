# Run individual flight tests

Each flight test sends ROS actions to the application in the normal PX4 SITL session. The flight
command runs one case and records its telemetry, action results, and assertions. The existing
simulation session runs Gazebo, PX4, the XRCE-DDS Agent, and the application. QGroundControl runs
on the host.

## Start the simulation

Start QGroundControl on the host with UDP auto-connect enabled. Start the normal simulation
session:

```bash
./scripts/tmux-session test
```

The `test` command builds the flight and application packages inside the simulation container
before opening the four sourced container shells.

In pane 0, start the XRCE-DDS Agent:

```bash
./scripts/xrce-agent
```

In pane 1, start PX4 SITL with Gazebo:

```bash
./scripts/px4-sitl
```

In the application pane, launch the flight-enabled application:

```bash
ros2 launch drone_bringup single_vehicle.launch.py enable_flight_commands:=true
```

Confirm that Gazebo shows the X500 and QGroundControl shows the vehicle with updating telemetry.
The `vehicle_control` node must report that flight actions are enabled before a case starts.

## Run one flight

From the sourced container shell in pane 3, run one case:

```bash
./scripts/test-flights --case FT-00
```

The script uses `DRONE_RUN_ID` from the container shell. If `flight_test` is absent from the
container's ROS install or cannot import its dependencies, the script builds the package and its
dependencies from the mounted source before running the case. The script leaves the simulation
and QGroundControl running after the flight.

In a session started with `start`, run the same script in the sourced ROS CLI pane. After
`flight_test` is installed, the ROS command also runs the case:

```bash
ros2 run flight_test flight_test_case --case FT-00
```

Both commands use `DRONE_RUN_DIR` supplied by `scripts/sim-shell` and create a new
`flight-tests/FT-00-<UTC timestamp>-<suffix>/` directory for every invocation.

| Case | Flight sequence |
| --- | --- |
| FT-00 | Take off, hold position, land |
| FT-01 | Execute the four-waypoint square mission and land |
| FT-02 | Fly a fast outbound leg, stop, turn, return, land |
| FT-03 | Cancel an orbit, hold, complete an orbit, land |
| FT-04 | Fly search lanes and land |
| FT-05 | Fly altitude steps and land |
| FT-06 | Cancel navigation, hold position, land |

The routine smoke flights are FT-00, FT-01, and FT-06. The stress flights are FT-02 through FT-05.
Run each case with its own command. A failed case reports a nonzero exit status and attempts to
land when the vehicle is airborne.

FT-02 through FT-05 require valid yaw before sending a heading command. PX4 can report yaw invalid
while the vehicle is landed. These cases take off using valid position telemetry, then wait up to
15 seconds for airborne yaw validity. They attempt to land if yaw remains invalid.

## Reset before the next flight

Wait for the case to report landed and disarmed. If it remains airborne, land it through
QGroundControl before resetting. From a host terminal, close the simulation tmux session and
start a new one:

```bash
tmux kill-session -t drone-swarming
./scripts/sim stop
./scripts/tmux-session test
```

QGroundControl stays open. Stopping the simulation container ends its PX4 and Gazebo processes.
The new session creates a new run ID. Start the Agent and PX4 in panes 0 and 1, launch the
flight-enabled application in pane 2, confirm QGroundControl has reconnected, then run the next
case. The flight-test command does not reset or restart the simulation.

## Read the result

The run is stored under `run-logs/<run-id>/`:

The simulation container bind mounts this directory from the host. New flight result directories
have mode `0755` so the host user can read their files.

- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/case-report.md` records the case result and
  assertions.
- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/case-result.json` records the action outcomes,
  measurements, and recovery status.
- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/vehicle-state.csv` records position, velocity,
  yaw, armed state, Offboard state, and landed state.
- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/commanded-trajectory.csv` records trajectory
  setpoints when the case uses them.
- `console/` stores terminal output. `scripts/test-flights` saves a flight console log and exit
  status file.
- `ros/` stores ROS node logs. `px4/log/` stores PX4 ULogs.

PX4 may reject arming when a preflight check fails. Check the PX4 console log and QGroundControl's
vehicle status. QGroundControl uses host UDP port `14550`; the flight command does not reserve
that port.

## Run package tests

Run the ROS package tests in the simulation container:

```bash
./scripts/sim-shell bash -lc 'colcon test --packages-up-to flight_test vehicle_control mission_execution drone_bringup && colcon test-result --verbose'
```

FT-03 orbits a virtual `local_enu` point. The default `lawn` world has no building fixture or
contact monitor. The case does not report building clearance or collision safety.
