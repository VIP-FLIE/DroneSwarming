# Run individual flight tests

The [blind node torture suite](../../tests/blind_torture/README.md) runs planner, executor, and vehicle-control processes headlessly with synthetic ROS peers in an isolated container. It requires no PX4 or Gazebo process and records malformed-input, concurrency, cancellation, telemetry, and resource-budget results.

Each flight test sends ROS actions through `mission_execution` to `vehicle_control` in the normal PX4 SITL session. The flight
command runs one case and records its telemetry, action results, and assertions. The existing
simulation session runs Gazebo, PX4, the XRCE-DDS Agent, and the application. QGroundControl runs
on the host.

FT-01 executes an uploaded PX4 mission and requires the MAVLink link on UDP 14540 and a valid global reference. Other cases retain standalone Offboard operations. See [onboard missions](../architecture/PX4-ONBOARD-MISSIONS.md) for setup and cancellation behavior.

## Start the simulation

Start QGroundControl on the host with UDP auto-connect enabled. Start the normal simulation
session:

```bash
./scripts/tmux-session
```

Creating the session builds every workspace package, including `flight_test`, before opening the
four sourced container shells.

In pane 0, start the XRCE-DDS Agent:

```bash
./scripts/xrce-agent
```

In pane 1, start PX4 SITL with Gazebo:

```bash
./scripts/px4-sitl
```

Confirm that Gazebo shows the X500 and QGroundControl shows the vehicle with updating telemetry and passing PX4 preflight checks. In the application pane, launch the flight-enabled application:

```bash
ros2 launch drone_bringup single_vehicle.launch.py enable_flight_commands:=true
```

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

The full workspace build installs `flight_test`. The ROS command also runs one case:

```bash
ros2 run flight_test flight_test_case --case FT-00
```

Both commands use `DRONE_RUN_DIR` supplied by `scripts/sim-shell` and create a new
`flight-tests/FT-00-<UTC timestamp>-<suffix>/` directory for every invocation.

| Case | Flight sequence |
| --- | --- |
| FT-00 | Take off, hold position, land |
| FT-01 | Execute the four-waypoint square mission and land |
| FT-02 | Visit an outbound point, hold it, turn, return, land |
| FT-03 | Cancel an orbit, hold, complete an orbit, land |
| FT-04 | Fly search lanes and land |
| FT-05 | Fly altitude steps and land |
| FT-06 | Cancel navigation, hold position, land |
| FT-07 | Fly forward, return to a fixed GPS target, move north, land |

FT-01 uses `/execute_waypoint_mission`. The other operations use
`/mission_execution/{takeoff,navigate_to,follow_trajectory,orbit_point,land}` with their existing
action types. Cancellation and recovery landing use the same mission-execution boundary.
The runner subscribes directly to `/vehicle_state` for read-only measurements. It has no
vehicle-control command clients.

The routine smoke flights are FT-00, FT-01, and FT-06. The stress flights are FT-02 through FT-05.
Run each case with its own command. A failed case reports a nonzero exit status and attempts to
land when the vehicle is airborne.

FT-02 commands a heading change through `FollowTrajectory.control_yaw`. FT-03 requests
center-facing yaw through `OrbitPoint.face_center`. Both cases wait up to 15 seconds after takeoff
for valid yaw telemetry before sending the heading command. FT-04 and FT-05 command position
trajectories with yaw control disabled. A case attempts to land if a required yaw estimate remains
invalid.

FT-02 passes when the trajectory action succeeds, the vehicle visits the outbound and return
positions in order within 0.75 meters, stays within 1.5 meters of the three-dimensional path,
remains below 5 meters per second horizontal speed, and lands. The speed ceiling detects gross
overspeed relative to the 12-meter, 4-second outbound position schedule. Stopping time and
deceleration are not pass or fail criteria. The case records horizontal landing offset from its
starting position. `Land` has no target position, so this offset is diagnostic.

FT-03 records observed radial and heading errors. The orbit action enforces its requested yaw
tolerance and completes only after the requested orbit progress. FT-05 records climb and descent
settling durations; the case checks that the commanded altitudes were reached in order.

After editing flight control or ROS interfaces, stop the application, rebuild the affected
packages, source the install, and restart the application before the next case:

```bash
colcon build --symlink-install --packages-up-to flight_test vehicle_control mission_execution drone_bringup
source /workspace/install/setup.bash
```

## Reset before the next flight

Wait for the case to report landed and disarmed. If it remains airborne, land it through
QGroundControl before resetting. From a host terminal, close the simulation tmux session and
start a new one:

```bash
tmux kill-session -t drone-swarming
./scripts/sim stop
./scripts/tmux-session
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
- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/action-events.json` records complete action
  results, including measured position and yaw errors when the action provides them.
- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/action-feedback.jsonl` records timestamped
  action feedback, including trajectory tracking error and orbit radial and yaw error.
- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/vehicle-state.csv` records position, velocity,
  yaw, armed state, Offboard state, and landed state.
- `flight-tests/FT-XX-<UTC timestamp>-<suffix>/commanded-trajectory.csv` records trajectory
  setpoints when the case uses them.
- `console/` stores terminal output. `scripts/test-flights` saves a flight console log and exit
  status file.
- `ros/` stores ROS node logs. `px4/log/` stores PX4 ULogs.

The `vehicle_control` node logs rejected goal values and telemetry freshness. Failed action
results include measured progress, PX4 state, and applicable tolerances.

PX4 may reject arming when a preflight check fails. Check the PX4 console log and QGroundControl's
vehicle status. QGroundControl uses host UDP port `14550`; the flight command does not reserve
that port.

## Run package tests

Run the ROS package tests in the simulation container:

```bash
./scripts/sim-shell bash -lc 'colcon test --packages-up-to flight_test vehicle_control mission_execution drone_bringup && colcon test-result --verbose'
```

FT-03 orbits a virtual `local_enu` point. The default `vip_windy` world has no building fixture or
contact monitor. The case does not report building clearance or collision safety.

## Earlier onboard-backend validation

Validation on November 18, 2026 used PX4 v1.17.0, the project `vip_x500` model, and the default `vip_windy` world with 5 m/s wind. Preflight checks, failsafes, and flight-test tolerances remained enabled. The Docker image built with strict compiler diagnostics. Both the development workspace and an isolated container from the finished image passed 126 tests with zero errors, failures, or skips.

| Flight check | Result |
| --- | --- |
| FT-00 hover and landing | Passed |
| FT-01 uploaded square | Passed |
| FT-02 timed position and yaw | Passed |
| FT-03 finite orbit | Failed when heading validity dropped; recovery landing completed |
| FT-04 timed search lanes | Failed the ordered target-visit check; recovery landing completed |
| FT-05 altitude steps | Passed |
| FT-06 navigation cancellation | Failed the hold-drift check; recovery landing completed |
| JSON six-waypoint lawnmower | Passed |
| JSON circle | Passed uploaded takeoff, Hold-to-Offboard orbit handoff, and landing |
| Native mission interruption | Passed cancellation to Hold, zero application mission setpoints, navigation handoff, and landing preemption |
| Vehicle-control process termination | PX4 completed the stored route, landed, and disarmed |

FT-03 recorded 126 invalid heading-for-control samples during the failing orbit interval. FT-04's closest distance to its third ordered target was 0.87 meters. FT-06 measured 3.027 meters of hold drift against a 0.750-meter limit. These were the unresolved results before the coordinate migration. Current results and remaining limitations are recorded in [coordinate migration validation](COORDINATE-MIGRATION.md).

The process-loss check terminated only vehicle control with SIGKILL. Independent PX4 telemetry measured a 0.143-meter minimum horizontal waypoint error before landing. QGroundControl and the DDS agent remained running. This check does not cover complete companion-computer power loss.

Local evidence is stored under `run-logs/20261118T221233Z/`, including flight reports, ULogs, `onboard-process-loss.json`, validation scripts, and workspace/image test logs. Run artifacts are ignored by Git and are not included in a fresh clone.

Physical Raspberry Pi builds, HITL, hardware flight, multi-vehicle PX4 routing, complete Pi power loss, and native ROI heading behavior remain unvalidated. ROS pause/resume, airborne mission replacement, and perception-driven rejoin are unimplemented.

FT-07 takes off, flies 3 meters forward at the captured heading, returns to an exact GPS/AMSL target captured after takeoff, moves 2 meters north, and lands. Each observed endpoint must be within 0.75 meters. Run it with `./scripts/test-flights --case FT-07`. Geographic coordinates, frame revision, and local meter measurements are recorded in the telemetry CSV.

FT-06 now checks that navigation cancellation returns after a settled hold, including horizontal speed at most 0.3 m/s. The action has a 12-second hold-confirmation deadline; the test waits up to 15 seconds for its canceled result. The 0.75-meter drift limit is unchanged.

## Coordinate migration validation

The [migration validation record](COORDINATE-MIGRATION.md) records package tests, isolated PX4 flights, native JSON routes, clean-image checks, and remaining limits for the geographic coordinate contracts.
