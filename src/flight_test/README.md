# Individual flight tests

The [flight test guide](../../docs/testing/README.md) gives the setup, run, reset, and log
inspection commands. Each case sends application ROS actions to an already running PX4 SITL
simulation. The package does not start, stop, or reset PX4, Gazebo, or the application.

Run one case from pane 3 of `scripts/tmux-session` or `scripts/tmux-session start`:

```bash
./scripts/test-flights --case FT-00
```

The script builds `flight_test` and its dependencies inside the simulation container if the
package is missing or cannot import. Once it is installed, the ROS command also runs one case:

```bash
ros2 run flight_test flight_test_case --case FT-00
```

Each invocation writes a separate directory under the active run's `flight-tests/` directory.
The runner records `vehicle-state.csv`, `commanded-trajectory.csv`, `action-events.json`,
`assertions.json`, `case-result.json`, and `case-report.md`. The `DRONE_RUN_DIR` environment
variable selects the active run. A caller may provide `--artifact-dir` to the ROS command to
create one explicitly named directory.

| Case | Flight sequence |
| --- | --- |
| FT-00 | Take off, hover, land |
| FT-01 | Fly the launch-relative square mission, land |
| FT-02 | Visit an outbound point, hold it, turn, return, land |
| FT-03 | Cancel an orbit, hold, complete an orbit, land |
| FT-04 | Fly search lanes, land |
| FT-05 | Fly altitude steps, land |
| FT-06 | Cancel navigation, hold, land |

The runner waits for fresh landed `local_enu` telemetry and the required action servers before
sending the first goal. Each case checks its own measured outcomes and ends landed, disarmed, and
outside Offboard. A failed case attempts a landing action if the vehicle is airborne or remains in
Offboard. The next case can use the same running simulation; its takeoff action starts a new
one-second setpoint prestream and requests Offboard when needed.

FT-02 and FT-03 wait for valid yaw after takeoff before requesting yaw control. The wait ends after
15 seconds if PX4 does not report yaw validity. FT-04 and FT-05 command position trajectories
without yaw control. Flight results are written to the host-mounted run directory with
host-readable directory permissions.

FT-02 checks a 1.5-meter corridor around the outbound and return path, visits to both endpoints
in order within 0.75 meters, and a 5-meter-per-second observed horizontal speed ceiling. The
`FollowTrajectory` action checks its final position and requested yaw tolerance. The runner records
the horizontal landing offset from the starting position. It does not require a specific stopping
time or deceleration profile.

FT-03 records radial and heading error distributions. The `OrbitPoint` action enforces its orbit
progress and requested center-facing yaw tolerance. FT-05 records climb and descent settling
durations without requiring a fixed settling time.

FT-03 uses a virtual `local_enu` orbit center. The stock Gazebo world has no building fixture or
contact monitor. The case makes no building-clearance or collision-safety claim.
