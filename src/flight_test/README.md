# Flight test suite

Use the [flight test guide](../../docs/testing/README.md) for run commands, prerequisites, and
result locations.

`scripts/test-flights` builds the affected ROS packages and runs selected PX4 SITL cases in Gazebo.
The command starts a host-persisted run, launches PX4 and the XRCE-DDS Agent in the simulation
container, starts the application with flight actions enabled, and shuts down only processes it
started. Gazebo runs in PX4 headless mode.

## Commands

Run the routine smoke cases:

```bash
./scripts/test-flights --profile smoke
```

Run the stress cases:

```bash
./scripts/test-flights --profile stress
```

Run one case:

```bash
./scripts/test-flights --case FT-03
```

The wrapper creates a run directory through `scripts/new-run`. It records the active commit and
dirty-tree state, builds `flight_test`, `vehicle_control`, `mission_execution`, and
`drone_bringup`, and returns nonzero when a build, process, action, assertion, cleanup, or report
step fails.

## Profiles and cases

| Profile | Cases |
| --- | --- |
| `smoke` | FT-00 hover and land, FT-01 square mission, FT-06 navigation cancellation and hold |
| `stress` | FT-02 fast stop and return, FT-03 orbit and yaw tracking, FT-04 search lanes, FT-05 altitude steps |

FT-03 first cancels an orbit after measured angular progress and checks that PX4 restores Offboard
hold with a continuing setpoint stream. It then completes a full virtual `local_enu` orbit and
measures angular progress, radial tracking, and yaw toward the center. The stock Gazebo world has
no building fixture or contact monitor, so FT-03 does not grade physical clearance or collision
events.

## Run artifacts

Each run is stored under `run-logs/<run-id>/`:

- `run-info.txt` records the host commit, branch, and dirty-tree state.
- `flight-tests/flight-test-run.txt` records the selected cases and grading commit.
- `flight-tests/<case-id>/vehicle-state.csv` records timestamped ENU position, velocity, yaw,
  arming, Offboard, landing, and setpoint publication state.
- `flight-tests/<case-id>/commanded-trajectory.csv` records timed trajectory setpoints.
- `flight-tests/<case-id>/action-events.json` records action goals and terminal results.
- `flight-tests/<case-id>/assertions.json` and `case-report.md` record case metrics and outcomes.
- `flight-tests/<case-id>/case-result.json` records the runner result.
- `flight-tests/<case-id>/process-result.json` records process exit codes and run metadata.
- `flight-tests/<profile>-profile.md` links the case reports and process results.
- `console/` stores captured build and suite output. `px4/log/` stores PX4 ULog files.
  Preexisting rootfs logs are preserved under `px4/preexisting-*` before log routing.

The command requires Docker Compose, the project simulation image, and the PX4 v1.17.0/Gazebo
Harmonic workspace. The suite uses ROS domain 77, PX4 topic prefix `/fmu`, and system ID 1. It
refuses to start if the required PX4 UDP ports are occupied or another suite owns the PX4 rootfs
lock. It does not run on connected hardware.
