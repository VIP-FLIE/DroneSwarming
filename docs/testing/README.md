# Run the flight test suite

## Start the flight tests

Open a terminal in the `DroneSwarming` repository root. Keep Docker running. Run both profiles to
exercise all seven flight cases:

```bash
./scripts/test-flights --profile smoke
./scripts/test-flights --profile stress
```

Each command creates a separate run ID. A profile stops after its first failed case. Run the other
profile separately to collect its results even when the first profile fails.

The launcher starts the `simulation` Compose service, builds the flight-test ROS packages, then
starts an owned XRCE-DDS Agent, PX4/Gazebo, the drone application, and the case runner. The
simulation runs headless and the launcher manages its processes.

Before starting, make sure:

- Docker Compose is installed and the Docker daemon is running.
- Host UDP ports `8888` and `14550` are available.
- The repository's `run-logs/` directory is writable by the current user.

The suite runs PX4 SITL in simulation. It does not connect to a physical drone.

## Select a profile or case

| Command | Cases |
| --- | --- |
| `./scripts/test-flights --profile smoke` | FT-00 hover and landing, FT-01 square mission, FT-06 navigation cancellation and hold |
| `./scripts/test-flights --profile stress` | FT-02 fast stop and return, FT-03 orbit and yaw, FT-04 search lanes, FT-05 altitude steps |

Run one case while debugging:

```bash
./scripts/test-flights --case FT-03
```

The runner prints the run ID and artifact directory. A zero exit status means every requested case
passed. A nonzero exit status means setup, build, a case, process cleanup, or report generation
failed.

## Read a run result

Runs are stored under `run-logs/<run-id>/`:

- `flight-tests/smoke-profile.md` or `flight-tests/stress-profile.md` gives the profile result.
- `flight-tests/<case-id>/case-report.md` gives measurements and assertions for one case.
- `flight-tests/<case-id>/case-result.json` records the runner outcome and failure detail.
- `flight-tests/<case-id>/process-result.json` records process exit codes and simulation settings.
- `flight-tests/<case-id>/vehicle-state.csv` records observed position, velocity, yaw, and vehicle
  state.
- `flight-tests/<case-id>/action-events.json` records action goals and terminal results.
- `flight-tests/<case-id>/commanded-trajectory.csv` records trajectory setpoints when used.
- `console/` contains host-side build and suite output.
- `px4/log/` contains ULogs from the run. Unarchived PX4 logs from before the run are preserved
  under `px4/preexisting-*`.

## Run ROS package tests

The flight launcher builds the affected packages. It does not run `colcon test`. Run ROS package
tests in the simulation container with:

```bash
docker compose up -d simulation
docker compose exec -T simulation bash -lc '
set -e
cd /workspace
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
colcon build --symlink-install \
  --packages-up-to flight_test vehicle_control mission_execution drone_bringup \
  --event-handlers console_direct+
source /workspace/install/setup.bash
colcon test \
  --packages-up-to flight_test vehicle_control mission_execution drone_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
'
```

Run the flight-test Python checks on the host with:

```bash
cd src/flight_test
PYTHONPATH=. python3 -m pytest -q
```

## Resolve common setup failures

If the runner reports that UDP port `8888` or `14550` is occupied, close the SITL or XRCE session
started for this project and retry. The suite refuses to share those ports with another process.

PX4 may reject arming when its preflight checks fail. Read the case report, `runner.log`, and
`px4-sitl.log` in the run directory. The suite preserves PX4 preflight checks and reports the
failed action.

FT-03 orbits a virtual `local_enu` point. The stock world has no building fixture or contact
monitor, so the case does not report building clearance or collision safety. See the [suite
case descriptions](../../src/flight_test/README.md) for the current case set.
