# Coordinate migration validation

Validation on November 18, 2026, Eastern time used the `px4-offload` working tree, ROS 2 Jazzy, PX4/`px4_msgs` v1.17.0, MAVSDK 4.2.0, and Gazebo Harmonic. The simulator used the project `vip_x500` model and `vip_windy` world with 5 m/s wind. PX4 preflight checks and failsafes remained enabled.

The validation container used a private Docker bridge network, ROS domain 93, headless Gazebo, and a simulated MAVLink ground-station heartbeat. An existing SAR simulation occupied the host's default MAVLink ports. Initial host-network setup attempts are excluded from flight evidence. The existing SAR session was preserved.

## Workspace checks

The complete workspace builds with project compiler diagnostics, warnings as errors, C++17, and GNU language extensions disabled. `colcon test-result --verbose` reports 154 tests, zero errors, zero failures, and zero skips. Coverage includes geographic waypoint preservation, forward/left axis rotation, shared routes across estimator origins, unavailable GPS/heading rejection, projection boundaries, local-origin resets, mission admission, landing preemption, cancellation, and land-detector freshness. The cancellation mock waits for ROS to enter its canceling state before publishing its terminal result; the landing-preemption case also passed 20 consecutive repetitions.

Evidence paths refer to local `run-logs/` artifacts, which Git ignores.

Evidence:

- [Full build log](../../run-logs/20261119T023000Z/coordinate-validation/build.log)
- [Workspace test log](../../run-logs/20261119T023000Z/coordinate-validation/workspace-tests.log)
- [Test-result summary](../../run-logs/20261119T023000Z/coordinate-validation/workspace-test-results.txt)
- [Repeated landing-preemption test](../../run-logs/20261119T020500Z/coordinate-validation/cancellation-stress.log)

Reproduce the workspace checks in a sourced project container:

```bash
colcon build --symlink-install
source /workspace/install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

All six JSON examples compile with valid geographic start telemetry. GPS and shared-origin example coordinates match the zero-elevation `vip_windy` world. Hardware deployments require their own site positions and AMSL heights. Body-relative JSON requires a valid landed heading; standalone body-relative navigation can start after takeoff.

## Clean image

`docker compose --progress=plain build development` completed with the new package and ROS interfaces. The resulting image is also tagged locally as `drone-swarming:coordinates-20261119`. A new container from that image, with a private bridge network and no mounted source or build volumes, passed all 154 package tests with zero errors, failures, or skips.

- [Image build log](../../run-logs/20261119T023000Z/coordinate-validation/image-build.log)
- [Clean-image test log](../../run-logs/20261119T023000Z/coordinate-validation/clean-image-tests.log)

Reproduce the clean-image check:

```bash
docker compose --progress=plain build development
docker run --rm --network bridge drone-swarming:px4-v1.17.0 bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/px4_ros2/install/setup.bash && source /workspace/install/setup.bash && cd /workspace && colcon test && colcon test-result --verbose'
```

## Flight cases

| Case | Result | Evidence |
| --- | --- | --- |
| FT-00 hover and landing | Passed | [Report](../../run-logs/20261119T020500Z/coordinate-validation/FT-00/flight/case-report.md) |
| FT-01 uploaded square | Passed | [Report](../../run-logs/20261119T020500Z/coordinate-validation/FT-01/flight/case-report.md) |
| FT-02 timed position and yaw | Passed | [Report](../../run-logs/20261119T020500Z/coordinate-validation/FT-02/flight/case-report.md) |
| FT-03 finite orbit and landing | Passed after land-state freshness correction | [Report](../../run-logs/20261119T023000Z/coordinate-validation/FT-03/flight/case-report.md) |
| FT-04 timed search lanes | Failed: third ordered target missed by 0.88 m against a 0.75 m limit; recovery landing completed | [Report](../../run-logs/20261119T020500Z/coordinate-validation/FT-04/flight/case-report.md) |
| FT-05 altitude steps | Passed | [Report](../../run-logs/20261119T020500Z/coordinate-validation/FT-05/flight/case-report.md) |
| FT-06 navigation cancellation | Passed after settled-hold confirmation was added | [Report](../../run-logs/20261119T023000Z/coordinate-validation/FT-06/flight/case-report.md) |
| FT-07 forward, GPS return, and north-relative navigation | Passed | [Report](../../run-logs/20261119T023000Z/coordinate-validation/FT-07/flight/case-report.md) |

FT-06 retains the 0.75-meter hold-drift limit and now checks horizontal speed at most 0.3 m/s when the canceled result arrives. The controller confirms position and velocity settling for 0.5 seconds before reporting canceled. Failure to confirm hold within 12 seconds returns aborted. FT-07 checks the three observed endpoints against a 0.75-meter limit and verifies landed/disarmed completion.

The FT-03 rerun initially rejected landing when unchanged land-detector telemetry crossed the old 1-second expiry. PX4 republishes unchanged land state once per second. Land-state freshness now permits at least 2 seconds; position and status freshness retain their configured 1-second default. A regression test verifies that a 1.3-second-old land sample remains usable and a 2.3-second-old sample expires.

Run each case in a separate controlled simulation session using the [flight-test procedure](README.md):

```bash
./scripts/test-flights --case FT-07
```

## Native JSON route checks

| Input | Result | Evidence |
| --- | --- | --- |
| GPS fleet example, `uav_1` | Passed; 2 geographic waypoints, landed and disarmed | [Result](../../run-logs/20261119T023000Z/coordinate-validation/JSON-GPS/flight/case-result.json) |
| Lawnmower example | Passed; 8 geographic waypoints, landed and disarmed | [Result](../../run-logs/20261119T023000Z/coordinate-validation/JSON-LANES/flight/case-result.json) |
| Relative ENU circle, radius 3 m and speed 1 m/s | Passed; native takeoff, geographic-center orbit handoff, one revolution, and landing | [Result](../../run-logs/20261119T023000Z/coordinate-validation/JSON-CIRCLE/flight/case-result.json) |

GPS and lawnmower runs recorded zero Offboard setpoint publications for the complete route. The circle run recorded 224 publications for its retained local orbit approach and hold recovery. Each result directory contains the submitted JSON plan. These checks verify uploaded mission execution and native completion; they do not apply FT-04's timed-trajectory tracking metric.

Reproduce the GPS and lawnmower routes in separate controlled simulation sessions:

```bash
ros2 run mission_planner submit_flight_plan /workspace/src/mission_planner/examples/gps-fleet.json
ros2 run mission_planner submit_flight_plan /workspace/src/mission_planner/examples/lawnmower.json
```

## Remaining validation limits

FT-04 remains a failed timed-trajectory tracking check. Its schedule, ordered-target metric, and 0.75-meter tolerance were retained. The full flight suite has not passed. Package-test success does not establish physical path accuracy or collision clearance.

Physical hardware, Raspberry Pi runtime flight, simultaneous multi-vehicle PX4 flight, GPS accuracy under field conditions, inter-vehicle separation, and obstacle clearance remain unvalidated. The implementation supplies GPS-based navigation and coordinate transforms; it supplies no vision or obstacle-avoidance system. Flight commands remain disabled by default.

The [coordinate guide](../architecture/COORDINATES.md) defines the contracts, public commands, and literature basis.
