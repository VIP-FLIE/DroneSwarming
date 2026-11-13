# JSON planner validation

Validation date: November 9, 2026, local time; artifacts use November 10 UTC. Branch: `learn`. Changes remain local.

## Implementation

The parser, lawnmower geometry, legacy file format, file CLI behavior, and focused tests are adapted from local `sar-drone` commit `6be553ded796ba18d7bb18dad6cd39b5dc607506`. The SAR source repository was read without modification. Its BSD license is retained in `src/mission_planner/LICENSE.sar-drone` and installed with the package.

JSON content enters `mission_execution/execute_flight_plan`. The executor captures one fresh `VehicleState`, calls `mission_planner/plan_flight` with its stable vehicle ID, validates the translated response, and sequences existing vehicle-control actions. The planner has no flight-action clients. Flight commands remain disabled by default.

Version 1 retains SAR absolute local-origin positions and altitude. Version 2 adds explicit fleet assignments, per-vehicle relative ENU, shared geographic origins, and GPS latitude/longitude with AMSL altitude. Geographic translation uses validity-gated application telemetry and aborts when the captured local-frame revision changes. See the [schema guide](../../src/mission_planner/README.md) and [extension plan](../architecture/JSON-FLIGHT-PLANS.md).

## Automated checks

The project `drone-swarming:px4-v1.17.0` Docker image built all seven workspace packages with `colcon build --symlink-install`. C++ targets and tests passed the repository warning profile with warnings treated as errors and GNU extensions disabled.

The complete suite passed: **119 tests, 0 errors, 0 failures, 0 skipped**. Tests used ROS domain 173; the PX4 adapter test retains its own domain 77. Flight traffic used domain 0.

```bash
colcon build --symlink-install
ROS_DOMAIN_ID=173 colcon test
colcon test-result --verbose
```

Coverage includes:

- Forty pure parser/compiler/geometry cases covering SAR compatibility, nonzero legacy origin, fixed mission-start offsets, explicit assignment selection, duplicate IDs, malformed JSON, non-finite values, coordinate bounds, orbit bounds, convex area validation, and generated waypoint limits.
- Geographic axis and round-trip checks at the equator, high latitude, and the date line; GPS/AMSL conversion; shared geographic anchors; identical GPS lawnmower routes across distinct vehicle references; and remote unselected assignments.
- Nine integration cases using real C++ executors and Python planner nodes with action backends in two namespaces: separate relative assignments, one GPS destination across two frames, invalid JSON, cancellation drain, frame reset, landing preemption, unavailable planner, invalid translated output, and a late translation after landing interrupts planning.
- Actual namespaced launch ownership of three nodes, seven public action servers, five internal execution servers, five executor child-action clients, and no planner action clients.
- Adapter revision changes on reference changes and estimator resets, with no revision change for ordinary position updates.
- Existing mission authority, runner routing, trajectory, orbit, cancellation, failure, and landing tests.

An initial integration run exposed a fixture race during service replacement. The fixture now injects response callbacks into the existing planner service. The affected bringup tests and subsequent complete suite passed. The executor timed out without dispatching takeoff during that failed fixture run.

Ruff 0.17.1 correctness checks (`E4,E7,E9,F`) and formatting passed for the new Python package and bringup tests. Launch lint passed the repository's full stable Ruff rule set and formatter. `make diagrams` rendered both Graphviz sources, and both PNGs were inspected. `git diff --check` passed.

Logs: [build](../../run-logs/20261110T003922Z/console/colcon-build.log), [tests](../../run-logs/20261110T003922Z/console/colcon-test.log), [test results](../../run-logs/20261110T003922Z/console/colcon-test-result.log).

## SITL flights

The X500 ran in the project container with the Gazebo `lawn` world and Micro XRCE-DDS Agent. The existing host QGroundControl instance supplied the connected GCS. PX4 reported system ID 1, passing preflight checks, and no failsafe before commands were enabled for this controlled session. PX4 checks, failsafes, telemetry timeouts, and flight-test thresholds were retained.

Every flight used `submit_flight_plan -> mission_execution -> mission_planner -> mission_execution -> vehicle_control -> PX4`. Each CLI returned success after mission completion. Terminal telemetry confirmed landed, disarmed, and outside Offboard.

| File/coordinates | Executed route | Result | Evidence |
| --- | --- | --- | --- |
| Shipped version 2 relative fleet file, `uav_1` assignment | Four waypoints forming a 3-meter square at a 3-meter climb, then landing | passed | [CLI log](../../run-logs/20261110T003922Z/console/relative-plan.log), [landed state captured before GPS](../../run-logs/20261110T003922Z/state-before-gps.json) |
| Generated version 2 absolute GPS file | Three geographic destinations corresponding to offsets `(2,0)`, `(2,2)`, `(0,0)` at AMSL height 3 meters above the captured start, then landing | passed | [GPS file](../../run-logs/20261110T003922Z/gps-sitl-plan.json), [compiled offsets](../../run-logs/20261110T003922Z/console/gps-compilation.log), [CLI log](../../run-logs/20261110T003922Z/console/gps-plan.log), [landed state](../../run-logs/20261110T003922Z/console/state-after-gps.yaml) |
| Shipped version 1 SAR triangle | Absolute local-origin triangle `(0,0)`, `(8,0)`, `(4,7)`, `(0,0)` at local Z 5 meters, then landing | passed | [CLI log](../../run-logs/20261110T003922Z/console/legacy-plan.log), [landed state](../../run-logs/20261110T003922Z/console/state-after-legacy.yaml) |

The GPS validation file was generated from the live `VehicleState` reference rather than an assumed simulator home or altitude. Its [generation script](../../run-logs/20261110T003922Z/create-gps-plan.py) and [starting state](../../run-logs/20261110T003922Z/state-before-gps.json) are retained. Node logs, PX4 console output, and ULog files remain under [the run directory](../../run-logs/20261110T003922Z).

## Limits and assumptions

- Fleet assignments and constant-altitude paths are authored explicitly. GPS heights use AMSL, and GPS coordinate pairs use latitude first.
- Fleet delivery is a separate submission to each configured namespace. Automatic allocation, synchronized starts, peer health, collision avoidance, and runtime replanning are unimplemented.
- Two-drone translation and execution were tested with simulated ROS action backends. Multi-vehicle PX4 flight and physical hardware validation were not run.
- The new JSON entry point was flown with relative, GPS, and legacy waypoint routes. JSON circle and lawnmower flight were not run in this validation session; deterministic planning and existing execution tests cover those paths.
- The earlier FT-03 orbit-yaw failure and FT-06 cancellation hold-drift failure remain recorded in [command architecture validation](COMMAND-ARCHITECTURE-VALIDATION.md). This change does not establish a passing full flight suite.
- Standards conflicts: none. Existing feature-branch and local-only instructions take precedence over the repository's general push/PR workflow.
