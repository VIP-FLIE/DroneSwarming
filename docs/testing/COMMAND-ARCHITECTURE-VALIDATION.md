# Command architecture validation

Validation date: November 9, 2026. Branch: `learn`. Changes remain local.

## Command contract

- `/execute_waypoint_mission` retains `ExecuteWaypointMission` and the single mission-start origin.
- Public individual operations use `/mission_execution/{takeoff,navigate_to,follow_trajectory,orbit_point,land}` and the existing action types.
- Only mission execution creates application clients for `/vehicle_control/<operation>`.
- All public commands share admission, state checks, and one worker. Explicit landing interrupts ordinary flight work and drains its child before landing.
- The runner reads `/vehicle_state` directly for measurements. Its motion requests, cancellation, and recovery landing enter mission execution.
- Internal endpoint scoping expresses ownership. External DDS access control is outside this change.
- The [mission package guide](../../src/mission_execution/README.md) defines rejection, failure, cancellation, unknown child termination, and committed landing behavior.

## Automated checks

The project `drone-swarming:px4-v1.17.0` Docker image built all six workspace packages with `colcon build --symlink-install`. Project C++ targets and tests use `ProjectWarnings.cmake`, including warnings as errors and disabled GNU extensions.

The complete test suite passed. `colcon test-result --verbose` reported **77 tests, 0 errors, 0 failures, 0 skipped**. Test processes ran on ROS domain 173, isolated from the SITL vehicle's domain.

```bash
colcon build --symlink-install
ROS_DOMAIN_ID=173 colcon test
colcon test-result --verbose
```

The tests cover:

- All seven runner cases constructing command clients exclusively for mission execution.
- Actual namespaced launch ownership of six public action servers, five internal execution servers, and five mission-execution child clients, without duplicate application nodes.
- Takeoff and landing dispatch, trajectory feedback and measured failure results, navigation cancellation, orbit cancellation progress, and failed orbit hold recovery during cancellation.
- Conflicting command rejection, invalid landing rejection, explicit landing preemption, and parent cancellation waiting for the child terminal result.
- Unavailable and rejecting child servers, stale/non-finite state, state validity between mission stages, fixed mission-origin offsets, and optional mission landing.
- Late goal acceptance cancellation, bounded unconfirmed cancellation, and blocked dispatch until child termination is confirmed.
- Terminal landing telemetry arriving after an action result, and failure when that telemetry never arrives.

The bringup launch lint and formatter checks passed with Ruff 0.17.1's full stable rule set. Changed Python files passed correctness checks (`E4,E7,E9,F`) and formatter checks. An exploratory full-profile lint run against the flight runner reported existing style findings. Comparison against the original runner found no newly introduced rule/message findings; those unrelated findings remain outside this refactor.

`make diagrams` rendered both Graphviz sources to SVG and PNG. Both PNGs were inspected. `git diff --check` passed.

## SITL observations

The X500 ran in the project Docker environment with the `lawn` Gazebo world, Micro XRCE-DDS Agent, and QGroundControl. PX4 reported system ID 1, a connected GCS, passing preflight checks, and valid landed position telemetry. The application used `/fmu`, default flight-safety settings, and explicitly enabled commands for this controlled simulation. No PX4 preflight check, failsafe, freshness threshold, or flight-test threshold was changed.

The final FT-00 passed takeoff, hover, and landing. FT-01 passed the square mission after the runner's terminal-state synchronization fix. An earlier FT-01 run aborted during landing on a PX4 telemetry freshness failure and completed recovery landing. A fresh FT-01 run then completed the mission but exposed an older terminal state sample in the runner. The runner now reuses the existing 15-second landing-state wait for FT-01, with unchanged landed, disarmed, and outside-Offboard checks.

FT-06 received a canceled navigation result and completed recovery landing. Its unchanged hold-drift assertion failed at **3.962 m**, against a **0.75 m** limit. The final 4.4 seconds before recovery landing contained 45 armed Offboard samples and 88 additional setpoint publications. The physical hold-drift check remains a flight validation issue. Vehicle-control execution behavior was retained apart from endpoint names.

FT-02 passed the timed trajectory, heading, path, and landing checks. FT-03 reached vehicle control through mission execution, then the controller aborted the orbit before the requested cancellation progress: yaw error stayed outside its 0.523599-radian tolerance for two seconds (reported error 0.601407 radians). Vehicle control restored and confirmed Offboard hold at 0.17 m position error, and the runner completed recovery landing through mission execution. Live orbit cancellation/completion remains unverified because that entry failed; automated action tests cover cancellation and failure propagation. The yaw tolerance was retained.

Every row uses `flight_test -> mission_execution -> vehicle_control`. Individual operation endpoints use the `mission_execution/` prefix. FT-01 uses the unchanged mission endpoint. All recovery landings use `mission_execution/land`.

| Case | Command flow | Latest result | Report |
| --- | --- | --- | --- |
| FT-00 | Takeoff, hover, Land | passed | [Case report](../../run-logs/20261109T233409Z/flight-tests/FT-00-20261109T234207Z-mlx76_53/case-report.md) |
| FT-01 | ExecuteWaypointMission, child Takeoff/NavigateTo/Land | passed | [Case report](../../run-logs/20261109T234412Z/flight-tests/FT-01-20261109T234629Z-vbro5f86/case-report.md) |
| FT-02 | Takeoff, FollowTrajectory with yaw, Land | passed | [Case report](../../run-logs/20261109T234412Z/flight-tests/FT-02-20261109T234840Z-qiz3eby_/case-report.md) |
| FT-03 | Takeoff, OrbitPoint, cancellation/hold/completion checks, Land | failed | [Case report](../../run-logs/20261109T234412Z/flight-tests/FT-03-20261109T234911Z-t872ie4o/case-report.md) |
| FT-04 | Takeoff, FollowTrajectory search lanes, Land | passed | [Case report](../../run-logs/20261109T234412Z/flight-tests/FT-04-20261109T235052Z-0a9akoie/case-report.md) |
| FT-05 | Takeoff, FollowTrajectory altitude steps, Land | passed | [Case report](../../run-logs/20261109T234412Z/flight-tests/FT-05-20261109T235129Z-afl2cy49/case-report.md) |
| FT-06 | Takeoff, NavigateTo cancellation/hold, Land | failed | [Case report](../../run-logs/20261109T234412Z/flight-tests/FT-06-20261109T234654Z-3s5n8a_z/case-report.md) |

FT-00, FT-01, FT-02, FT-04, and FT-05 passed. FT-03 and FT-06 failed and completed recovery landing. The latest FT-03 run did not reach live orbit cancellation or completion because its yaw check aborted first. The current flight suite is not fully validated.

Build and automated-test logs: [build](../../run-logs/20261109T233409Z/console/colcon-build.log), [tests](../../run-logs/20261109T233409Z/console/colcon-test.log), [test-result summary](../../run-logs/20261109T233409Z/console/colcon-test-result.log). Flight telemetry, feedback, action events, and node/PX4 logs remain under the two run directories linked above.
## Scope and unresolved policy

The [architecture guide](../architecture/README.md) and separate current/planned diagrams document the subsequent JSON planner and future swarm/operator special instructions, and perception results entering mission execution. The planned priority order is safety landing, urgent special instructions, then the normal planned route.

That validation phase did not implement JSON planning. The subsequent [JSON planner validation](JSON-PLANNER-VALIDATION.md) records its implementation and checks. Swarm runtime/protocol, perception runtime, and future arbitration policy remain unimplemented. Plan replacement, source authorization, equal-priority ordering, instruction expiry, detection confidence, and interrupted-route resume remain future design decisions.

Assumptions: existing action types remain the current operation contract; public landing commits at admission; telemetry is application input with validity and freshness constraints. Hardware validation was not run. Standards conflicts: none.
