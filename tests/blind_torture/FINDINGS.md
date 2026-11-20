# Blind test observations, November 19, 2026

The final 34-scenario run used seed `20261119` and fuzz count `64`. It completed with 27 scenarios passing and 7 failing. Artifacts are in `run-logs/blind-final-20261119/`. `manifest.json` contains the test-file hashes. `results.json` and `junit.xml` contain the scenario outcomes. Each scenario directory contains `wire.jsonl`, `worker.log`, and node logs.

The application implementation and existing tests were not read. These findings describe ROS-visible behavior. They do not identify an implementation cause.

## Executor limit probes

`test_malformed_goals_never_reach_children` observed child dispatch and successful public results for four probes against a synthetic child that reports success:

| Goal | Value |
| --- | --- |
| `OrbitPoint.timeout_sec` | 3,601 seconds |
| `Takeoff.altitude_m` | 121 meters |
| `OrbitPoint.radius_m` | 21 meters |
| `OrbitPoint.tangential_speed_mps` | 3.1 meters per second |

The documented standalone timeout ceiling is 3,600 seconds. The documented default vehicle orbit limits are 20 meters and 3 meters per second. Requiring the executor to enforce vehicle bounds before dispatch is a harness assumption. The 121-meter standalone takeoff probe extrapolates the documented 120-meter mission envelope; the standalone takeoff ceiling requires contract clarification. A real vehicle backend can independently reject these requests. The test establishes what the executor forwards and reports with a permissive peer.

## Flood reply deadlines

`test_concurrent_goal_flood_admits_one` submitted 32 ordinary action goals. At least one goal response did not arrive within the 5-second harness deadline. The test could not establish the complete admission count.

`test_seeded_invalid_flood_and_recovery` submitted 64 schema-invalid planning requests concurrently. At least one service response did not arrive within the shared 10-second deadline. The scenario failed before its post-flood canary.

Both tests use default ROS service QoS. These observations do not distinguish node callback backlog, DDS history limits, or client-side response delivery. They are liveness findings under the stated budgets.

## Unsolicited PX4 output

Four vehicle-control scenarios failed their no-output assertions under fresh synthetic airborne Offboard telemetry:

- Default flight commands disabled.
- Telemetry conversion followed by a stale stream.
- Poisoned telemetry fields followed by valid-field recovery.
- Rejected navigation goals with fresh telemetry.

The probe observed `TrajectorySetpoint` and `OffboardControlMode` messages without an accepted flight operation. The initial observed setpoint used the local origin, while synthetic local telemetry represented ENU position `(5, 2, 3)` meters. `wire.jsonl` records every observed output. These scenarios did not observe an autopilot executing a command. The no-output assertion treats unsolicited position control as a safety-contract finding.

The conversion, poisoned-validity, and stale-validity assertions passed before the no-output assertions failed. Enabled actions without telemetry and invalid geographic uploads passed their rejection and no-output checks.

## Validation

- The five packages through `mission_execution` built in the project container.
- Seed `42` with fuzz count `512` passed the randomized valid-route oracle and reproduced the planning flood reply-deadline failure. Artifacts are in `run-logs/blind-seed-42-512/`.
- The complete run's peak sampled aggregate resident memory was 183.1 MiB and peak sampled thread count was 49, within the harness budgets.
- Ruff `0.17.1` passed the repository's full stable rule profile with documented unittest-specific exceptions.
- Python syntax compilation passed.
- Harness controls verified a successful worker, an intentional assertion failure, and a hung worker with a sleeping descendant. The watchdog failed the hung scenario and stopped its live descendant.
- Test imports contain no planner, executor, vehicle-control, or geometry implementation imports.
- Cancellation draining, failed cancellation diagnostics, landing ordering, unavailable children, late child acceptance, rejected and failed children, unknown child outcomes, and late planner responses passed.
- The real planner/executor handoff, geographic target preservation across local revisions, independent coordinate oracle, JSON limits, and tiny-spacing expansion rejection passed.

No SITL flight, physical vehicle, DDS serialization fuzzer, or complete workspace `colcon test` run was performed. Production behavior was not changed. The synthetic child confirms ordering and reported results; it does not establish physical hold or landing.
