# Single-drone readiness and transition to multi-drone flight

Review date: November 19, 2026, Eastern time. Reviewed checkout: `torture-test`, commit `d8c2b17a9c983da696db32cbdb13876870bc8b0d`, including the local blind and seeing torture suites and available `run-logs/` artifacts. This document proposes acceptance criteria; it records no new flight qualification.

## Readiness decision

**The reviewed system has unresolved blockers for autonomous physical flight.** Continue isolated simulation, native Raspberry Pi build work, and supervised props-off integration. Close the applicable software and integration gates below before enabling autonomous flight on a physical vehicle.

The project already has uploaded PX4 waypoint missions, a planning-only JSON compiler, application command serialization, landing preemption, geographic coordinate contracts, and recorded SITL flights. Recorded adversarial tests expose unsafe publication, numeric conversion, state-transition, and result-validation paths. Physical companion-computer operation and flight remain unvalidated.

Use four separate acceptance decisions:

| Gate | Meaning | Current evidence |
| --- | --- | --- |
| G0: props-off integration | The team can inspect and test a physical controller and Pi with propellers removed and a written bench procedure. | Build instructions exist. Native arm64 build and link verification remain open. |
| G1: first autonomous hardware flight | One approved vehicle, site, software revision, parameter profile, and restricted flight sequence have passed software, integration, and recovery checks. | Blocked by the safety findings and missing hardware evidence below. |
| G2: single-drone baseline complete | Every retained flight capability meets its contract, adverse-event behavior is measured, and repeated physical missions meet the approved envelope. | Blocked. FT-04 and adversarial findings remain unresolved. |
| G3: two-drone flight development | G2 is complete, and identity, separation, recovery conflicts, and peer-loss requirements have been defined and tested in two-vehicle SITL. | Coordination is an empty scaffold. Two-vehicle physical flight is a later gate. |

G1 may approve a smaller capability set than G2. Excluded operations must be rejected by the deployment configuration or code, covered by a rejection test, and listed on the flight card. Choosing not to call an exposed operation does not close its defect. Shared publication, identity, and recovery defects block every autonomous flight profile.

## Mission scope

The [fall 2026 project proposal](https://docs.google.com/document/d/1RS-wXLY9SmiPJJIe6-eLTf87f_TJj5O6yl37zzr4kRg/edit) identifies physical single-drone waypoint flight without object detection as an initial deliverable. Its later SAR goal includes task delegation, navigation through clutter, target identification, and reporting to ground control. Those are separate capability claims.

Define the first baseline as an outdoor, GPS/GNSS-based multicopter flying a surveyed, obstacle-free route under pilot supervision. Record the airframe, payload, weather limits, geographic boundary, altitude datum, speed limits, endurance reserve, and recovery locations. The current implementation supplies no obstacle sensing, obstacle avoidance, victim detection, or terrain following. A lawnmower route establishes flight geometry; SAR search performance also requires a sensing and detection model.

Retain these implemented boundaries:

| Existing design | Required constraint |
| --- | --- |
| Planner computes routes without flight clients. | Malformed plans and planning failures must have zero flight side effects. |
| `mission_execution` admits public intent and serializes child operations. | Add future operator and swarm intent at this boundary. |
| `vehicle_control` owns PX4 types, DDS, and MAVLink mission upload. | Keep PX4 details out of the planner and coordinator. |
| Geographic routes retain WGS84 latitude/longitude and explicit AMSL height. | Resolve relative input once; preserve destinations across local estimator resets. |
| PX4 executes uploaded waypoint missions. | Keep low-level stabilization, preflight checks, and failsafes in PX4. |
| Standalone navigation, timed trajectories, and orbit handoffs retain local operations. | Apply a common ownership and fault policy to every retained operation. |

Sources: [architecture](../architecture/README.md), [onboard mission contract](../architecture/PX4-ONBOARD-MISSIONS.md), [coordinate contract](../architecture/COORDINATES.md), and [planner contract](../../src/mission_planner/README.md).

## What the existing evidence establishes

These entries describe recorded runs. Package-test counts and torture-scenario counts measure different test sets and must remain separate.

| Evidence | Recorded result | Limit |
| --- | --- | --- |
| [Coordinate migration validation](COORDINATE-MIGRATION.md), [workspace summary](../../run-logs/20261119T023000Z/coordinate-validation/workspace-test-results.txt), and [clean-image log](../../run-logs/20261119T023000Z/coordinate-validation/clean-image-tests.log) | 154 package tests, zero errors, failures, or skips; workspace and isolated image checks recorded. | Historical test success does not close later adversarial findings. |
| Coordinate migration SITL | FT-00, FT-01, FT-02, FT-03, FT-05, FT-06, and FT-07 have passing records, including corrected FT-03 and FT-06 reruns. | Results span several sessions. They do not constitute a repeated full-suite qualification on the final hardware profile. |
| [FT-04 report](../../run-logs/20261119T020500Z/coordinate-validation/FT-04/flight/case-report.md) | Failed: third ordered target missed; closest remaining error 0.88 m against 0.75 m. Recovery landing completed. | The timed-trajectory flight suite has not passed. Native lawnmower success uses a different execution path and metric. |
| [Blind final results](../../run-logs/blind-final-20261119/results.json) and [findings](../../tests/blind_torture/FINDINGS.md) | 34 scenarios, 27 passed and 7 failed, seed `20261119`, fuzz count 64. | Synthetic peers establish ROS-visible behavior, not physical vehicle response. |
| [Seeing initial full report](../../run-logs/seeing-full-initial/report.json) | 36 scenarios: 16 passed, 18 failed, 2 harness errors, zero supervisor timeouts. | This is the initial run, not a final qualification result. |
| [Seeing pipeline report](../../run-logs/seeing-pipeline-initial/report.json) | 5 scenarios: 3 passed and 2 failed. Public-path non-finite setpoint and trajectory-reset checks failed. | Real executor and controller ran against synthetic PX4 telemetry. |
| [Seeing orbit report](../../run-logs/seeing-orbit-initial/report.json) | 3 scenarios: 2 passed and 1 failed while waiting for the action result. | The teleport scenario does not prove that a false revolution was reported successfully. |
| [Seeing positive controls](../../run-logs/seeing-positive-3/report.json) and [takeoff harness correction](../../run-logs/seeing-takeoff-fixed/report.json) | Three positive controls passed; the two corrected takeoff scenarios passed. | Earlier positive-control runs failed during harness development. Do not attribute the two initial takeoff errors to production code. |
| [Stored-route process-loss result](../../run-logs/20261118T221233Z/onboard-process-loss.json) and [interruption result](../../run-logs/20261118T221233Z/onboard-interruptions.json) | Vehicle-control SIGKILL was followed by stored-route completion, landing, and disarming. Native cancellation and landing preemption passed in that session. | The DDS agent and ground station remained available. Complete Pi power loss and combined link loss were not established. |

The seeing reports' recorded production source hashes match the current corresponding files. Source hashes alone do not prove which installed libraries or binaries were loaded. One saved planner scenario log names a `1e300` snapshot probe that is absent from the current snapshot corpus, despite the aggregate report's current suite hashes. Preserve this as an evidence-provenance discrepancy and rerun it from an immutable test snapshot. Do not classify it as acceptance of NaN telemetry.

`run-logs/` is ignored by Git. Archive the cited evidence with the release record; these links will be absent in a fresh clone. The blind and seeing suites and `scripts/test-seeing-torture` were untracked at review time and require review and inclusion in the reproducible test baseline.

## Software work required before flight

P0 items block G1 wherever the affected behavior is reachable. P1 items block G2 and any G1 profile that depends on them. Each item needs an assigned maintainer, a reviewer, a linked change, and evidence from the final revision.

### R1. Make setpoint publication depend on acquired control ownership. P0

**Confirmed defect.** In [`publish_control_setpoint`](../../src/vehicle_control/src/vehicle_control_node.cpp), publication is enabled by `stream_requested_ || (telemetry.offboard && !landed_and_disarmed)`. The timer does not check `enable_flight_commands_`. An airborne Offboard sample can trigger publication after startup without an accepted operation. The initial target is the local origin. Blind and seeing tests observed this behavior.

- Require flight enablement, explicit application ownership, and an initialized target before any flight-control publication.
- Define ownership states for idle, Offboard prestream, active operation, retained hold, native mission, landing, and released control. An enum and checked transitions can implement this without adding a framework.
- Retain the setpoint stream after a confirmed application-owned Offboard hold. A completed ROS action does not automatically end hold ownership.
- On startup or restart, require explicit reconciliation with the operator and current PX4 mode. Do not infer permission to command from observing Offboard mode.
- Define what happens when ownership loses fresh telemetry. Continuing a heartbeat must represent a controller that can still satisfy its control contract.
- Mark startup-only safety parameters read-only, or implement validated changes to effective control state. The controller copies `enable_flight_commands` into a member at startup and has no parameter-update callback. A successful ROS parameter update must not appear to disable an active controller while its effective enablement remains unchanged.

**Completion evidence:** zero commands, `OffboardControlMode`, and setpoints in disabled mode under every synthetic PX4 state; zero unsolicited output after restart in airborne Offboard; no origin target; positive takeoff and retained-hold controls still pass. Verify the corrected behavior in SITL with process and agent restarts.

### R2. Bound local commands and validate numeric conversions. P0

**Confirmed defect.** [`local_navigation_goal`](../../src/vehicle_control/src/vehicle_control_node.cpp) returns `local_enu` input before applying the geographic path's 1,000 m horizontal and 120 m vertical-change bounds. Admission checks double precision finiteness. [`Px4Adapter::publish_position_setpoint`](../../src/vehicle_control/src/px4_adapter.cpp) narrows coordinates to `float` without a representability check. The seeing controller and public pipeline observed a finite `1e300` input becoming a non-finite PX4 position.

- Apply a documented physical envelope to local, relative, body-relative, and geographic targets after resolution. Separate a command-relative bound from the site containment boundary.
- Validate finiteness and range before narrowing. Check the encoded value after conversion at the PX4 output boundary.
- Apply the same checks to takeoff arithmetic, look-at points, orbit centers, trajectory interpolation, and mission-item altitude encoding.
- Preserve PX4's intentional NaN sentinels for unused velocity, acceleration, and yaw fields. Required position fields must be finite.
- Reject positive values that narrow to zero when zero selects a default or violates the action contract. The seeing planner tests observed `1e-50` tolerance and takeoff values becoming zero in successful service responses.

**Completion evidence:** the complete public command path rejects oversized local targets before output; exact boundaries, next representable values, overflow, underflow, and arithmetic overflow have tests. Failed planning returns a diagnostic and a valid recovery canary succeeds.

### R3. Check flight state throughout execution and before success. P0

**Confirmed defect.** [`execute_navigation`](../../src/vehicle_control/src/vehicle_control_node.cpp) checks fresh position/status and frame revision while running, but does not require continued armed Offboard flight or reject an active failsafe in its ordinary tracking loop. Seeing tests observed successful navigation after mode exit and after a failsafe flag change. Takeoff also needs a review of its continuing mode, arming, and reset conditions.

- Define allowed PX4 states for every stage, including deliberate Mission, Hold, Orbit, Offboard, and Land transitions.
- Recheck state before accepting target proximity as success. A vehicle reaching a coordinate after pilot takeover must not count as completion of an operation that lost authority.
- Abort the application operation on unexpected mode exit, disarming, or failsafe. Release application control without automatically retaking control from the pilot or PX4 recovery mode.
- Supply a continuing execution-health callback to the relevant [`run_child_action`](../../src/mission_execution/src/mission_execution_node.cpp) call sites. Several current paths validate state only before child dispatch.
- Give fault and takeover conditions precedence over a simultaneous child success result. Avoid applying landed-start predicates throughout an airborne mission.

**Completion evidence:** inject takeover, disarm, failsafe, and status loss during takeoff, navigation, trajectory, orbit, route, and landing stages. Every action produces its specified bounded outcome, and no operation reenters Offboard after an operator mode change.

### R4. Apply the estimator-reset contract to every local operation. P0

**Confirmed gap.** Navigation captures a local-frame revision, and native orbit checks references and reset counters. [`execute_follow_trajectory`](../../src/vehicle_control/src/vehicle_control_node.cpp) does not capture or check a revision. The controller and public pipeline reset tests failed to terminate within their test deadlines.

- Capture the frame revision when admitting local trajectories and local takeoff/hold targets.
- Invalidate affected local targets when position or reference resets. Prevent the timer from publishing an old-frame target after the reset is known.
- Establish a new hold only from a fresh valid estimate and the approved recovery policy. Report an unconfirmed recovery as failure.
- Preserve fixed geographic destinations for uploaded missions. Do not abort a valid geographic route solely because its estimator-local origin changes.

**Completion evidence:** independent XY, Z, and geographic-reference resets during each local operation produce the required result and output behavior. Geographic mission tests retain identical destinations across resets. Specify the response deadline rather than adopting the harness's 0.7 or 0.8 seconds as an undocumented protocol guarantee.

### R5. Validate the whole mission before dispatching takeoff. P0

**Confirmed defect and contract inconsistency.** The seeing invalid-orbit mission dispatched an onboard child before its orbit tail was rejected. Standalone public orbit validation also omits the documented timeout ceiling. The launch passes orbit limits to planner and vehicle control, but does not pass them to mission execution. Planner-response validation permits speeds up to 10 m/s while the default vehicle limit is 3 m/s.

- Validate all mission stages, orbit radius/speed/acceleration, centers, timeout, envelope, and requested heading behavior before the first flight child.
- Use consistent configured limits across planner, executor, and controller. Maintain downstream defensive validation.
- Specify a standalone takeoff ceiling. The blind 121 m takeoff probe extrapolates a mission limit; it does not establish an existing standalone contract violation.
- Revalidate state and frame availability at handoff. Preflight validation cannot guarantee that telemetry remains valid later.

**Completion evidence:** invalid standalone and mission-tail requests produce zero flight children; non-default limits agree across all three nodes; a permissive synthetic backend cannot turn invalid public input into reported success.

### R6. Make cancellation and failure postconditions explicit. P0

**Confirmed inconsistency.** Navigation cancellation waits for settled Offboard hold. Trajectory cancellation calls `hold_current_position` and reports canceled immediately. Native-route cancellation confirms PX4 Hold mode. These have different physical postconditions. A failed orbit tail ends its parent without automatically dispatching landing.

- Define the required postcondition for every canceled or aborted operation: settled application hold, confirmed native Hold, committed landing, externally controlled recovery, or unknown state.
- For retained local operations, confirm braking and settling before reporting the documented hold contract. Include horizontal and vertical speed and a settling interval.
- Document native Hold mode confirmation separately from measured stationary flight. Fleet separation must account for the stopping transient.
- Preserve child draining, landing priority, late-acceptance cancellation, and the prohibition on new child dispatch while a prior child outcome is unknown.
- Provide an independent pilot recovery path when ROS landing is blocked by an uncooperative or unreachable child. Do not bypass child draining by creating competing ordinary command publishers.
- Define supervision after a failed mission or orbit so the aircraft cannot remain in an unattended hold until battery exhaustion.

**Completion evidence:** cancel every action while moving and during handoffs; land during upload, arming, takeoff, waypoint flight, orbit, and cancellation; inject child death and missing results. Verify postconditions using telemetry and PX4 logs rather than terminal status alone.

### R7. Expose explicit health and freshness to the executor. P0

**Source-review finding.** [`VehicleState`](../../src/swarm_interfaces/msg/VehicleState.msg) has no independent status-valid, land-state-valid, failsafe, or battery-age field. [`publish_state`](../../src/vehicle_control/src/vehicle_control_node.cpp) publishes fresh application messages while stale PX4 status causes armed and mode flags to become false. Battery availability has no receive-age check in the adapter. A fresh application header cannot establish the freshness of every underlying stream.

- Represent unknown/stale status and land detection explicitly. Keep observation validity separate from the observed boolean value.
- Expose the health needed for admission and monitoring through application types, including failsafe and battery freshness. Define which layer owns each recovery decision.
- For mission readiness, require a usable global estimate and heading only where the command needs it. Add measured position-quality criteria for field use.
- Detect stalled or replayed source samples as well as receive silence. Specify acceptable handling of source-clock resets and reconnection.
- Preserve stream-specific timing. The recorded land-detector correction allows at least 2 seconds for unchanged 1 Hz land messages; position/status retain a 1-second default.

**Completion evidence:** drop each telemetry stream independently, keep other streams alive, inject invalid flags and replayed samples, and verify that neither stale status nor stale battery becomes fresh readiness. Establish deadlines from measured publication and transport rates.

### R8. Approve and exercise a PX4 recovery profile. P0

**Missing validation.** One vehicle-control SIGKILL run establishes only one process-loss case. Native missions and Offboard operations have different companion-loss behavior.

- Record actual PX4 values for Offboard loss, RC loss, data-link loss, position loss, low battery, geofence action, Return altitude/path, and automatic disarming. Preserve preflight checks.
- Define whether companion loss during an uploaded route permits completion or requires a different configured recovery. Verify that behavior with the physical communication topology.
- For Offboard, verify `COM_OF_LOSS_T` and `COM_OBL_RC_ACT` through measured loss tests. Test with and without RC availability under controlled simulation conditions.
- Check interactions among failsafes. A continued ground-station heartbeat can mask the loss of a different link.
- Record recoveries that require position, such as Hold or Return, and the fallback when position is unavailable.

PX4 documents distinct recovery mechanisms in [Safety configuration](https://docs.px4.io/v1.17/en/config/safety) and supports fault exercises in [Simulate failsafes](https://docs.px4.io/v1.17/en/simulation/failsafes). The chosen parameter values and test deadlines are project decisions for the actual vehicle.

**Completion evidence:** the fault matrix below passes in SITL and the applicable props-off/hardware integration environment. The pilot's independent takeover procedure is demonstrated before autonomous flight.

## Work required to complete the single-drone baseline

| ID and priority | Required work | Completion evidence |
| --- | --- | --- |
| R9, P1 | Validate child progress and terminal results in `execute_mission`. Seeing tests reported public success for zero completed waypoints and for `UINT32_MAX`. Validate bounds, required final count, orbit completion, and landing postconditions. | Inconsistent results and impossible progress cannot produce public success. Real PX4 progress still completes correctly, including takeoff-only orbit prefixes. |
| R10, P1 | Diagnose action/service flood reply loss. Separate callback scheduling, middleware queues, client delivery, and application admission. Define supported request rates and overload behavior. | At supported load, at most one ordinary operation owns control, every supported request has a bounded outcome, landing retains its response budget, and a post-flood canary works. Record CPU, memory, threads, and response latency. |
| R11, P1 | Resolve FT-04. Compare command timing, actual position, speed, acceleration, wind, simulation real-time factor, and PX4 smoothing. The current interpolator produces position-only piecewise-linear targets and succeeds on final settling. | Every required lane endpoint is visited in order and route tracking meets predeclared limits. Keep the historical failure and explain any change to schedule or acceptance criteria. |
| R12, P1 | Verify native mission protocol failure paths: wrong system, no discovery, upload rejection, partial/canceled upload, lost acknowledgments, lost progress, reconnect, old same-length mission progress, and external mission replacement. | No unintended arming or late start; results refer to the intended route; mission stop is confirmed or explicitly unknown. Uploaded route contents and progress are correlated in evidence. |
| R13, P1 | Reconcile orbit completion evidence. `OrbitProgress` accumulates position angles without timestamps or speed plausibility. The node rejects angular jumps above 90 degrees, but smaller repeated jumps need investigation. | Rerun the teleport probe with a budget that includes hold recovery. Demonstrate whether completion is possible, then add elapsed-time/displacement plausibility and an independent valid-orbit control as required. |
| R14, P1 | Audit exception and shutdown paths. The vehicle worker catch logs and calls `finish_goal`; that catch alone does not terminate a ROS action. Cover queued work, unexpected library exceptions, node shutdown, and callback lifetime. | Each accepted action reaches a terminal result when the server remains alive, or the client detects server loss within its deadline. No orphan operation, callback use after destruction, or shutdown hang. |
| R15, P1 | Freeze and automate the test baseline. Review both torture suites, repair harness assumptions, preserve failure traces, and integrate build/package tests plus headless fault tests into CI. | Fresh checkout and clean image reproduce results. No unexplained failure, harness error, or skip is accepted as a pass. PX4/Gazebo tests run in a separate controlled job. |
| R16, P1 | Build and run the same commit on the Pi. Measure scheduler delay, setpoint gaps, serial/network use, memory, temperature, storage, and power behavior under intended payload load. | Native arm64 build and applicable tests pass. The complete mission and recovery sequence runs from the actual companion hardware without resource exhaustion or watchdog-budget violation. |
| R17, P1 | Add deployment and flight evidence. Save software/image hashes, dependency versions, actual parameters, plans, identities, fault timings, ROS actions, ULogs, and operator observations. | Every acceptance claim links to one immutable run and the exact deployed configuration. Evidence survives a fresh checkout and is accessible to reviewers. |
| R18, P1 | Establish energy and route-duration limits. Current planning limits bound geometry, not flight endurance; waypoint speed also depends on PX4 configuration. | A measured payload/wind endurance budget includes departure, route, recovery, landing, and reserve. Reject or shorten missions outside it. Verify low-battery recovery in simulation. |
| R19, P1 | Define stored-mission lifecycle and restart reconciliation. Cancellation leaves a mission stored in PX4; an external client can reenter Mission mode. | Operators can inspect, retain, replace, or clear the stored route deliberately. Restart never automatically rearms or resumes an ambiguous action. Test same-route resubmission and unintended duplicate clients. |
| R20, P1 | Validate the real geographic and altitude contract. Survey launch and route positions, distinguish AMSL from ellipsoid and ground-relative heights, and measure GNSS uncertainty and landing dispersion. | Independent measurements support the flight envelope. Shared-origin examples are replaced with actual site values; an estimator reporting its own target proximity is not the sole accuracy measurement. |

R9, R12, R14, and R19 apply to a native-waypoint G1 profile. R11 and R13 apply before enabling their associated trajectory or orbit capabilities. R10, R15 through R18, and R20 require evidence appropriate to every G1 deployment.

## PX4 and implementation recommendations

### Keep native waypoint missions as the default static-route backend

PX4 [Mission mode](https://docs.px4.io/v1.17/en/flight_modes_mc/mission) executes stored routes and performs mission feasibility checks. Geographic mission items match this backend. Keep survey lanes and predetermined waypoint routes on that path. The process-loss run supports retaining this boundary for static missions.

PX4's first-waypoint-distance parameter is a warning in the documented v1.17 behavior. Application containment and PX4 fence checks still need explicit acceptance tests. Download or inspect the uploaded route during integration and confirm that site altitude, landing location, acceptance radii, and configured mission speeds match the plan.

### Treat Offboard heartbeat as a control-health obligation

PX4 [Offboard mode](https://docs.px4.io/v1.17/en/flight_modes/offboard) requires an incoming proof-of-life stream above 2 Hz and at least one second of prestreaming. ROS 2 supplies that proof through `OffboardControlMode`; setpoints carry the target. The project's 20 Hz default and one-second prestream follow this model.

Measure maximum publication gaps under load. Set the operating margin using the actual loss timeout and transport behavior. A running timer must not keep ownership alive after the application's required telemetry or state becomes unusable. Keep recovery mode selection in PX4 and verify any application-requested handoff.

The current parameter validator accepts exactly 2 Hz. Reject configurations without margin above the documented Offboard proof-of-life boundary and verify the delivered rate under load.

### Preserve adapter isolation and verify transport configuration

PX4's [uXRCE-DDS guide](https://docs.px4.io/v1.17/en/middleware/uxrce_dds) documents non-default QoS, automatic timestamp handling, and incompatibility between its v2 client and a v3 agent. Retain the pinned v2.4.3 agent and matching PX4/`px4_msgs` releases until an explicit compatibility update.

Verify actual endpoint QoS, versioned topic names, source timestamps, and reconnect behavior on the hardware image. Evaluate command durability explicitly: replay of an obsolete flight command after a subscriber reconnect must not cause an unintended transition. Keep monotonic clocks for local deadlines and verify timestamp behavior across simulation pauses, wall-clock corrections, and PX4 restarts.

### Consolidate policy without rewriting flight control

- Define one documented table of configured and hard limits, defaults, sentinels, coordinate meaning, and authority lifetime. Pass applicable configuration to every enforcing layer and test their agreement.
- Share application validation code where it removes duplication. Cross-language planner/controller agreement needs contract tests; it does not require a new code-generation system.
- Replace repeated state booleans with explicit ownership transitions where combinations are currently ambiguous. Keep action-specific flight behavior separate from transport decoding.
- Keep ROS callbacks bounded and long-running work on workers. Measure overload before changing executors or introducing more threads.
- For retained timed trajectories, assess whether smoother position targets and correctly modeled feedforward are required. Adding velocity to `TrajectorySetpoint` changes PX4 control behavior and requires a separate validated change.
- Apply repository typing and documentation rules when touching code. `trajectory.cpp` contains deducible `auto` references despite the explicit-type rule, and several helper functions lack the required production-function comments. These are maintenance tasks after flight correctness fixes.
- Move test-only package discovery into `if(BUILD_TESTING)` in the [vehicle-control build](../../src/vehicle_control/CMakeLists.txt) and [executor build](../../src/mission_execution/CMakeLists.txt). Both currently require `ament_cmake_gtest` even when testing is disabled, while their manifests declare it as a test dependency and the Pi setup excludes test dependencies. Verify a minimal native build and explicitly install test dependencies for qualification runs.

## Required fault-test matrix

Run synthetic faults in containers that cannot reach a vehicle. Use SITL for dynamics and PX4 failsafes. Use props-off bench work for physical identity, links, restart, and power integration. A destructive flight fault must first pass the corresponding simulation and bench procedure.

| Fault or transition | Required observable result | Evidence environment |
| --- | --- | --- |
| Disabled commands; airborne Offboard at startup or restart | No output without authorized ownership; no origin target; operator reconciliation required. | Synthetic peers, SITL, props-off bench. |
| NaN, infinity, huge finite values, float underflow, invalid frames | Reject before unsafe dispatch or output; bounded diagnostic; subsequent valid request works. | Unit, synthetic, public pipeline. |
| Position, status, land, heading, or battery loss independently | Validity expires per stream; no false readiness or success; specified recovery. | Synthetic, SITL, bench links. |
| Local XY/Z reset or reference change | Local operations invalidate old targets; geographic routes preserve destinations. | Synthetic and SITL estimator scenarios. |
| Pilot mode switch, failsafe, unexpected disarm | Application gives up affected authority and cannot report false completion or regain control automatically. | Synthetic, SITL, supervised hardware progression. |
| Cancel while moving; landing during any stage | Required hold or landing postcondition; previous child drained; ambiguous child blocks competing dispatch. | Synthetic ordering plus independent SITL dynamics. |
| Child rejection, death, late acceptance, missing result, contradictory result | Unsuccessful bounded parent result; no late mission execution; no false success. | Synthetic and process-loss SITL. |
| MAVLink loss with DDS healthy; DDS loss with MAVLink healthy | Measured mode-specific behavior matches approved monitoring and recovery policy. | SITL and props-off link tests. |
| Pi power loss; agent loss; ROS executor stall | PX4 performs the approved native-mission or Offboard-loss response; recovery cannot depend on a dead ROS node. | SITL topology tests, Pi-to-SITL, props-off bench. |
| RC loss, ground-station loss, GNSS degradation, low battery, fence breach | PX4 selects the approved response and application does not defeat it. | PX4 failsafe simulation; authorized hardware checks. |
| Upload interrupted; stale progress; externally replaced stored mission | No wrong-route success or unintended start; restart reconciles stored mission. | Protocol peer tests and real PX4 SITL. |
| Repeated goals, duplicate nodes, planner/action flood | Single ownership; bounded supported-load response; recovery remains responsive; post-load canary. | Synthetic, Pi-to-SITL load tests. |

Record injection time, detection time, last unsafe/old target publication, terminal action time, selected PX4 mode, stopping distance, and eventual landed/disarmed status where landing is expected. A synthetic success response cannot establish physical hold.

## Qualification sequence and acceptance evidence

The repetition counts below are proposed project minima. They are not PX4 requirements, regulatory certification, or a statistical reliability claim. Review the actual vehicle envelope and budgets before adopting them.

### G0: supervised props-off integration

- [ ] Assign hardware and software maintainers, pilot, and reviewer. Remove all propellers and record the bench power/arming procedure.
- [ ] Build the reviewed commit on arm64 with pinned dependencies. Run applicable tests with testing enabled; the current Pi guide's `-DBUILD_TESTING=OFF` command alone does not provide test evidence.
- [ ] Verify supply voltage/current margin, wiring retention, serial permissions, cooling, storage, and logs. Pi power loss must not remove flight-controller power.
- [ ] Document application namespace, `vehicle_id`, DDS prefix/key, PX4 system ID, MAVLink endpoint, and physical airframe identifier in one mapping.
- [ ] Demonstrate that DDS telemetry and the MAVLink mission endpoint belong to that same physical controller. A command target ID does not authenticate the source of telemetry on a topic.
- [ ] Confirm disabled startup, duplicate-process detection, correct topic versions, and a separate recovery/control path.
- [ ] Use separate serial links or a tested router for DDS and MAVLink; two clients cannot independently own one serial device. Follow the [Pi guide](../setup/HITL-PI-SETUP.md).

The proposal uses “HITL” for running companion code against a simulated drone. Label that setup **Pi-to-SITL integration** when PX4 remains on the desktop. PX4 [HITL](https://docs.px4.io/v1.17/en/simulation/hitl) runs PX4 on the physical flight controller. Its v1.17 guide marks HITL as community maintained and lists jMAVSim/Gazebo Classic configurations. Verify compatibility before claiming that Gazebo Harmonic provides controller HITL. Record which hardware, sensors, actuators, and transport paths each setup exercises.

### G1: approve one restricted autonomous hardware flight

- [ ] Close R1 through R8 for every reachable operation. Resolve applicable R9 through R20 work, or disable excluded capabilities with tested rejection.
- [ ] Freeze one commit, clean image/native build, complete test corpus, launch profile, and PX4 parameter export. All required build, package, synthetic, and public-pipeline checks pass.
- [ ] Reproduce each relevant safety regression 20 consecutive times without unexplained failure. Use blind seeds `20261119` and `42` with fuzz count 512, preserving flood diagnostics and positive controls.
- [ ] Run at least 10 consecutive isolated SITL repetitions of the intended first-flight sequence, including cancellation and recovery. Apply the intended speed, containment, and loss policies. Preserve every failed attempt.
- [ ] Run a Pi-to-SITL soak for at least twice the planned sortie duration under intended logging and payload load. Observe no resource exhaustion or configured health-deadline violation.
- [ ] Verify the applicable fault matrix and the independent recovery path. A PX4 controller HITL run, when available, supplements the recorded bench evidence.
- [ ] Survey the physical site and approve its clearance, containment, recovery areas, wind limit, GNSS/heading quality, and actual AMSL values. Record required site and operational permissions with the flight record.
- [ ] Verify the assembled vehicle through supervised pilot-controlled checkout before application flight. Review motor direction, sensors, position hold, landing, battery, and logs using the actual payload.
- [ ] Have the pilot and software reviewer approve a written flight card with numeric limits and abort conditions. Do not transfer the simulation's 30 m/100 m planner defaults directly into a first-flight approval.

PX4's [First Flight Guidelines](https://docs.px4.io/v1.17/en/flying/first_flight_guidelines) recommend open space, advance route and landing planning, and an experienced operator. The flight card must identify the pilot's mode switch and recovery procedure, rather than relying on the companion computer's landing action.

The first autonomous sequence should use the approved low-altitude takeoff, a brief hold, and landing. Review its logs before adding displacement. Add a short out-and-back, an uploaded square, and native search lanes in separate accepted steps. Exercise a retained orbit or timed trajectory only after its own qualification.

### G2: declare the single-drone baseline complete

- [ ] Close every P0 and P1 item for the retained capability set. No unresolved result is labeled passed because a recovery landing succeeded.
- [ ] Pass FT-00 through FT-07 on the final SITL revision for all retained features. Resolve FT-04 or formally remove and disable its unsupported capability. Preserve the original tracking metric and any approved replacement requirement.
- [ ] Add native-mission lane coverage checks and an independent orbit-progress check. Require path quality in addition to action success and landing.
- [ ] Complete at least 10 consecutive representative physical sorties on the approved platform, spanning at least two batteries and two sessions, within the approved weather envelope. Record all attempts and any pilot intervention.
- [ ] Measure endpoint error, cross-track error, maximum speed, stop distance/time, hold drift, heading error when controlled, landing dispersion, battery reserve, telemetry gaps, and action latency. Use predeclared pass limits and an independent position reference for claimed physical accuracy.
- [ ] Verify reboot and resubmission behavior, stored mission cleanup/reconciliation, manual takeover, return/landing geometry, and post-failure supervision.
- [ ] Publish a versioned capability statement: supported operations, disabled operations, site/weather/payload limits, known uncertainties, exact configuration, and linked qualification evidence.
- [ ] Complete the repository's release checks and two-person release review in [Contributing](../../CONTRIBUTING.md). Keep `main` at the latest fully validated release.

Use the current 0.75 m SITL endpoint and drift checks as existing software-test requirements. Physical tolerances require measured navigation uncertainty and clearance budgets; the simulator's estimator-relative errors do not establish field GNSS accuracy. Approve a hardware requirement before its test rather than widening it after a failed flight.

## Conditions for two drones, then three

G2 permits shifting implementation effort to coordination. Two-vehicle SITL can be used earlier to investigate routing and fault isolation, but it does not close a single-vehicle safety defect.

Before simultaneous physical flight:

- [ ] Prove identity isolation with two PX4 instances, two independent local origins, distinct DDS keys, system IDs, prefixes, and MAVLink endpoints. Send a command to each vehicle and observe zero cross-command effects. Inject deliberately incorrect mappings.
- [ ] Add per-vehicle health, mission identity, assignment acknowledgment, task progress, and command-source priority through mission execution. Preserve operator and emergency recovery authority.
- [ ] Define peer-loss behavior and task reassignment. Communication loss does not prove that a missing aircraft has stopped flying or vacated its assigned region.
- [ ] Reserve separate takeoff, route, return, hold, and landing volumes. Test simultaneous Return and low-battery landing, shared approach conflicts, and one failed companion while its stored mission continues.
- [ ] Set separation from airframe size, measured localization errors, tracking errors, detection/communication latency, stopping distance, wind, and rotor/downwash interaction. Different commanded altitudes alone do not establish separation.
- [ ] Validate route conflict checks and separation with independent simulated truth and physical measurements. Do not use identical GNSS estimates as the only collision-clearance evidence.
- [ ] Update the flight harness for namespaces and per-vehicle logs. The current runner expects root node names and subscribes to `/vehicle_state`; namespaced mock planner tests do not qualify real PX4 routing.
- [ ] Pass two-drone SITL fault cases, Pi/network integration, and staged physical flights before adding a third vehicle. Repeat routing, load, separation, and simultaneous-recovery qualification for three.

PX4's [ROS 2 multi-vehicle guide](https://docs.px4.io/v1.17/en/ros2/multi_vehicle) distinguishes instance number, DDS namespace/key, and command target system. The current launch already separates application namespace, PX4 prefix, and MAVLink connection; preserve that separation and validate the complete mapping.

Automatic allocation, perception-driven replanning, dynamic obstacle avoidance, and search adaptation require later requirements and evidence. They do not need implementation before the restricted single-drone waypoint baseline.

## Research guidance and its application

The following papers inform engineering choices. Their results do not qualify this airframe or prescribe this document's test counts.

| Source | Supported result or scope | Project application |
| --- | --- | --- |
| Choset and Pignon, 1997, [Coverage Path Planning: The Boustrophedon Decomposition](https://publications.ri.cmu.edu/coverage-path-planning-the-boustrophedon-decomposition) | Coverage is defined over an environment; decomposition and traversal provide completeness under the paper's geometric assumptions. | Add coverage metrics for native search lanes. The current convex-polygon/circle stripe generator does not implement general obstacle-aware cellular decomposition. Validate boundary strips, turn connections, and excluded areas. |
| Waharte and Trigoni, 2010, [Supporting Search and Rescue Operations with UAVs](https://www.cs.ox.ac.uk/files/3198/submission_waharte.pdf) | The study treats sensing quality, energy, environmental hazards, and inter-UAV information exchange as search-design factors, and evaluates search time. | A later SAR requirement should measure time to detection, probability of detection, coverage, reserve energy, and communication behavior. Until sensing is implemented, label current results as route execution. |
| Kazemdehbashi and Liu, 2024, [An exact coverage path planning algorithm for UAV-based search and rescue operations](https://arxiv.org/abs/2405.11399) | The preprint optimizes coverage time for multiple UAVs over rectangular areas under wind using a grid formulation. | Evaluate wind and heading when budgeting route duration and battery. Treat its geometry and optimization assumptions separately from field flight feasibility and obstacle clearance. |
| Preiss, Hönig, Sukhatme, and Ayanian, 2017, [Crazyswarm: A Large Nano-Quadcopter Swarm](https://whoenig.github.io/publications/2017_ICRA_Preiss_Hoenig.pdf) | The system measures latency and tracking for indoor nano-quadrotors with motion capture and substantial onboard computation. | Retain onboard flight responsibility and measure communications and tracking before scaling. Its localization and vehicle regime do not support importing its formation spacing into GPS-based X500 flights. |

For future sensor-based coverage, define sensor footprint, usable overlap, altitude above terrain, speed/exposure constraints, and a localization margin before selecting stripe spacing. Maintain separate ledgers for planned area, traversed area, observed area, and detected targets. Task reassignment requires the unfinished observed area, not only the count of completed waypoints. These are project recommendations derived from the papers' coverage and sensing constraints.

## Work order and review record

| Order | Deliverable | Exit condition |
| --- | --- | --- |
| 1 | Immutable evidence baseline and regression classification | Torture harness errors and provenance discrepancies are identified; source, binaries, tests, and configuration are traceable. |
| 2 | Publication ownership and numeric boundaries, R1–R2 | Disabled/restart and public non-finite-output regressions pass. |
| 3 | Continuing state, frame, preflight, cancellation, and health contracts, R3–R7 | Per-action fault matrix passes with explicit postconditions. |
| 4 | PX4 profile and native mission correctness, R8–R10 and R12/R14/R19 | Intended route and recovery work under process, protocol, and link faults. |
| 5 | Retained trajectory/orbit quality, R11/R13; automation and runtime evidence, R15–R18/R20 | G1 software and integration checklist is complete for the approved scope. |
| 6 | Staged physical qualification | G1 flight card is executed; G2 evidence accumulates against predeclared limits. |
| 7 | Two-vehicle requirements and SITL implementation | G2 baseline is frozen; identity, conflict, and peer-loss contracts are testable. |

For each R item, record: responsible maintainer; reviewer; issue/change link; affected capability; failure reproduction; required postcondition; acceptance deadline/tolerance; passing run IDs; deployed revision; and closure date. For each gate, record the pilot/software reviewers, exact approved scope, configuration hashes, evidence links, and outstanding exclusions.

## Review limits and assumptions

- This review inspected application control paths, coordinate and planning logic, interfaces, launch/build configuration, test harnesses, saved aggregate results, selected failure traces, and recorded flight reports. It did not inspect every historical ULog sample or establish structural, electrical, or regulatory flight approval.
- No production code was changed. No new package build, torture run, SITL sortie, Pi run, HITL session, or physical flight was executed for this document. The session's unprivileged Docker inventory check returned a socket permission error; recorded test results remain historical evidence.
- The assumed first platform is the project's GPS-based multicopter with a Raspberry Pi companion, matching PX4/`px4_msgs` v1.17.0 and MAVSDK 4.2.0. Requalify compatibility and behavior when hardware, firmware, dependencies, or payload changes.
- “Single-drone complete” means complete for a declared operating envelope and capability set. No finite suite establishes correct behavior for every possible condition.
- The source-backed defects, harness assumptions, and proposed validation requirements have separate labels. The planner snapshot discrepancy, flood root cause, orbit teleport outcome, physical navigation accuracy, and complete companion-loss response remain open questions requiring the specified evidence.
