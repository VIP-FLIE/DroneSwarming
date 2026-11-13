# Flight systems review

Review date: November 13, 2026. Source commit: `66c6bb80f905d2ad9a730566ef3749f6678f9fb4`. Branch: `issues`. Source locations refer to this commit. This report changes no application behavior.

## Scope and evidence

The review covers the PX4 adapter, flight actions, mission arbitration, JSON planning and geography, ROS interfaces, flight-test runner, package tests, launch/build configuration, simulation assets, shell scripts, and repository documentation. The compatibility baseline is ROS 2 Jazzy with PX4 and `px4_msgs` v1.17.0. The report contains 40 findings: 11 P1, 25 P2, and four P3.

- **Defect:** the implementation contains the described failure path. Entries identify runtime reproduction when available.
- **Safety gap:** a recovery or ownership requirement is absent or insufficient. The entry states the triggering conditions.
- **Recorded failure:** an existing validation document reports the failure. This review does not establish its physical root cause.
- **Simplification:** an architectural or maintenance choice with explicit migration costs.
- **P1:** address before flight use that can encounter the triggering condition.
- **P2:** address in the next reliability or maintainability pass.
- **P3:** consolidation or removal with lower immediate operational impact.

No owning raw allocation or confirmed lock-order deadlock was identified in the reviewed application code. Shared ownership retention, unbounded message allocation, unsafe duration conversion, cancellation liveness, and worker shutdown are the resource and concurrency findings. No hardware flight or Gazebo flight was performed for this review.

Validation results are recorded at the end of this report. Runtime probes use synthetic PX4 messages, isolated topic prefixes, and a separate ROS domain. They establish node behavior without establishing airframe response.

## Flight authority and recovery

### ISS-01 | P1 | Defect: disabled and uninitialized nodes publish flight setpoints

**Locations:** [vehicle_control_node.cpp](src/vehicle_control/src/vehicle_control_node.cpp#L769), lines 769-818; [vehicle_control_node.hpp](src/vehicle_control/include/vehicle_control/vehicle_control_node.hpp#L136), lines 136-145; [reserve_goal](src/vehicle_control/src/vehicle_control_node.cpp#L423), lines 423-426.

**Issue:** `enable_flight_commands_` gates action admission but does not gate the setpoint timer. `should_publish` becomes true whenever telemetry reports Offboard and the vehicle is not both landed and disarmed. A newly started node has a zero-initialized target and no ownership of that vehicle's current flight. Launching the default, supposedly observation-only application beside an existing Offboard controller therefore introduces setpoints at the PX4 local origin. Duplicate application instances can also publish competing targets.

**Evidence:** with commands disabled and synthetic airborne Offboard telemetry at NED `(5,7,-3)`, the actual node published 29 unsolicited setpoints during the observation window. The first target was NED `(0,0,0)`.

**Pros of changing:** makes the default launch passive; prevents unrequested motion and interference during inspections or restarts.

**Cons of changing:** a restarted process cannot resume an existing Offboard flight implicitly. Recovery needs an explicit ownership/adoption procedure or PX4-managed mode.

**Proposed solution:** require enabled commands and an initialized, locally owned control session before publishing either heartbeat or setpoints. Remove the unconditional telemetry-based publication path. Preserve hold through the explicitly owned session. Add a regression using the existing node with synthetic telemetry.

### ISS-02 | P1 | Safety gap: heartbeat survives stale telemetry and loss of mission authority

**Locations:** [publish_control_setpoint](src/vehicle_control/src/vehicle_control_node.cpp#L786), lines 786-818; [hold_current_position](src/vehicle_control/src/vehicle_control_node.cpp#L716), lines 716-727; [mission failure exits](src/mission_execution/src/mission_execution_node.cpp#L886), lines 886-890 and 922-926; [VehicleState](src/swarm_interfaces/msg/VehicleState.msg), lines 1-25.

**Issue:** the timer continues publishing from cached Offboard state when incoming telemetry expires. `hold_current_position` can also adopt an old position because it checks validity without freshness. Independently, a failed or terminated mission-execution process leaves vehicle control streaming its last target. PX4 still receives proof of an active external controller even though the mission owner is gone. A failed route with `land_after_waypoints=true` skips its final landing and can remain in this state until another actor intervenes or a separate PX4 failsafe activates. PX4's Offboard-loss mechanism depends on loss of the external heartbeat. [PX4 Offboard behavior](https://docs.px4.io/v1.17/en/flight_modes/offboard).

**Evidence:** after synthetic telemetry had expired for longer than the configured one-second timeout, the node emitted another 10 setpoints in 0.5 seconds.

**Pros of changing:** bounds unattended hold and gives PX4 an accurate indication of companion control health.

**Cons of changing:** brief link interruptions can terminate application control; the chosen PX4 recovery action must fit the operating site.

**Proposed solution:** define one bounded recovery policy for stale telemetry, mission-owner loss, and mission failure. Prefer handoff to PX4 Hold/Land/Return while command communication remains available, with configured Offboard-loss recovery as the fallback. Do not replace this with a second application failsafe state machine. Keep an intentional operator-requested hold distinct from an orphaned mission.

### ISS-03 | P1 | Defect: takeoff and navigation can succeed after flight-mode loss

**Locations:** [takeoff loop](src/vehicle_control/src/vehicle_control_node.cpp#L935), lines 935-954; [navigation loop](src/vehicle_control/src/vehicle_control_node.cpp#L1049), lines 1049-1103; compare [trajectory checks](src/vehicle_control/src/vehicle_control_node.cpp#L1235), lines 1235-1242.

**Issue:** takeoff and navigation check telemetry freshness and target distance during execution, but do not require continued armed Offboard flight. An RC takeover, PX4 failsafe, or disarm can leave an action running. Reaching the target under another controller can then produce success. A takeoff altitude inside the default 0.5-meter acceptance radius can also succeed without proving that the vehicle left the ground. `FollowTrajectory` already applies stronger mode checks, so the operation contracts differ.

**Evidence:** the actual navigation action returned `success=true` after synthetic `VehicleStatus` changed from Offboard to Manual and position reached its target.

**Pros of changing:** prevents false mission progress and respects operator/PX4 control changes.

**Cons of changing:** short telemetry transition intervals require phase-specific checks during takeoff rather than an airborne check before arming.

**Proposed solution:** reuse a phase-appropriate armed/mode/landed invariant in each execution loop. Abort on ownership loss without requesting Offboard again. Require airborne confirmation and a meaningful climb before takeoff success. Keep the check inside vehicle control, where PX4 state is available.

### ISS-04 | P1 | Defect: estimator resets leave ordinary setpoints in the old coordinate frame

**Locations:** [adapter revision detection](src/vehicle_control/src/px4_adapter.cpp#L59), lines 59-95; [mission frame check](src/mission_execution/src/mission_execution_node.cpp#L651), lines 651-668 and 817-822; [navigation](src/vehicle_control/src/vehicle_control_node.cpp#L998), lines 998-1158; [trajectory](src/vehicle_control/src/vehicle_control_node.cpp#L1235), lines 1235-1255; [idle hold stream](src/vehicle_control/src/vehicle_control_node.cpp#L792), lines 792-818.

**Issue:** the adapter detects XY/Z estimator resets and changes `local_frame_revision`. Only translated JSON missions and Orbit actively check the captured frame. Direct navigation, timed trajectories, ordinary waypoint missions, and the idle hold stream retain numerical targets across a reset. A position reset changes the physical location represented by those numbers. JSON cancellation through another process also leaves a window before the lower-level target is replaced.

**Pros of changing:** prevents a frame reset from commanding an unintended displacement; gives every entry point the same coordinate contract.

**Cons of changing:** aborting an otherwise recoverable reset interrupts a mission. Correctly rebasing every target requires more state and PX4 reset-delta handling.

**Proposed solution:** capture the revision at the vehicle-control boundary and invalidate active targets synchronously when it changes. Use a fresh hold or PX4-owned recovery mode, then require mission resubmission. Prefer aborting to implementing a custom target-rebasing system unless uninterrupted local flight is a demonstrated requirement.

### ISS-05 | P1 | Safety gap: explicit landing can be blocked behind an unconfirmed child

**Locations:** [child dispatch and cancellation](src/mission_execution/src/mission_execution_node.cpp#L81), lines 81-98, 146-158, and 175-218; [single worker](src/mission_execution/src/mission_execution_node.cpp#L591), lines 591-624; [Orbit recovery](src/vehicle_control/src/vehicle_control_node.cpp#L1508), lines 1508-1562; [existing blocked-landing test](src/mission_execution/test/test_command_authority.cpp#L561), lines 561-580.

**Issue:** landing interrupts ordinary intent but still waits on the same worker and `child_pending` gate. A missing child response/result can prevent a Land child from ever being sent. The tests explicitly preserve this behavior. Orbit cancellation can require a one-second prestream, up to three ACK/mode-transition attempts, and a 15-second settle. That exceeds the parent's 15-second cancellation grace and can also exceed the queued landing's ten-second dispatch wait. A landing request can fail while the vehicle continues Orbit or hold. The goal-response wait also ignores parent interruption until response or timeout.

**Pros of changing:** emergency landing latency no longer depends on unrelated child recovery or a lost ROS result.

**Cons of changing:** bypassing the gate without changing the lower-level controller would allow a previous child to overwrite landing.

**Proposed solution:** put terminal landing preemption at the vehicle-control/PX4 authority boundary. Atomically revoke the previous operation's permission to publish or restore modes, then request PX4 Land without requiring a prior hover settle. Preserve child draining for ordinary command replacement. Keep PX4's own mode prerequisites and failsafes intact.

### ISS-06 | P1 | Defect: recovery retries can reclaim control after operator takeover

**Locations:** [restore_offboard_after_orbit](src/vehicle_control/src/vehicle_control_node.cpp#L1483), lines 1483-1526; [request_transition](src/vehicle_control/src/vehicle_control_node.cpp#L620), lines 620-692.

**Issue:** Orbit recovery checks that the current mode is Orbit or Offboard once, then prestreams for one second and requests Offboard. During that interval, or between retries, an operator can select Land/Return/Manual. Prestream checks freshness only. `request_transition` retries against a completion predicate without revalidating the original ownership condition. A later Offboard request can therefore reclaim control after a takeover. The documentation's promise to leave another PX4 mode in control does not cover this race.

**Pros of changing:** prevents an application recovery path from undoing an operator or PX4 recovery decision.

**Cons of changing:** recovery will abort more often when observed state changes; an explicit subsequent request is needed to resume application control.

**Proposed solution:** revalidate the permitted source mode and active operation generation immediately before every mode-changing command and throughout prestream. Stop recovery on any external mode change. Consider PX4 ROS 2 mode ownership in ISS-28 instead of expanding the local retry protocol.

### ISS-07 | P1 | Recorded failure and safety gap: navigation cancellation does not confirm a stopped hold

**Locations:** [navigation cancellation](src/vehicle_control/src/vehicle_control_node.cpp#L1025), lines 1025-1047; [hold target](src/vehicle_control/src/vehicle_control_node.cpp#L716), lines 716-727; [FT-06](src/flight_test/flight_test/runner.py#L1227), lines 1227-1253; [recorded result](docs/testing/COMMAND-ARCHITECTURE-VALIDATION.md#L47), lines 47 and 61-63.

**Issue:** cancellation replaces the target with the measured current position and immediately returns `CANCELED`. It does not confirm low velocity or a stable position. A vehicle moving toward a step position target must still brake. The recorded FT-06 drift was 3.962 meters against a 0.75-meter limit while the heartbeat continued. The record establishes a failed physical hold requirement; it does not prove that the position-target implementation is the only cause. Orbit recovery already waits for velocity and position settling.

**Pros of changing:** action termination can represent a verified hold; higher-level operations gain a defined physical handoff.

**Cons of changing:** cancellation takes longer. An immediate stop cannot promise zero braking distance.

**Proposed solution:** use PX4's braking/Hold behavior or a compatible PX4 go-to controller, then verify fresh velocity and position within the documented hold envelope. Keep a separate terminal-landing handoff that skips hover settling. Re-run FT-06 with retained flight evidence; retain the existing threshold until the physical requirement is reviewed.

### ISS-08 | P1 | Defect: direct flight interfaces bypass the JSON flight envelope

**Locations:** [public validation](src/mission_execution/src/mission_execution_node.cpp#L253), lines 253-300 and 393-414; [takeoff/navigation validation](src/vehicle_control/src/vehicle_control_node.cpp#L180), lines 180-215; [setpoint narrowing](src/vehicle_control/src/px4_adapter.cpp#L159), lines 159-163; [JSON-only bounds](src/mission_execution/src/mission_execution_node.cpp#L721), lines 721-747.

**Issue:** JSON routes default to a 30-meter climb and 100-meter horizontal envelope. Direct takeoff and navigation require positive/finite values without equivalent altitude, distance, or destination limits. Raw waypoint missions also accept arbitrary finite offsets. A caller can bypass the documented plan limits by using another public endpoint. Finite `float64` coordinates such as `1e300` also pass navigation validation before narrowing to PX4 `float32`, where they cannot remain finite. Orbit bounds cover center distance but direct requests do not bound center altitude or the whole circle using the same mission envelope.

**Pros of changing:** every command source receives the same physical and representational checks.

**Cons of changing:** previously accepted goals outside the chosen envelope become invalid; mission-relative limits and home-relative fences require explicit definitions.

**Proposed solution:** apply one operation envelope at the execution boundary and reject nonrepresentable values before narrowing. Retain schema-specific checks in the planner. Use PX4 geofencing for independent physical containment rather than reproducing geofence enforcement in several ROS validators. [PX4 geofences](https://docs.px4.io/v1.17/en/flying/geofence).

### ISS-09 | P1 | Defect: raw missions take off before validating later operations

**Locations:** [mission admission](src/mission_execution/src/mission_execution_node.cpp#L390), lines 390-417; [takeoff before route construction](src/mission_execution/src/mission_execution_node.cpp#L825), lines 825-845; [navigation/orbit construction](src/mission_execution/src/mission_execution_node.cpp#L861), lines 861-885 and 903-925.

**Issue:** raw mission admission checks finite fields but omits execution constraints. A mission with an Orbit radius of 21 meters passes admission, takes off, and later fails the default 20-meter vehicle-control limit. A waypoint whose horizontal location equals the look-at point similarly fails only when its navigation child is sent. Summing two finite origin/offset values can also produce an invalid absolute target after admission. The failure path returns without the requested final landing. JSON compilation already attempts whole-route validation before takeoff.

**Pros of changing:** invalid missions fail while disarmed; reduces mid-flight validation failures and endpoint-dependent behavior.

**Cons of changing:** validation must use a captured starting frame and still be repeated for execution-time state changes.

**Proposed solution:** normalize and validate every requested operation against the same limits before takeoff, including absolute target sums, look-at geometry, Orbit limits, and required references. Remove the separate permissive raw-mission validator or retire this ingress if the JSON contract covers its users.

### ISS-10 | P1 | Defect: runtime safety-parameter updates report success without taking effect

**Locations:** [vehicle parameter declarations](src/vehicle_control/src/vehicle_control_node.cpp#L130), lines 130-144; [mission parameters](src/mission_execution/src/mission_execution_node.cpp#L326), lines 326-354; compare [immutable planner parameters](src/mission_planner/mission_planner/node.py#L21), lines 21-27.

**Issue:** both C++ nodes declare writable ROS parameters and copy them into fields once. They register no parameter-update callback. `ros2 param set` can report that `enable_flight_commands` was disabled or that a timeout/bound changed while execution still uses its startup value. PX4 prefix/system-ID changes have the same discrepancy between reported and effective configuration.

**Evidence:** a runtime change to `enable_flight_commands=false` returned a successful parameter result on an initially enabled node. A subsequent takeoff goal was accepted.

**Pros of changing:** reported configuration matches effective behavior; removes a misleading disable mechanism.

**Cons of changing:** marking parameters immutable requires a restart to apply changes. A real in-flight disable operation needs a defined hold/landing handoff.

**Proposed solution:** mark startup configuration read-only, following the planner's existing descriptor pattern. Provide a runtime operation only for a demonstrated requirement, with explicit state transition and synchronization. Do not add generic dynamic parameter plumbing for every field.

## Execution correctness and resource use

### ISS-11 | P2 | Defect: finite timeouts can overflow steady-clock conversion

**Locations:** [vehicle deadline_after](src/vehicle_control/src/vehicle_control_node.cpp#L93), lines 93-98; [timeout admission](src/vehicle_control/src/vehicle_control_node.cpp#L211), lines 211-215, 334-335, and 362-366; compare [saturating mission deadline](src/mission_execution/src/mission_execution_node.cpp#L309), lines 309-318.

**Issue:** `std::isfinite` and a sign check do not establish that a duration fits the clock representation. `duration_cast` converts the supplied seconds into an integral tick count. A legal action value near `FLT_MAX` overflows that conversion. Large startup timeout parameters have the same problem. Mission execution already contains a different helper that checks the remaining clock range.

**Evidence:** compiling the vehicle helper's conversion with `-fsanitize=float-cast-overflow` and supplying `FLT_MAX` seconds reported `3.40282e+47` outside the range of `long int`.

**Application reproduction:** the later torture suite submitted a `3e38`-second timeout through the real navigation action to a UBSan-instrumented vehicle-control executable. It accepted the goal and exited with code 1. Its directly written sanitizer file reports an out-of-range `3e+47` conversion in `std::chrono`, with `deadline_after` and `execute_navigation` in the stack. See the [retained report](run-logs/torture/ubsan-20261113-s1731/report.md).

**Pros of changing:** removes undefined numeric conversion and gives all actions bounded, predictable deadlines.

**Cons of changing:** practical maximum durations must be specified; saturation alone can allow operations to occupy a worker for an unreasonable period.

**Proposed solution:** reject durations outside one documented operational range before conversion. Use the same checked deadline calculation in both nodes. Bound configured defaults as well as action values. Avoid accepting a centuries-long timeout merely because a clock can represent it.

### ISS-12 | P2 | Defect: action success precedes release of the execution reservation

**Locations:** [takeoff completion](src/vehicle_control/src/vehicle_control_node.cpp#L946), lines 946-954; [navigation completion](src/vehicle_control/src/vehicle_control_node.cpp#L1083), lines 1083-1103; [mission worker cleanup](src/mission_execution/src/mission_execution_node.cpp#L595), lines 595-605; [goal reservation](src/vehicle_control/src/vehicle_control_node.cpp#L469), lines 469-472.

**Issue:** vehicle actions publish a terminal result before `finish_goal` clears `goal_active_`. A client can receive success and immediately send its next operation while the old reservation remains set. The next request is rejected as busy. Mission execution similarly publishes results before its wrapper releases `command_active_`. The normal waypoint sequencer uses immediate child chaining, so this ordering is part of the production path. This is an ordering race identified in source, not a reproduced failure in this review.

**Pros of changing:** a terminal result becomes a usable handoff point and removes timing-dependent rejection of sequential commands.

**Cons of changing:** cleanup must complete safely before allowing the next request; releasing ownership before final target/stream state is committed would create another race.

**Proposed solution:** commit target/stream cleanup and release the matching operation reservation before publishing the terminal result. Tie cleanup to that operation's generation so late cleanup cannot release a newer operation. Apply one terminal-completion path per node.

### ISS-13 | P2 | Simplification: landing duplicates PX4 auto-disarm and adds an unrelated mode requirement

**Locations:** [execute_land](src/vehicle_control/src/vehicle_control_node.cpp#L1408), lines 1408-1450; [Land contract](src/swarm_interfaces/action/Land.action), lines 1-8; [landing documentation](docs/simulation/HOW-IT-WORKS.md#L19), line 19.

**Issue:** after touchdown, the action explicitly disarms, requests Auto Loiter, and requires that navigation state for success. A successful physical landing can therefore be reported as failed because a post-landing mode change was denied or its ACK was lost. PX4 already provides land detection and configurable automatic disarming through `COM_DISARM_LAND`. Immediate companion-commanded disarm also replaces PX4's configured landed hysteresis with application timing. [PX4 Land behavior](https://docs.px4.io/v1.17/en/flight_modes_mc/land), [v1.17 commander implementation](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/commander/Commander.cpp#L2094).

**Pros of changing:** removes two command transitions, retries, timeout paths, and the Auto Loiter completion condition.

**Cons of changing:** auto-disarm must be enabled and its delay included in the action deadline; deployments that intentionally disable auto-disarm need an explicit policy.

**Proposed solution:** let PX4 land and auto-disarm; succeed after fresh landed/disarmed telemetry. Remove Auto Loiter from landing success and stop the owned stream at the confirmed handoff. Retain a manual disarm operation only if an operational requirement needs it.

### ISS-14 | P2 | Defect: telemetry freshness and unknown state lose source information

**Locations:** [adapter callbacks](src/vehicle_control/src/px4_adapter.cpp#L54), lines 54-130; [application state publication](src/vehicle_control/src/vehicle_control_node.cpp#L730), lines 730-765; [mission state checks](src/mission_execution/src/mission_execution_node.cpp#L475), lines 475-496; [VehicleState](src/swarm_interfaces/msg/VehicleState.msg), lines 1-25.

**Issue:** freshness measures callback receipt time without checking source timestamp progression. Repeated or delayed retained samples can refresh the cache. `VehicleState.header.stamp` is publication time, which hides the age of the underlying sample. Stale status/land telemetry is represented as `armed=false`, `offboard=false`, or `landed=false`, without a separate validity flag. Battery availability never expires after a valid battery message. Consumers cannot distinguish a confirmed state from an unavailable state, and the displayed battery can remain current-looking after its stream stops.

**Pros of changing:** consumers can reject incomplete snapshots and report unknown state accurately.

**Cons of changing:** source-age checks require an explicit PX4/ROS time-synchronization contract and handling of PX4 restart.

**Proposed solution:** reject nonprogressing source samples within a session, preserve source-age/validity information, and represent control/land validity explicitly. Add a battery receive time with an appropriate timeout for its lower publication rate. Keep time synchronization in the adapter; avoid separate clock heuristics in every consumer.

### ISS-15 | P2 | Defect: one ACK cache conflates command transactions

**Locations:** [ACK subscription](src/vehicle_control/src/px4_adapter.cpp#L132), lines 132-144; [command identity](src/vehicle_control/src/px4_adapter.cpp#L209), lines 209-230; [acknowledgement_after](src/vehicle_control/src/px4_adapter.cpp#L297), lines 297-307; [ACK waiter](src/vehicle_control/src/vehicle_control_node.cpp#L587), lines 587-605.

**Issue:** every ACK addressed to system 255/component 1 overwrites one cache entry. An ACK for a different command can replace the expected ACK before the 50-millisecond polling loop reads it. Another client using the same identity can contribute a matching command ACK. A delayed ACK from an earlier retry also satisfies the generation test because generation records local arrival order, not command identity or age. This can produce unnecessary retries or attribute a rejection/success to the wrong request. Transition telemetry limits some consequences but does not repair transaction matching.

**Pros of changing:** reduces false command failure and removes retries caused by cache overwrites.

**Cons of changing:** PX4 command ACKs do not provide a general application transaction ID; perfect correlation cannot be invented locally.

**Proposed solution:** use a dedicated component identity, serialize the one outstanding command, and retain its relevant ACK until consumed. Reconcile retries with observed state before resending. Prefer an established PX4 command/mode client if adopted under ISS-28 rather than building a general transaction framework.

### ISS-16 | P2 | Defect and simplification: trajectory validation does not bound the commanded acceleration

**Locations:** [trajectory validation](src/vehicle_control/src/trajectory.cpp#L119), lines 119-146; [linear interpolation](src/vehicle_control/src/trajectory.cpp#L153), lines 153-188; [setpoint publication](src/vehicle_control/src/px4_adapter.cpp#L160), lines 160-175; [accepted test schedule](src/vehicle_control/test/test_trajectory.cpp#L31), lines 31-48.

**Issue:** the validator estimates acceleration from differences between average segment velocities divided by midpoint spacing. The interpolator remains piecewise linear, with instantaneous velocity changes at corners and start/stop. The computed 3 m/s² bound therefore does not bound the interpolated reference's acceleration. Two-point routes have no acceleration check at all. The adapter sends position only, with velocity, acceleration, and jerk unset, so these schedule limits are also not direct limits on measured vehicle motion.

**Pros of changing:** removes misleading safety guarantees and can delete custom interpolation/rate-limiting code for ordinary routes.

**Cons of changing:** PX4 go-to smoothing does not preserve arbitrary exact waypoint timestamps. Adding a custom smooth trajectory generator would increase code substantially.

**Proposed solution:** use PX4's go-to position/heading smoother for waypoint, search-lane, and altitude-step operations that do not require exact timing. Preserve timed trajectories only for a documented timing requirement, with explicit continuity and tracking semantics. PX4 v1.17 already generates position, velocity, acceleration, and jerk references in [GotoControl](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mc_pos_control/GotoControl/GotoControl.cpp#L62).

### ISS-17 | P2 | Defect: timed trajectory success proves only the final endpoint

**Locations:** [trajectory loop](src/vehicle_control/src/vehicle_control_node.cpp#L1245), lines 1245-1283; [runner ordered-visit check](src/flight_test/flight_test/runner.py#L469), lines 469-498.

**Issue:** the active target advances with wall elapsed time regardless of tracking error. The action checks position/yaw only after the schedule reaches its final point. A vehicle that lags, misses an outbound leg, or cuts a search lane can catch the final point and return success. The runner independently checks ordered target visits for some cases, which does not protect production callers of `FollowTrajectory`. The configured position tolerance is used for final settling rather than as an execution tracking bound.

**Pros of changing:** action success can establish that the requested path was flown, with defined error limits.

**Cons of changing:** enforcing timing and path error can abort otherwise recoverable routes in wind; progress-based pacing changes the meaning of a timed trajectory.

**Proposed solution:** choose one contract. Use PX4 waypoint/go-to execution when path coverage is required without exact timing. For a retained timed action, define a bounded sustained tracking-error failure and specify what success guarantees. Reuse existing ordered-visit evidence in tests; do not create another generic planner.

### ISS-18 | P2 | Defect: minimum stream rate and prestream checks provide no timing margin

**Locations:** [rate limits](src/vehicle_control/src/vehicle_control_node.cpp#L28), lines 28-30 and 150-153; [timer construction](src/vehicle_control/src/vehicle_control_node.cpp#L171), lines 171-174; [prestream wait](src/vehicle_control/src/vehicle_control_node.cpp#L698), lines 698-713.

**Issue:** a configured rate of exactly 2 Hz is accepted at PX4's Offboard liveness threshold. Scheduling jitter leaves no margin. Prestream waits one wall-clock second without verifying that publications actually occurred at a sufficient cadence. A busy callback group or delayed first timer can satisfy the application wait while PX4 has not received the required stream. Millisecond truncation also makes some configured frequencies differ from their requested values. [PX4 Offboard requirements](https://docs.px4.io/v1.17/en/flight_modes/offboard).

**Pros of changing:** reduces avoidable Offboard-entry rejection and detects delayed heartbeat scheduling.

**Cons of changing:** publication counts alone cannot prove DDS delivery; end-to-end timing still requires PX4 state and SITL evidence.

**Proposed solution:** prefer the existing fixed 20-Hz rate and remove the exposed rate knob unless another rate is needed. Verify publication cadence during prestream and bound timer gaps. A library-managed PX4 mode can remove this custom prestream path.

### ISS-19 | P2 | Defect: action deadlines do not bound the complete operation

**Locations:** [transition deadlines](src/vehicle_control/src/vehicle_control_node.cpp#L620), lines 620-692; [takeoff deadline](src/vehicle_control/src/vehicle_control_node.cpp#L875), lines 875-936; [landing deadline](src/vehicle_control/src/vehicle_control_node.cpp#L1351), lines 1351-1353 and 1408-1442; [Orbit recovery](src/vehicle_control/src/vehicle_control_node.cpp#L1529), lines 1529-1562.

**Issue:** each command retry creates new ACK and transition deadlines, independent of the action deadline. Takeoff can spend its entire nominal 30 seconds in prestream/mode/arm transitions before its climb loop starts. Landing's descent deadline does not include the later disarm/Loiter waits. Orbit can enter recovery at its deadline and spend another prestream/retry/settle interval. Parent deadlines and cancellation grace then disagree with lower-level completion times.

**Pros of changing:** bounds operation and landing-preemption latency and removes timeout arithmetic duplicated across layers.

**Cons of changing:** recovery after an expired motion deadline may need a separately documented budget; abruptly abandoning an active landing is inappropriate.

**Proposed solution:** pass one checked overall deadline through ordinary operation stages and cap each wait by its remaining budget. Define terminal landing and recovery budgets explicitly. Remove timeout multipliers and margins that compensate for other layers without a shared contract.

### ISS-20 | P2 | Safety gap: unbounded ROS payloads reach a shared control callback group

**Locations:** [mission interface](src/swarm_interfaces/action/ExecuteWaypointMission.action), lines 1-6; [trajectory interface](src/swarm_interfaces/action/FollowTrajectory.action), lines 1-3; [JSON interface](src/swarm_interfaces/action/ExecuteFlightPlan.action), lines 1-3; [raw admission loop](src/mission_execution/src/mission_execution_node.cpp#L393), lines 393-414; [executor](src/vehicle_control/src/main.cpp#L12), lines 12-15.

**Issue:** action sequences and strings are unbounded in the interface definitions. Limits enforced inside goal callbacks occur after middleware deserialization/allocation. Raw missions permit almost `UINT32_MAX` waypoints and scan the entire sequence. The vehicle timers, action servers, and subscriptions use the default mutually exclusive callback group; a three-thread executor does not isolate the heartbeat from a long callback in that group. Oversized requests can therefore consume memory and delay control telemetry/heartbeat work without requiring a memory leak. [ROS Jazzy default callback group](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/node_interfaces/node_base.cpp#L134).

**Pros of changing:** bounds allocation and validation work and makes the control scheduling budget reviewable.

**Cons of changing:** bounded ROS interface definitions require rebuilding all participants; callback-group separation adds concurrency that must be audited.

**Proposed solution:** bound sequence/string sizes in the ROS contracts and apply a practical raw-mission cap consistent with the planner's 256-waypoint limit. Keep admission work bounded. Use a single executor if that suffices; isolate the control timer only if measured callback latency requires it. Do not assume additional executor threads provide isolation automatically.

### ISS-21 | P2 | Defect: worker shutdown and exception handling can leave operations unresolved

**Locations:** [vehicle destructor](src/vehicle_control/src/vehicle_control_node.cpp#L387), lines 387-395; [vehicle worker catch](src/vehicle_control/src/vehicle_control_node.cpp#L847), lines 847-865; [vehicle cancellation predicates](src/vehicle_control/src/vehicle_control_node.cpp#L873), lines 873-874 and 1013-1014; [mission worker catch](src/mission_execution/src/mission_execution_node.cpp#L595), lines 595-605.

**Issue:** vehicle destruction sets `worker_stopping_`, but active jobs check only goal cancellation and global ROS shutdown. Destroying/removing the node while the ROS context remains alive can block `join()` until the goal's potentially long deadline. A job-level exception escaping an action handler is logged and releases a reservation without guaranteeing a terminal action result. A caller can then retain a pending child indefinitely. Normal executable shutdown calls ROS shutdown first, so the destructor problem specifically affects independent node teardown and test/composition use.

**Additional runtime evidence:** the later 36-case torture run recorded vehicle-control exit `-6` (SIGABRT) during teardown in the finite-position-overflow and mission-owner-crash cases. The first native log ends at the ROS SIGTERM handler; the second native log is empty. The retained timeline identifies the active goal, injected fault, and exit code. These observations establish abnormal teardown, but the available C++ diagnostics do not establish that the worker path above caused either abort. See the [torture-test validation record](docs/testing/TORTURE-TESTS.md#recorded-validation).

**Pros of changing:** bounds teardown and makes exception paths preserve the action lifecycle.

**Cons of changing:** shutdown cannot guarantee communication of a terminal result after the ROS context is already gone.

**Proposed solution:** include a local stop flag in every worker wait and stop accepting/queuing new work during teardown. Give each queued operation a terminal-failure path and a scope-bound reservation cleanup. Keep the worker owned and joined; no detached threads or new scheduler are required.

## Flight tests and command clients

### ISS-22 | P2 | Defect: a latched flight-test failure suppresses recovery landing

**Locations:** [telemetry fault latch](src/flight_test/flight_test/runner.py#L216), lines 216-232; [current sample](src/flight_test/flight_test/runner.py#L311), lines 311-330; [landing helper](src/flight_test/flight_test/runner.py#L423), lines 423-433; [recovery handler](src/flight_test/flight_test/runner.py#L1332), lines 1332-1343.

**Issue:** a sample gap or unexpected flight-mode change sets `telemetry_fault`. `_current_sample` then always throws on that latched fault, even if later telemetry is fresh. The case's failure handler calls `_current_sample` before deciding to land, so recovery exits before sending a landing request. `_land` calls the same helper. A test can therefore report a telemetry failure and leave a recoverable airborne vehicle without attempting its advertised recovery action. The same path appears after an action timeout if a telemetry fault was recorded while waiting.

**Pros of changing:** preserves the failed test verdict while permitting recovery from fresh current state.

**Cons of changing:** recovery must still reject unusable telemetry and respect operator takeover; a failed test does not authorize reclaiming Manual mode.

**Proposed solution:** separate the latched test verdict from access to fresh recovery telemetry. Reuse one recovery-state check that ignores historical assertion faults, checks current mode/validity, and requests landing through mission execution when appropriate. Add a focused case for a recovered sample gap. Do not weaken the test's pass/fail thresholds.

### ISS-23 | P2 | Defect: flight-test locking is scoped to artifact directories

**Locations:** [lock selection](src/flight_test/flight_test/runner.py#L1496), lines 1496-1507; [failure landing](src/flight_test/flight_test/runner.py#L1335), lines 1335-1339; [readiness](src/flight_test/flight_test/runner.py#L278), lines 278-305; [lock test](src/flight_test/test/test_runner_artifacts.py#L63), lines 63-85.

**Issue:** the exclusive lock belongs to a run's `flight-tests` directory or the chosen artifact parent. Two processes using different run IDs or result parents acquire different locks while commanding the same root vehicle. Both can pass landed readiness before either starts. Mission execution rejects conflicting work, but the rejected runner's failure handler can then request explicit landing and preempt the other runner's flight. The existing test checks contention only within one lock directory.

**Pros of changing:** avoids overlapping test ownership and recovery actions directed at another test's vehicle.

**Cons of changing:** a host-local lock does not coordinate runners on different companion computers.

**Proposed solution:** key the local test lock to the ROS domain and controlled vehicle namespace, independently of output paths. Keep the artifacts per run. Let the existing command authority reject distributed conflicts; add ownership semantics there only if distributed test execution becomes a requirement.

### ISS-24 | P2 | Defect: the JSON CLI has unbounded response and result waits

**Locations:** [CLI goal-response loop](src/mission_planner/mission_planner/cli.py#L55), lines 55-59; [result/cancel loop](src/mission_planner/mission_planner/cli.py#L64), lines 64-75; [documented Ctrl-C behavior](src/mission_planner/README.md#L21), line 21.

**Issue:** the CLI bounds discovery to five seconds, then waits indefinitely for acceptance and a terminal result while the endpoint remains discoverable. A stalled server can stay discoverable. Ctrl-C sets a flag but the acceptance loop does not inspect it, so cancellation cannot progress while a response is missing. During result waiting, cancellation is sent once and the client can again wait forever if its terminal result is lost. Repeated Ctrl-C only sets the same flag.

**Pros of changing:** bounded operator feedback and an explicit distinction between confirmed cancellation and unknown flight outcome.

**Cons of changing:** timing out a CLI does not stop flight. Destroying the client before late acceptance can leave an operation running.

**Proposed solution:** use bounded acceptance and cancellation-drain waits with late-response handling. Report unknown outcome when confirmation is unavailable, including the existing explicit landing/recovery command. Derive mission-result expectations from the execution contract instead of adding a second arbitrary mission timer.

### ISS-25 | P2 | Safety gap: automated coverage bypasses most of the real flight-action state machine

**Locations:** [vehicle tests](src/vehicle_control/CMakeLists.txt#L43), lines 43-66; [controlled child fixture](src/mission_execution/test/test_command_authority.cpp#L41), lines 41-99; [fake flight backend](src/drone_bringup/test/test_planner_execution_flow.py#L35), lines 35-89; [flight validation limits](docs/testing/JSON-PLANNER-VALIDATION.md#L58), lines 58-60.

**Issue:** vehicle-control tests cover coordinate math, trajectory math, Orbit progress, and adapter messages. The real `VehicleControlNode` action implementation has no corresponding lifecycle/fault tests. Mission tests replace it with action servers that succeed/cancel under fixture control, and the planner integration fixture changes takeoff state immediately. Those tests cannot detect unsolicited streams, ignored parameter writes, mode loss, ACK handling, or hold semantics. Historical validation also retains unresolved FT-03 and FT-06 failures and did not fly JSON circle/lawnmower routes.

**Pros of changing:** detects flight-node regressions without requiring Gazebo for every failure case.

**Cons of changing:** a fake PX4 backend cannot prove braking, estimator behavior, or airframe stability.

**Proposed solution:** add one bounded synthetic-PX4 fixture around the real vehicle node and cover the failures in ISS-01 through ISS-21. Retain a small physical/SITL acceptance set for behaviors that depend on dynamics. Reduce duplicated happy-path routing assertions as new fault cases replace them. Keep historical flight failures open until new evidence resolves them.

## Code reduction and PX4 delegation

### ISS-26 | P2 | Simplification: overlapping command APIs duplicate validation and execution state

**Locations:** [public operation servers](src/mission_execution/src/mission_execution_node.cpp#L359), lines 359-377; [raw/JSON mission servers](src/mission_execution/src/mission_execution_node.cpp#L390), lines 390-435; [mission conversion](src/mission_execution/src/mission_execution_node.cpp#L721), lines 721-763; [vehicle servers](src/vehicle_control/src/vehicle_control_node.cpp#L178), lines 178-378.

**Issue:** callers can express a route through JSON, raw waypoint missions, individual navigation goals, or timed trajectories. Five primitive operations also have public proxy servers and internal action servers. The mission executor has a worker, pending-child state, cancellation grace, operation validation, and timeouts; vehicle control has another worker, reservation, validation, and timeout system. The JSON path constructs a raw mission to reuse some sequencing, but its safety checks still differ from the raw API. These differences produce ISS-05, ISS-08, ISS-09, and ISS-19.

**Pros of changing:** fewer public contracts and state combinations; fewer opportunities for safety behavior to diverge.

**Cons of changing:** the primitive API supports tests and manual operations, and the PX4-independent mission boundary is useful. Merging everything into a PX4-aware mission node would increase coupling.

**Proposed solution:** keep one mission intent contract and a small explicit operator-operation API. Normalize both to one validated operation representation. Retire unused route ingress after checking its consumers. Preserve the logical PX4 adapter boundary, but evaluate whether internal ROS actions/process separation earn their worker/cancellation cost. Do not introduce a generic behavior-tree engine to replace this small sequencer.

### ISS-27 | P2 | Simplification: static routes could execute as onboard PX4 missions

**Locations:** [waypoint sequencing](src/mission_execution/src/mission_execution_node.cpp#L817), lines 817-951; [planner instructions](src/mission_planner/mission_planner/compiler.py#L253), lines 253-323; [GPS contract](src/mission_planner/README.md#L44), lines 44-52.

**Issue:** fixed waypoint and search routes currently require the companion to sequence each waypoint, sustain Offboard control, judge arrival, and reach the final landing. PX4 Mission mode already handles uploaded geographic waypoint routes, takeoff/land items, progress, and pause/resume. The codebase pays companion liveness and cancellation costs for routes known before takeoff. [PX4 v1.17 Mission mode](https://docs.px4.io/v1.17/en/flight_modes_mc/mission).

**Pros of changing:** can remove the per-waypoint action loop and much of its timeout/hold machinery for static routes; the flight controller owns route progression; QGroundControl can display standard missions.

**Cons of changing:** standard missions require a suitable global estimate and geographic conversion, use different arrival/yaw/timing semantics, and continue according to PX4's configured link-loss policy when the companion disappears. Upload/control needs a supported transport such as MAVSDK; the current DDS adapter does not implement mission upload. Native Orbit completion and arbitrary timed trajectories are not automatic equivalents.

**Proposed solution:** evaluate one complete GPS/search mission through a pinned mission client and measure the application code it replaces. Adopt it only if static routes are the product requirement and cancellation/landing behavior passes SITL. Replace the existing backend for that requirement instead of permanently maintaining two equivalent route executors.

### ISS-28 | P2 | Simplification: PX4 mode executors can replace custom lifecycle polling

**Locations:** [ACK/mode waits](src/vehicle_control/src/vehicle_control_node.cpp#L556), lines 556-713; [takeoff lifecycle](src/vehicle_control/src/vehicle_control_node.cpp#L869), lines 869-995; [Orbit recovery](src/vehicle_control/src/vehicle_control_node.cpp#L1483), lines 1483-1563; [adapter commands](src/vehicle_control/src/px4_adapter.cpp#L184), lines 184-230.

**Issue:** the project implements command publication, ACK correlation, retries, arming, prestream, mode confirmation, mode ownership, and completion polling. PX4's ROS 2 interface library exposes registered modes and a `ModeExecutorBase` with takeoff, land, arm/disarm, mode scheduling, completion callbacks, and ownership status. These map to substantial parts of the current lifecycle machinery. [ModeExecutorBase API](https://auterion.github.io/px4-ros2-interface-lib/classpx4__ros2_1_1ModeExecutorBase.html).

**Pros of changing:** transfers mode/command protocol maintenance to the PX4 integration library and can remove custom polling and retry code. Native Takeoff also removes the application's Offboard-before-arm climb sequence.

**Cons of changing:** adds a pinned dependency and registered-mode lifecycle; some interfaces remain experimental. A compatible library commit must be validated against v1.17.0 rather than following `main`. Takeoff altitude/yaw contracts need conversion, and ROS action results still require an application adapter. [PX4 control-interface status](https://docs.px4.io/v1.17/en/ros2/px4_ros2_control_interface).

**Proposed solution:** prototype replacement of takeoff, landing, and one go-to operation behind the existing application boundary. Compare deleted production code and fault cases, not just example size. Keep application admission and mission-origin semantics. Adopt one lifecycle backend; do not wrap the library while leaving the old retry engine active.

### ISS-29 | P3 | Simplification: legacy schema branches and a no-op circle field add contracts

**Locations:** [legacy selection](src/mission_planner/mission_planner/compiler.py#L215), lines 215-224; [legacy parser](src/mission_planner/mission_planner/flight_plan.py#L209), lines 209-267; [unused circle spacing](src/mission_planner/mission_planner/flight_plan.py#L128), lines 128-153; [compatibility documentation](src/mission_planner/README.md#L64), line 64 and lines 82-84.

**Issue:** the runtime accepts version 1 absolute local-origin routes, version 2 relative routes, version 2 shared-origin routes, and GPS routes. The same altitude field has different absolute/relative meaning in version 1 and version 2. Circle `point_spacing_m` is validated but does not affect execution, including in the version 2 path. The behavior is documented, so this is compatibility cost rather than an undocumented implementation error. The legacy parser's standalone entry point is used by tests while production compilation takes a different entry path.

**Pros of changing:** fewer coordinate interpretations, parser branches, compatibility tests, and inert settings.

**Cons of changing:** existing SAR files and external callers can break; conversion of absolute local coordinates needs the correct origin.

**Proposed solution:** inventory actual legacy users. Reject the no-op spacing field in new schemas and deprecate version 1 when migration is possible. Use an offline converter if legacy import remains necessary; keep runtime execution on one coordinate contract. Retain version 1 if active users require it, with its cost explicitly accepted.

### ISS-30 | P3 | Simplification: launch defaults and safety limits have multiple owners

**Locations:** [launch argument declarations](src/drone_bringup/launch/single_vehicle.launch.py#L14), lines 14-56 and 66-167; [vehicle defaults](src/vehicle_control/include/vehicle_control/vehicle_control_node.hpp#L123), lines 123-134; [planner limits](src/mission_planner/mission_planner/compiler.py#L22), lines 22-53; [executor limits](src/mission_execution/src/mission_execution_node.cpp#L27), lines 27-36 and 326-345.

**Issue:** defaults appear in launch declarations, C++ member initializers, C++ parameter declarations, Python dataclasses, and prose. Several limits also differ by entry point: trajectory extent is 20/8 meters, JSON extent defaults to 100/30 meters, and direct navigation lacks those bounds. Launch forwards more than twenty individual settings using repeated parameter conversions. Updating a safety profile requires finding each representation.

**Pros of changing:** fewer independent defaults and less launch boilerplate; makes effective configuration reviewable.

**Cons of changing:** independent input validation across a ROS process boundary remains necessary; sharing a Python constant directly with C++ would add tooling.

**Proposed solution:** retain one immutable deployed ROS parameter profile and expose only operationally needed launch arguments such as namespace, PX4 prefix/system ID, enablement, and log level. Keep schema validation separate from execution limits. Remove redundant member initializers and repeated prose tables where the canonical profile or interface already defines the value. Add no configuration generator unless duplication remains after consolidation.

### ISS-31 | P3 | Simplification: the wind model copies upstream airframe assets

**Locations:** [custom model](worlds/models/vip_x500/model.sdf), lines 1-75; [custom base](worlds/models/vip_x500_base/model.sdf#L1), lines 1-28; [custom server configuration](px4/server.config), lines 1-22; [model selection](scripts/px4-sitl#L110), lines 110-129; [asset description](worlds/README.md#L3), lines 3-5 and 21.

**Issue:** the project owns copies of PX4's X500/base model, meshes, textures, motor/sensor configuration, and a server-plugin list to support its wind-enabled variant. The documented airframe change is enabling wind on the body link. Future PX4 upgrades require comparing these copies and plugin lists with upstream; a software update can otherwise retain stale simulation dynamics. The tracked `px4-vip-x500/x500` symlink exists and resolves correctly, so its launcher mapping is not a missing-path defect.

**Pros of changing:** reduces vendored assets and upgrade comparisons if a small SDF override can express the required change.

**Cons of changing:** Gazebo include/merge and link-override behavior must preserve wind and resource resolution; an invalid override can silently restore a wind-disabled vehicle.

**Proposed solution:** verify whether a wrapper model can include the pinned stock model and override only wind behavior. Remove copied assets only after model/physics verification. If the copy is required, retain it with the upstream tag, a minimal recorded diff, and an upgrade comparison. Avoid implementing another aerodynamic model solely to remove this copy.

### ISS-32 | P2 | Simplification: ordinary build/test commands depend on desktop simulation startup

**Locations:** [sim-shell startup](scripts/sim-shell#L44), lines 44-64; [display gate](scripts/sim#L43), lines 43-56 and 130-142; [simulation GPU reservation](compose.yaml#L53), lines 53-68; [duplicate service configuration](compose.yaml#L2), lines 2-31 and 33-74; [documented test command](docs/testing/README.md#L159), lines 159-165.

**Issue:** the documented shell/test path always starts the simulation service. It requires `DISPLAY`, `xhost`, and an NVIDIA GPU reservation before a non-GUI command such as `colcon test` can run. A separate development service already exists, but the normal shell wrappers do not use it. Both services repeat build arguments and runtime configuration while using different mounts and build storage. Contributors and headless runners must choose another invocation to run the same source tests.

**Pros of changing:** build/test access without a desktop or GPU; fewer duplicated service settings and startup branches.

**Cons of changing:** GUI simulation still needs explicit display/GPU setup; existing editor workflows depend on the development service's volumes.

**Proposed solution:** make a non-GUI development shell the canonical build/test path and start the simulation profile only for flight simulation. Share the common image/build definition. Retain explicit GUI access lifecycle for Gazebo and avoid starting PX4/Agent for source tests.

## Documentation and build maintenance

### ISS-33 | P1 | Documentation defect: stopping the application is described as stopping flight

**Locations:** [Linux guide](docs/setup/LINUX.md#L104), lines 104-106; [node shutdown](src/vehicle_control/src/main.cpp#L15), lines 15-16; [CLI cancellation documentation](src/mission_planner/README.md#L21), line 21.

**Issue:** the Linux guide instructs the operator to press Ctrl-C in the launch shell to stop a flight. That stops application processes and their heartbeat; it does not issue or confirm Land. PX4 then follows its configured Offboard-loss and other failsafe behavior. The planner CLI's Ctrl-C has another meaning: cancel and retain navigation hold. The same key therefore means process shutdown or action cancellation depending on the terminal. Neither establishes a landed vehicle. [PX4 Offboard-loss behavior](https://docs.px4.io/v1.17/en/flight_modes/offboard).

**Pros of changing:** gives operators a recovery procedure that can be verified from vehicle telemetry.

**Cons of changing:** the procedure needs a configured PX4/operator recovery path and a distinct process-shutdown step.

**Proposed solution:** document explicit Land/QGroundControl recovery, wait for fresh landed and disarmed state, then stop application/container processes. State the expected response to heartbeat loss for the deployed PX4 parameter profile. Explain launch-shell shutdown and CLI cancellation at their respective command examples.

### ISS-34 | P3 | Documentation defect and simplification: operational facts are repeated and have drifted

**Locations:** [root flight procedure](README.md#L63), lines 63-101; [Linux startup](docs/setup/LINUX.md#L98), lines 98-106; [tmux guide](docs/development/TMUX.md#L35), lines 35-98; [flight-test guide](docs/testing/README.md#L12), lines 12-43 and 167-168; [simulation fidelity](docs/simulation/SIMULATION-FIDELITY.md#L3), line 3; [Pi process description](docs/setup/pi-desktop-sim.md#L160), line 160; [codebase tour](docs/architecture/CODEBASE-TOUR.md#L193), lines 193-219.

**Issue:** startup, command ownership, safety defaults, coordinate conversion, and landing behavior recur across root, setup, architecture, simulation, and package guides. Existing drift includes the fidelity page naming the stock X500 while defaults use `vip_x500`, the test guide naming the default `lawn` world while defaults use `vip_windy`, and the Pi guide naming two launched nodes where launch starts three. The codebase tour embeds hand-maintained line ranges that no longer match function boundaries. The root procedure also interleaves automatic and manual tmux startup descriptions.

**Pros of changing:** fewer places to update after a flight contract or setup change; a shorter operator path.

**Cons of changing:** readers may need to follow one link instead of finding every detail on each page; background architecture and draft policy still need their own documents.

**Proposed solution:** keep one startup/recovery runbook, one interface/coordinate contract, and one build/test guide. Make package READMEs short responsibility/API indexes. Link to canonical facts from setup variants. Keep the arbitration draft labeled as future work and remove current-state material already owned elsewhere. Remove numeric prose line ranges from the tour or generate them from source.

### ISS-35 | P2 | Documentation defect: validation evidence is unavailable and not bound to an executable configuration

**Locations:** [command validation links](docs/testing/COMMAND-ARCHITECTURE-VALIDATION.md#L53), lines 53-65; [JSON validation links](docs/testing/JSON-PLANNER-VALIDATION.md#L38), line 38 and lines 46-52; [.gitignore](.gitignore#L1), lines 1-5; [run metadata](scripts/new-run#L37), lines 37-50.

**Issue:** the two validation records link to ignored local run directories. A link scan found 24 absent local targets in this checkout, all belonging to those evidence links. The records identify a date and branch, but do not bind each result to an immutable source commit, effective parameter set, image digest, or model/world revision. `new-run` records only a short Git ID, branch, and dirty flag before processes start. A dirty source tree and previously built C++ binaries can differ, and archived flights used `lawn` before the current wind-enabled default.

**Pros of changing:** reviewers can retrieve evidence and know which binaries/environment it validates.

**Cons of changing:** retaining full ULogs and telemetry consumes storage; durable artifact hosting needs a team location.

**Proposed solution:** keep concise validation records in Git and publish bulky artifacts to one durable location with checksums. Record full source/build IDs, image digest, effective ROS/PX4 parameters, and world/model IDs in the run manifest. State historical results as historical. Remove broken local links when artifacts cannot be recovered; do not rewrite past flights as validation of the new default.

### ISS-36 | P2 | Build defect: production-only CMake still requires test tooling

**Locations:** [vehicle CMake](src/vehicle_control/CMakeLists.txt#L7), lines 7-8 and 43-44; [mission CMake](src/mission_execution/CMakeLists.txt#L7), lines 7-8 and 30-31; [Pi dependency/build instructions](docs/setup/HITL-PI-SETUP.md#L130), lines 130-135; [test-only manifest declarations](src/vehicle_control/package.xml#L14), line 14, and [mission manifest](src/mission_execution/package.xml#L15), line 15.

**Issue:** both C++ packages call `find_package(ament_cmake_gtest REQUIRED)` outside `if(BUILD_TESTING)`. The Pi guide explicitly installs only build/build-export/buildtool/exec dependencies and then builds with `BUILD_TESTING=OFF`. Since `ament_cmake_gtest` is declared as a test dependency, a minimal environment following that guide can fail configuration even though no tests were requested. The cached review image already contains gtest, so this path was identified from the manifests/CMake rather than reproduced on a clean Pi.

**Pros of changing:** production builds match declared dependency categories and need fewer packages.

**Cons of changing:** test-enabled builds still need explicit test dependency installation; developer images can retain gtest.

**Proposed solution:** move the test-tool `find_package` calls inside the existing `BUILD_TESTING` blocks. Document test dependency installation only in the test workflow. Remove the unnecessary native Ruff setup from a production-only path if launch lint is intentionally disabled there.

### ISS-37 | P2 | Observed test failure: command-authority tests contain unbounded waits

**Locations:** [Orbit cancellation test](src/mission_execution/test/test_command_authority.cpp#L314), lines 314-330; [fixture teardown](src/mission_execution/test/test_command_authority.cpp#L141), lines 141-147; [suite timeout](src/mission_execution/CMakeLists.txt#L37), lines 37-40; [other unbounded result waits](src/mission_execution/test/test_command_authority.cpp#L469), lines 469-484.

**Issue:** several test calls use `future.get()` without a preceding bounded readiness check. In a retained review run, `test_command_authority` timed out after 90 seconds while executing `OrbitCancellationPreservesProgress`, following three successful cases. No per-case failure/result file was produced. An earlier isolated run completed. This establishes a nondeterministic test/liveness failure; the available trace does not establish whether the root cause is the fixture, ROS communication, or production cancellation handling. It is not evidence of a confirmed production mutex deadlock.

**Pros of changing:** failures identify the blocked operation and permit teardown/evidence collection instead of losing the entire suite to a global timeout.

**Cons of changing:** bounded test waits improve diagnosis but do not repair the underlying liveness failure.

**Proposed solution:** replace unbounded fixture/client waits with the existing deadline helper and report response, cancellation, and terminal-result stages separately. Make teardown stop the active child before joining the executor. Reproduce the timed-out case and capture worker/executor state before attributing a production cause. Preserve the failed run even if a retry passes. Ten isolated repetitions and a subsequent package rerun passed during this review, so the failed run remains unresolved.

### ISS-38 | P2 | Defect: hold-duration scoring includes a preceding false sample

**Locations:** [boolean-interval metric](src/flight_test/flight_test/runner.py#L1257), lines 1257-1292; [hold acceptance](src/flight_test/flight_test/runner.py#L1249), lines 1249-1253; [altitude settling metrics](src/flight_test/flight_test/runner.py#L1139), lines 1139-1156.

**Issue:** when a sample first satisfies the predicate and follows a contiguous sample, `_longest_boolean_interval` starts the interval at the previous sample's timestamp without requiring that previous sample to satisfy the predicate. A rising edge counts up to one allowed sample gap of time during which the condition was false. FT-06's four-second Offboard requirement and reported altitude settling durations therefore overestimate continuous qualifying time.

**Evidence:** the extracted repository function returned 0.8 seconds for samples `(0.0,false)`, `(0.4,true)`, `(0.8,true)`. The span between the qualifying samples is 0.4 seconds.

**Pros of changing:** duration assertions and diagnostic settling measurements use only the observed qualifying interval.

**Cons of changing:** borderline flights can fail after the score is corrected; sampled telemetry still limits temporal precision.

**Proposed solution:** start a new interval at the first true sample and extend it only through contiguous true samples. Keep the helper as one pure metric and add rising-edge, falling-edge, and sample-gap cases. Retain the existing acceptance duration.

### ISS-39 | P2 | Defect: a lost child result retains a shared ownership cycle

**Locations:** [child state](src/mission_execution/src/mission_execution_node.cpp#L38), lines 38-49; [goal/result callbacks](src/mission_execution/src/mission_execution_node.cpp#L117), lines 117-144; [unconfirmed-result exit](src/mission_execution/src/mission_execution_node.cpp#L217), lines 217-221; [node destructor](src/mission_execution/src/mission_execution_node.cpp#L439), lines 439-450.

**Issue:** `ChildActionState` owns `accepted_goal` through a shared pointer. The goal handle stores a result callback that captures the same child state through another shared pointer. This forms `child state -> goal handle -> result callback -> child state`. Normal result delivery clears the result callback in ROS and breaks the cycle. If the child disappears or never produces a result, the helper returns after its grace period without breaking it. The reviewed ROS client destructor invalidates pending goals but does not clear these callbacks. The state and captured parent feedback resources can therefore survive node teardown. One pending child blocks further dispatch, so this is not unbounded growth on every ordinary goal; repeated failed node lifecycles can retain additional cycles. [ROS Jazzy action callback lifecycle](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_action/include/rclcpp_action/client.hpp), [result callback release](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_action/include/rclcpp_action/client_goal_handle_impl.hpp#L65).

**Pros of changing:** releases abandoned action state and parent handles during teardown without weakening the unknown-child safety gate.

**Cons of changing:** callbacks must remain available while the node still expects a late terminal result; stopping them at the first timeout would lose reconciliation.

**Proposed solution:** use a weak accepted-goal reference for late-response tracking and explicit ownership of the one pending operation. Stop callbacks and release retained state at final node teardown or confirmed recovery. Keep dispatch blocked while the flight outcome remains unknown. Verify resource release with a weak-reference lifetime test or a targeted leak-sanitizer run.

### ISS-40 | P2 | Observed defect: planner signal shutdown can exit with an uncaught ROS context error

**Locations:** [planner entry point](src/mission_planner/mission_planner/node.py#L76), lines 76-89, especially the `rclpy.spin` call at line 82 and exception handling at lines 83-84.

**Issue:** an ordinary SIGTERM can invalidate the ROS context while the executor constructs its next wait set. In the ROS Jazzy validation environment, the resulting `rclpy._rclpy_pybind11.RCLError` escapes the entry point because it handles only `KeyboardInterrupt` and `ExternalShutdownException`. The planner exits with code 1 after its service has initialized. This obscures clean shutdown and can trigger restart-on-failure behavior in a supervisor. The planner does not command a vehicle, so this observation alone does not establish a flight-control failure.

**Evidence:** repeated torture-test controls recorded successful operation assertions followed by planner exit 1. The directly written `mission_planner/python-faults.log` traceback reports `failed to initialize wait set: the given context is not valid` from the call at line 82. The suite preserves both the operation result and the teardown error.

**Pros of changing:** ordinary signal shutdown reports a clean exit and preserves unexpected executor faults as actionable errors.

**Cons of changing:** a broad exception handler can hide a real executor defect; context validity and signal handling must distinguish expected shutdown from faults during operation.

**Proposed solution:** align executor and context shutdown using the supported Jazzy lifecycle APIs. If an executor exception is handled, suppress it only after confirmed shutdown and re-raise it while the context remains valid. Add repeated SIGTERM tests after service readiness and retain the unexpected-error path. PX4 has no role in this planning-process lifecycle.

## PX4 replacement decisions

These options overlap. Selecting all of them would add backends and configuration. Choose one execution approach for each actual requirement.

| Current responsibility | PX4 capability | Case for changing | Case for retaining | Recommendation |
| --- | --- | --- | --- | --- |
| Position steps, custom yaw rate limiting, ordinary search/altitude schedules | Multicopter go-to position/heading smoothing | Deletes application smoothing assumptions; uses FMU motion constraints | Exact externally timed trajectories require different semantics | Evaluate first for ordinary navigation, ISS-07/16. [Go-to types](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/control/setpoint_types/multicopter/goto.hpp). |
| Offboard/arm/ACK/retry lifecycle | Registered modes and mode executor | Removes protocol polling and provides ownership/completion hooks | New pinned library; compatibility/migration tests required | Replace the backend only if code removal is demonstrated, ISS-28. |
| Touchdown, explicit disarm, mandatory Auto Loiter | Native Land and auto-disarm | Removes post-touchdown command failure paths | Nondefault auto-disarm policy may require operator disarm | Delegate touchdown/disarming to PX4, ISS-13. |
| Fixed GPS/search route sequencing | Onboard Mission mode | Autopilot progression and standard GCS visibility | Requires global coordinates and mission transport; behavior differs | Prefer for a product limited to static geographic routes, ISS-27. |
| Companion JSON mission framework | PX4 ROS 2 waypoint missions | Library mission sequencing with custom ROS actions | Experimental, multicopter-only, companion-executed, incompatible with standard MAVLink missions/GCS display | Do not migrate merely because both use JSON. This option does not move execution onboard. [ROS 2 waypoint missions](https://docs.px4.io/v1.17/en/ros2/px4_ros2_waypoint_missions). |
| Ad hoc application distance checks and unattended hold | PX4 geofence and configured loss/failsafe actions | Independent flight-controller containment/recovery | Application input validation and mission limits remain necessary | Use PX4 for enforcement and one application validator for intent, ISS-02/08. |
| Orbit circle control | Existing native `DO_ORBIT` integration | Already delegates approach and circle tracking | Revolution counting, application completion, and command cancellation remain application requirements | Keep native Orbit; fix ownership/handoff. PX4's Orbit task does not expose this action's finite-revolution completion contract. [v1.17 Orbit implementation](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/flight_mode_manager/tasks/Orbit/FlightTaskOrbit.cpp). |
| Fleet assignment, lawnmower geometry, mission-start offsets, application ENU | Application planner and adapter | These are application-specific contracts | PX4 does not supply this fleet schema or search allocation; replacing the few frame operations with a large dependency adds burden | Retain pure planning and the ENU/NED boundary. Prefer global PX4 targets if adopting onboard missions; otherwise retain projection tests against the pinned PX4 convention. |

## Suggested change order

1. Fix unsolicited publication, parameter truthfulness, mode/frame ownership, terminal landing priority, and unsafe numeric conversions: ISS-01 through ISS-12.
2. Reestablish a verified cancellation/hold and landing contract: ISS-07, ISS-13, ISS-19, ISS-22, ISS-25, ISS-33, and ISS-37 through ISS-39.
3. Select an execution backend using a measured removal prototype: ISS-16, ISS-26, ISS-27, and ISS-28. Do not retain competing implementations of the same route behavior.
4. Remove unused schema/configuration paths and consolidate environment/documentation: ISS-29 through ISS-36.

## Review validation and limits

The temporary source copy used cached image `sar-drone:px4-v1.17.0`, ID `sha256:6244ed6774f545a48fdfc76fc0f3436bbc7e5eefd032fdd551e68b835a175115`. Its PX4 and `px4_msgs` checkouts both identify as v1.17.0; it contains Python 3.12.3 and ROS 2 Jazzy. This was an offline review environment, not a rebuild of the repository's current Dockerfile. The checkout was mounted read-only and build products were written outside the repository.

| Check | Result and limit |
| --- | --- |
| Current source build | All seven packages built. The six non-bringup packages used the current CMake warning profile and test builds. `drone_bringup` built with `BUILD_TESTING=OFF` because the cached image lacks Ruff. |
| Retained package test run | Orbit cancellation test timed out at 90 seconds. `colcon test-result --verbose` reported 101 test results, one error, one failure. The missing gtest result contributes to that report; these are not two independently diagnosed defects. |
| Targeted investigation | Ten repetitions of `CommandAuthority.OrbitCancellationPreservesProgress` passed. The mission-execution package rerun passed; the combined retained result then reported 115 test results, zero errors/failures/skips. This does not erase the timeout. |
| Standalone bringup checks | Real launch endpoint test: one test passed. Planner/executor integration: nine tests passed. These ran directly because bringup CTest configuration requires Ruff. |
| Host planner subset | Ten unittest geometry/legacy-parser cases passed. Host pytest was unavailable; the broader Python suite ran in the container. |
| Synthetic PX4 node probes | Reproduced unsolicited zero targets with commands disabled, heartbeat after telemetry expiry, accepted takeoff after a successful runtime disable update, and navigation success after Offboard exit. ROS domain 174; synthetic prefixes only. Package tests used domain 173, with the adapter test's own domain 77. |
| Numeric probe | GCC float-cast-overflow sanitizer reproduced the unsafe duration conversion in ISS-11. This was a standalone extraction of the conversion, not a sanitizer build of the complete application. |
| Flight metric probe | Extracted repository metric reproduced the false-sample duration in ISS-38. |
| Dependency ownership inspection | Checked callback release and client destruction in the exact cached ROS headers for ISS-39. This is source-level lifetime analysis; no complete-application leak-sanitizer run was performed. |
| Documentation checks | Verified issue field completeness and local source-link targets/line anchors. Found 24 missing historical evidence targets. Checked report whitespace. |
| Not run during the initial review | Current Dockerfile image rebuild, Ruff launch lint/format, Gazebo flights, hardware flights, multi-vehicle PX4 flights, full-application address/thread sanitizers, and native clean-Pi build. The later [torture-suite validation](docs/testing/TORTURE-TESTS.md#recorded-validation) records subsequent launch checks and targeted application sanitizer runs. |

Retained local diagnostic files are under `/tmp/drone-review-workspace/`: `build.log`, `bringup-build.log`, `probe.py`, `probe.log`, `tests.log`, `results.log`, `orbit-repeat.log`, `mission-retest.log`, `retest-results.log`, `endpoints.log`, and `planner-flow.log`. These temporary files are not durable release artifacts. No source fix, commit, remote push, or pull request is part of this review.

Assumptions: a single application authority controls each vehicle; operator/PX4 takeover must remain authoritative; mission-start ENU semantics remain required unless an execution-backend decision explicitly replaces them. Requirements not established by this review: airframe response, hardware readiness, wind-flight acceptance, multi-vehicle separation, and a passing full flight suite. The recommendations preserve the repository's PX4/application boundary and favor deletion of duplicate execution paths over new frameworks.
