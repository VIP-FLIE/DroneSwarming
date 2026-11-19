# mission_execution

`mission_execution` is the sole application command authority for `vehicle_control`. Mission callers and the flight-test runner submit flight intent here. The node selects operations using fresh `VehicleState` and owns all application clients for vehicle-control actions. PX4 messages, DDS topics, and frame conversion remain inside `vehicle_control`.

| Public endpoint | Action type | Behavior |
| --- | --- | --- |
| `mission_execution/execute_flight_plan` | `ExecuteFlightPlan` | Decode JSON through the planner, then execute the selected vehicle assignment and land |
| `execute_waypoint_mission` | `ExecuteWaypointMission` | Uploaded PX4 takeoff/waypoint/landing route, with an optional finite-orbit tail |
| `mission_execution/takeoff` | `Takeoff` | Select a vertical climb from a landed, disarmed state |
| `mission_execution/navigate_to` | `NavigateTo` | Select GPS/AMSL, relative ENU/body, or legacy local ENU navigation from airborne Offboard or native Hold |
| `mission_execution/follow_trajectory` | `FollowTrajectory` | Select a timed position trajectory with optional yaw |
| `mission_execution/orbit_point` | `OrbitPoint` | Select a finite orbit with optional center-facing yaw |
| `mission_execution/land` | `Land` | Interrupt ordinary flight work and request controlled landing |

All names are relative to the application namespace. The five operation types are reused to preserve flight-test goal fields, feedback, measured result fields, and cancellation. Public operation endpoints share admission and state checks with waypoint missions. Internal child endpoints use `vehicle_control/<operation>` and are called only by this package.

Request explicit landing through the public endpoint:

```bash
ros2 action send_goal /mission_execution/land swarm_interfaces/action/Land \
  "{timeout_sec: 60.0}" --feedback
```

Only one ordinary command is admitted at a time. Conflicting ordinary requests and duplicate landing requests are rejected, with a ROS rejection diagnostic. Invalid landing goals cannot interrupt a route. An accepted landing interrupts the active ordinary request, requests child cancellation, and waits for its terminal result before dispatching `Land`. The interrupted request returns aborted unless its caller also canceled it. A landing is committed when accepted by mission execution; its public cancel requests are rejected.

Ordinary cancellation propagates to the active child. Navigation cancellation leaves vehicle control streaming a hold and reports canceled only after position and velocity settle. An unconfirmed hold returns aborted. Orbit cancellation waits for vehicle control to restore and confirm Offboard hold. A confirmed canceled child ends its canceled parent with `success=false`. A child failure during cancellation returns aborted and retains its failure diagnostic, including an orbit hold-recovery failure. Child rejection, failure, missing result, unavailable server, and invalid or stale state return unsuccessful results with diagnostic text. Measured child result fields and operation feedback are retained.

The default server/goal-response deadline is 10 seconds. Cancellation drains for at most 15 seconds; a committed child landing can drain until its result deadline because PX4 may reject its cancellation. A late goal acceptance is canceled. If a child terminal result is unconfirmed, the parent aborts and further child dispatch remains blocked until rejection or a terminal result arrives. A queued explicit landing waits at most the server deadline for that outstanding child, then returns failure. It does not wait for the remaining route. PX4 and the operator recovery path retain responsibility when the execution boundary is unreachable.

State checks require a recently received, valid, finite `local_enu` position. Heading requests require valid finite yaw. Standalone airborne operations require armed, airborne Offboard or native Hold state. Vehicle control establishes Offboard before executing those operations. State is checked again immediately before each child dispatch. `VehicleState` supplies application telemetry, not ground truth; vehicle control independently checks PX4 telemetry freshness, execution limits, preflight checks, and failsafes. Flight commands remain disabled by default.

Relative waypoint offsets use one captured geographic start. ENU offsets use east/north/up; body offsets use forward/left/up at one captured valid heading. Relative input resolves once into geographic destinations. `frame_id: wgs84` uses `waypoint_positions` directly and requires the offset list to be empty. An empty frame preserves relative ENU input compatibility.

When `look_at_point_of_interest` is true, the uploaded route includes a fixed geographic ROI location, resolved from `look_at_offset` for relative input or supplied as `look_at_geographic_position` for WGS84 input. ROI is cleared after the waypoints. This uses PX4 mission heading behavior and does not apply standalone NavigateTo's measured yaw-settle contract.

When `orbit_after_waypoints` is true, the mission runs `OrbitPoint` after the waypoint list and before optional landing. The orbit center resolves to a geographic position using the same captured start as the waypoints. WGS84 requests supply `orbit_center_geographic`. At orbit handoff, the executor projects that center into the current estimator-local frame. `orbit_completed` is true only after the orbit action completes its requested revolutions and confirms Offboard hold. A failed or canceled orbit ends the mission without starting the landing action.

The action server runs mission work on a worker thread and uses asynchronous action clients. A mission cancellation requests cancellation of the active child and waits for its terminal result before ending the mission.

Mission lifecycle events use ROS logging. The action feedback stream reports waypoint progress and OrbitPoint progress without logging each control setpoint.

JSON plans enter this node as content in `ExecuteFlightPlan.plan_json`. The executor captures a single valid start state, sends the content and `vehicle_id` to `mission_planner/plan_flight`, validates the translated response, and dispatches one fixed geographic route. JSON is limited to 65,536 bytes. The planner service deadline uses `action_server_timeout_sec`. Cancellation and explicit landing also interrupt planning; late responses cannot execute.

`vehicle_id` defaults to the application namespace with slashes replaced by underscores, or `uav_1` at root. `maximum_plan_altitude_m` and `maximum_plan_distance_m` default to 30 and 100 meters. Geographic destinations remain fixed across local-frame changes. Vehicle control checks geographic start movement before arming. PX4 executes stored geographic destinations. An optional orbit tail captures the current local reference at handoff. JSON missions start landed and disarmed and land after a successful path. See the [schema](../mission_planner/README.md) and [handoff](../../docs/architecture/JSON-FLIGHT-PLANS.md).

Source-aware swarm/operator arbitration and perception-driven flight are unimplemented. The current executor admits one ordinary command and gives an accepted explicit landing priority over that command.

## Onboard waypoint execution

Waypoint and JSON missions dispatch one internal `vehicle_control/fly_mission` action for native takeoff, waypoints, and optional landing. The route requires a valid global reference, a MAVLink connection, and a landed/disarmed start. Native upload supports at most 256 waypoints, 120 meters vertically and 1,000 meters horizontally at vehicle control; JSON defaults remain 30 and 100 meters.

Cancellation ends the ROS mission after vehicle control confirms native Hold, or disarming if still landed. It does not preserve a resumable ROS action. Explicit landing waits for this child result before dispatching Land. A failed hold confirmation aborts the parent. PX4 failsafes and externally selected flight modes retain control.

A finite orbit remains a separate child after a takeoff/waypoint-only upload. Vehicle control transitions native Hold to Offboard before orbit, then retains the established finite-revolution completion and optional landing behavior.

Waypoint acceptance uses PX4 mission acceptance and a 0.5-second dwell. Mission look-at requests become native ROI items and clear ROI after the route. They no longer promise NavigateTo's measured 0.2-radian heading tolerance. Standalone NavigateTo retains its existing heading contract.

Pause/resume, airborne route replacement, and perception-driven rejoin selection are not exposed by the current ROS API. The native mission remains stored in PX4 after cancellation. Reentering Mission through an external client can resume it. See the [onboard mission guide](../../docs/architecture/PX4-ONBOARD-MISSIONS.md).

Native waypoint deadlines are positive and at most 600 seconds; zero on the public mission action selects the configured default. Companion timeout parameters and standalone operation timeouts have a 3,600-second upper bound.

The [coordinate guide](../../docs/architecture/COORDINATES.md) provides forward, north, GPS, and landing commands. The drones use GPS/GNSS without vision or obstacle detection.
