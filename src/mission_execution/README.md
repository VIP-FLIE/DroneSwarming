# mission_execution

`mission_execution` is the sole application command authority for `vehicle_control`. Mission callers and the flight-test runner submit flight intent here. The node selects operations using fresh `VehicleState` and owns all application clients for vehicle-control actions. PX4 messages, DDS topics, and frame conversion remain inside `vehicle_control`.

| Public endpoint | Action type | Behavior |
| --- | --- | --- |
| `mission_execution/execute_flight_plan` | `ExecuteFlightPlan` | Decode JSON through the planner, then execute the selected vehicle assignment and land |
| `execute_waypoint_mission` | `ExecuteWaypointMission` | Takeoff, mission-origin waypoints, optional orbit, optional landing |
| `mission_execution/takeoff` | `Takeoff` | Select a vertical climb from a landed, disarmed state |
| `mission_execution/navigate_to` | `NavigateTo` | Select absolute `local_enu` navigation from airborne Offboard hold |
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

Ordinary cancellation propagates to the active child. Navigation cancellation leaves vehicle control streaming a hold at the latest valid position. Orbit cancellation waits for vehicle control to restore and confirm Offboard hold. A confirmed canceled child ends its canceled parent with `success=false`. A child failure during cancellation returns aborted and retains its failure diagnostic, including an orbit hold-recovery failure. Child rejection, failure, missing result, unavailable server, and invalid or stale state return unsuccessful results with diagnostic text. Measured child result fields and operation feedback are retained.

The default server/goal-response deadline is 10 seconds. Cancellation drains for at most 15 seconds; a committed child landing can drain until its result deadline because PX4 may reject its cancellation. A late goal acceptance is canceled. If a child terminal result is unconfirmed, the parent aborts and further child dispatch remains blocked until rejection or a terminal result arrives. A queued explicit landing waits at most the server deadline for that outstanding child, then returns failure. It does not wait for the remaining route. PX4 and the operator recovery path retain responsibility when the execution boundary is unreachable.

State checks require a recently received, valid, finite `local_enu` position. Heading requests require valid finite yaw. Airborne operations require armed, airborne Offboard state. State is checked again immediately before each child dispatch. `VehicleState` supplies application telemetry, not ground truth; vehicle control independently checks PX4 telemetry freshness, execution limits, preflight checks, and failsafes. Flight commands remain disabled by default.

Waypoint offsets are all measured from the single local_enu position captured at mission start. For an origin `(x0, y0, z0)`, `(3, 0, 2)` targets `(x0 + 3, y0, z0 + 2)`, even if earlier waypoints moved the vehicle.

When `look_at_point_of_interest` is true, every waypoint navigation goal faces `look_at_offset`, also measured from the mission-start position. The mission executor passes the absolute point to `NavigateTo`; yaw follows its horizontal bearing from the vehicle's current position. Waypoint position targets remain independent of yaw.

When `orbit_after_waypoints` is true, the mission runs `OrbitPoint` after the waypoint list and before optional landing. `orbit_center_offset` uses the same mission-start origin as waypoint offsets. `orbit_completed` is true only after the orbit action completes its requested revolutions and confirms Offboard hold. A failed or canceled orbit ends the mission without starting the landing action.

The action server runs mission work on a worker thread and uses asynchronous action clients. A mission cancellation requests cancellation of the active child and waits for its terminal result before ending the mission.

Mission lifecycle events use ROS logging. The action feedback stream reports waypoint progress and OrbitPoint progress without logging each control setpoint.

JSON plans enter this node as content in `ExecuteFlightPlan.plan_json`. The executor captures a single valid start state, sends the content and `vehicle_id` to `mission_planner/plan_flight`, validates the translated response, and uses the existing mission sequencer with that captured origin. JSON is limited to 65,536 bytes. The planner service deadline uses `action_server_timeout_sec`. Cancellation and explicit landing also interrupt planning; late responses cannot execute.

`vehicle_id` defaults to the application namespace with slashes replaced by underscores, or `uav_1` at root. `maximum_plan_altitude_m` and `maximum_plan_distance_m` default to 30 and 100 meters. Translation is invalidated if the local-frame revision changes or position becomes unavailable. JSON missions start landed and disarmed and land after a successful path. See the [schema](../mission_planner/README.md) and [handoff](../../docs/architecture/JSON-FLIGHT-PLANS.md).

Swarm or operator special instructions and perception results that affect flight will enter through this node. Their runtime implementations and arbitration policy remain deferred. The planned priority order and unresolved policies are in the [architecture guide](../../docs/architecture/README.md).
