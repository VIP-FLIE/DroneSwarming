# swarm_interfaces

Compiled mission destinations use `GeographicPosition`: float64 WGS84 latitude/longitude in degrees and altitude in meters above mean sea level. Shared planning geometry uses ENU meters from a documented geographic anchor. PX4 message types remain confined to `vehicle_control`.

Public endpoints belong to `mission_execution`. `vehicle_control/<operation>` endpoints are internal execution contracts. Build and source the entire workspace after changing these messages, actions, or services.

| Contract | Coordinate semantics |
| --- | --- |
| `VehicleState` | Local ENU telemetry plus a validity-gated current `geographic_position` and estimator-origin diagnostics |
| `NavigateTo` | Public `relative_enu`, `relative_body`, `wgs84`, or legacy `local_enu`; relative input resolves once to WGS84 before child dispatch |
| `ExecuteWaypointMission` | Authored relative ENU/body offsets or WGS84 destinations, selected by `frame_id` |
| `PlanFlight` | JSON, vehicle ID, and a captured state produce fixed geographic waypoints, takeoff climb, and an optional geographic orbit center |
| `FlyMission` | Captured geographic starting position, geographic takeoff/waypoint/ROI destinations, and execution settings |
| `FollowTrajectory` | Timed estimator-local ENU positions and optional ROS yaw |
| `OrbitPoint` | Estimator-local ENU center and finite orbit settings |

`NavigateTo.geographic_target` and `look_at_geographic_position` apply to `wgs84`. `target_position` and `look_at_position` apply to the other public frames. Relative body axes are forward, left, and up at one captured heading. Relative ENU axes remain east, north, and up.

`ExecuteWaypointMission.waypoint_positions`, `look_at_geographic_position`, and `orbit_center_geographic` apply to `wgs84`. Its offset fields apply to `relative_enu` or `relative_body`. Geographic and relative waypoint lists cannot be combined. An empty frame retains the previous relative ENU input convention. Every relative mission waypoint uses one captured start. Takeoff altitude is a vertical climb from that start for every input frame.

`VehicleState.global_position_valid` gates the current geographic estimate. `global_reference_valid` gates the reference latitude, longitude, and AMSL altitude of the estimator-local origin. `local_frame_revision` identifies local reference or position-reset changes. Geographic missions remain fixed across revisions. Local operations retain frame checks. Geographic orbit centers are projected only at the orbit handoff.

`VehicleState.holding` and `VehicleState.mission` expose fresh native PX4 mode status. Uploaded mission cancellation confirms native Hold or landed/disarmed state. Navigation confirms a settled Offboard hold; orbit confirms Offboard recovery. The action result and state topic arrive independently.

`SearchRegion`, `TaskAssignment`, and `VictimReport` are provisional definitions without coordination or perception runtime. The [coordinate guide](../../docs/architecture/COORDINATES.md) provides commands and sensing limits; the [planner guide](../mission_planner/README.md) defines JSON inputs; the [onboard mission guide](../../docs/architecture/PX4-ONBOARD-MISSIONS.md) defines upload and cancellation.
