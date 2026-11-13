# swarm_interfaces

Application contracts use `local_enu`: X is east, Y is north, and Z is up, in meters from PX4's local-position origin. PX4-specific message types belong in `vehicle_control` only.

The public command endpoints belong to `mission_execution`; `vehicle_control/<operation>` endpoints are internal execution contracts. Both use the existing `Takeoff`, `NavigateTo`, `FollowTrajectory`, `OrbitPoint`, and `Land` types. `ExecuteWaypointMission` retains its public endpoint and origin semantics.

`OrbitPoint` accepts an absolute `local_enu` center and requests a finite, signed orbit from `vehicle_control`. The center Z value is orbit altitude. The result reports measured completed revolutions and succeeds only after Offboard hold is confirmed. `ExecuteWaypointMission` can optionally run the same action after its waypoints; its orbit-center offset uses the mission-start origin.

`NavigateTo` accepts an optional absolute `local_enu` look-at point. The vehicle yaw tracks the horizontal bearing to that point while position control continues to use the independent ENU position target. `ExecuteWaypointMission` can apply one look-at offset, relative to the mission-start position, to every waypoint.

`SearchRegion`, `TaskAssignment`, and `VictimReport` are provisional initial contracts. Refine them when coordination and perception are implemented.

`ExecuteFlightPlan` carries bounded JSON content to mission execution. `PlanFlight` is a planning-only service: JSON, vehicle ID, and one captured `VehicleState` produce mission-start waypoint offsets, takeoff climb, and optional orbit instructions. Its response contains application coordinates and no flight side effects. JSON execution always lands after successful path completion. See the [schema](../mission_planner/README.md).

`VehicleState.local_frame_revision` identifies changes to the local geographic reference or estimator position-reset counters. `global_reference_valid` gates `reference_latitude_deg`, `reference_longitude_deg`, and `reference_altitude_amsl_m`. These fields describe the geographic location of the local ENU origin. AMSL height is meters above mean sea level. Geographic planning requires valid reference fields and a nonzero revision. Executors abort translated missions if the revision changes.
