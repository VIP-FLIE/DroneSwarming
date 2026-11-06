# swarm_interfaces

Application contracts use `local_enu`: X is east, Y is north, and Z is up, in meters from PX4's local-position origin. PX4-specific message types belong in `vehicle_control` only.

`OrbitPoint` accepts an absolute `local_enu` center and requests a finite, signed orbit from `vehicle_control`. The center Z value is orbit altitude. The result reports measured completed revolutions and succeeds only after Offboard hold is confirmed. `ExecuteWaypointMission` can optionally run the same action after its waypoints; its orbit-center offset uses the mission-start origin.

`NavigateTo` accepts an optional absolute `local_enu` look-at point. The vehicle yaw tracks the horizontal bearing to that point while position control continues to use the independent ENU position target. `ExecuteWaypointMission` can apply one look-at offset, relative to the mission-start position, to every waypoint.

`SearchRegion`, `TaskAssignment`, and `VictimReport` are provisional initial contracts. Refine them when coordination and perception are implemented.
