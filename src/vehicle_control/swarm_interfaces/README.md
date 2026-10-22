# swarm_interfaces

Application contracts use `local_enu`: X is east, Y is north, and Z is up, in meters from PX4's local-position origin. PX4-specific message types belong in `vehicle_control` only.

`SearchRegion`, `TaskAssignment`, and `VictimReport` are provisional initial contracts. Refine them when coordination and perception are implemented.
