# Swarm stages

The current vertical slice supports one PX4 vehicle. `swarm_coordination` is an empty buildable scaffold.

## Initial vehicle

Use `drone_bringup` to start `vehicle_control`, `mission_execution`, and `mission_planner`. Flight commands default to disabled. See the root [simulation flight procedure](../../README.md#simulation-flight-procedure) for PX4 SITL commands and the example square mission.

## Fleet plan input

Version 2 [JSON plans](../../src/mission_planner/README.md) contain explicit per-vehicle assignments. Submit the same document to each executor namespace. Relative paths use each vehicle's captured start; shared-origin and GPS paths use geographic anchors and AMSL heights. The planner performs no discovery, automatic allocation, synchronized start, or collision avoidance. Two-namespace ROS integration tests exercise assignment selection and coordinate translation with simulated action backends. Multi-vehicle PX4 flight validation remains required.

## Planned development order

1. Verify the single-vehicle mission in PX4 SITL.
2. Validate one physical vehicle with a controlled test plan and operator recovery controls.
3. Add PX4 DDS namespacing and bringup for multiple vehicles.
4. Add peer status and health monitoring.
5. Add search-area assignment and unfinished-task reassignment.

PX4 ROS namespaces and application ROS namespaces are configured independently. The default PX4 topic prefix is `/fmu`; namespaced PX4 instances use a prefix such as `/uav_1/fmu` when the DDS client publishes there.
