# mission_execution

This package sequences the application actions `Takeoff`, `NavigateTo`, `OrbitPoint`, and `Land`. It depends only on `swarm_interfaces` action types; PX4 messages and topics remain inside `vehicle_control`.

Waypoint offsets are all measured from the single local_enu position captured at mission start. For an origin `(x0, y0, z0)`, `(3, 0, 2)` targets `(x0 + 3, y0, z0 + 2)`, even if earlier waypoints moved the vehicle.

When `look_at_point_of_interest` is true, every waypoint navigation goal faces `look_at_offset`, also measured from the mission-start position. The mission executor passes the absolute point to `NavigateTo`; yaw follows its horizontal bearing from the vehicle's current position. Waypoint position targets remain independent of yaw.

When `orbit_after_waypoints` is true, the mission runs `OrbitPoint` after the waypoint list and before optional landing. `orbit_center_offset` uses the same mission-start origin as waypoint offsets. `orbit_completed` is true only after the orbit action completes its requested revolutions and confirms Offboard hold. A failed or canceled orbit ends the mission without starting the landing action.

The action server runs mission work on a worker thread and uses asynchronous action clients. A mission cancellation requests cancellation of the active child and waits for its terminal result before ending the mission.

Mission lifecycle events use ROS logging. The action feedback stream reports waypoint progress and OrbitPoint progress without logging each control setpoint.
