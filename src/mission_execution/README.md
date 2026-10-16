# mission_execution

This package sequences the application actions `Takeoff`, `NavigateTo`, and `Land`. It depends only on `swarm_interfaces` and `vehicle_control` action types; PX4 messages and topics remain inside `vehicle_control`.

Waypoint offsets are all measured from the single local_enu position captured at mission start. For an origin `(x0, y0, z0)`, `(3, 0, 2)` targets `(x0 + 3, y0, z0 + 2)`, even if earlier waypoints moved the vehicle.

The action server runs mission work on a worker thread and uses asynchronous action clients. A mission cancellation requests cancellation of the active child and waits for its terminal result before ending the mission.
