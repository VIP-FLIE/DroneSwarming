# Geographic missions and metric commands

The supported platform uses GPS/GNSS and PX4's fused position estimate. Vision, visual-inertial odometry, motion capture, obstacle detection, and inter-vehicle collision avoidance are not application capabilities.

## Coordinate contracts

| Representation | Units and axes | Owner and purpose |
| --- | --- | --- |
| WGS84/AMSL | Latitude and longitude in degrees; altitude in meters above mean sea level | Authoritative compiled mission destinations and GPS navigation goals |
| `relative_enu` | East, north, up in meters from one captured command-start position | Relative mission and navigation input |
| `relative_body` | Forward, left, up in meters at one captured command-start heading | Relative mission and navigation input |
| `shared_enu` | East, north, up in meters from an explicit geographic `origin` | JSON site geometry shared by several vehicles |
| `local_enu` | East, north, up in meters from PX4's estimator origin | Local telemetry, legacy navigation, trajectories, and orbit execution |
| PX4 NED | North, east, down in meters | Flight-controller local setpoints, confined to the PX4 adapter |

`GeographicPosition` carries `latitude_deg`, `longitude_deg`, and `altitude_amsl_m` as float64 values. AMSL height, height above home, height above terrain, and WGS84 ellipsoid height are different quantities. The application accepts AMSL geographic goals. MAVLink relative-altitude frames still use geographic latitude and longitude horizontally.

Relative commands capture position once. Body-relative commands also capture yaw once. The frame is level with gravity; roll and pitch do not tilt the commanded displacement. A forward command does not rotate its destination as the vehicle turns. For ROS yaw zero, forward points east and left points north. At yaw pi/2, forward points north and left points west. PX4 heading is converted to ROS yaw at the adapter boundary. Heading must be valid for `relative_body`. JSON body-relative missions require valid heading while landed because compilation precedes takeoff. Standalone body-relative navigation can start after takeoff when heading becomes valid.

Every relative mission point uses the same captured start. Points `[3, 0]` and `[3, 3]` describe two destinations from that start, rather than successive increments. `relative_enu` with an `origin` remains a compatibility spelling for `shared_enu`. An empty `ExecuteWaypointMission.frame_id` remains a compatibility spelling for `relative_enu`.

## Compilation and execution

The planner expands circles and search stripes in a fixed metric planning frame. A `shared_enu` route uses its explicit geographic origin. A WGS84 search area uses a deterministic anchor derived from the area. These anchors are independent of each vehicle's estimator origin. Direct WGS84 waypoint coordinates are preserved exactly by the compiler.

The planner returns geographic waypoint and orbit destinations. Mission execution validates meter-based altitude and distance bounds around the captured launch position and forwards geographic destinations to `FlyMission`. Projected-coordinate envelope checks allow 1 millimeter of numerical rounding tolerance. Authored relative metric distances are checked before projection. Vehicle control encodes those destinations into MAVLink mission items without projecting them through the estimator's local frame. MAVLink encodes horizontal coordinates at 1e-7 degree resolution and altitude as float32.

Takeoff remains a vertical climb in meters from the captured start. Vehicle control checks the captured geographic start against current telemetry before upload and before arming. Movement beyond 1 meter invalidates the start conditions. A local-frame revision change alone does not invalidate a geographic mission. Uploaded destinations and geographic distance feedback remain anchored to their physical locations after a local-frame reset.

An optional finite-orbit tail retains its geographic center until the handoff. The executor projects it into the current local frame immediately before dispatching `OrbitPoint`. The local orbit rejects a subsequent reference change. Standalone geographic navigation projects its target at the vehicle-control boundary and aborts into a current-position hold if the estimator frame resets during execution. Submit a new navigation request after telemetry is usable.

Position and status freshness use `telemetry_timeout_sec`, default 1 second. Land-state freshness allows at least 2 seconds because PX4 republishes unchanged land detection once per second. The additional land-state window does not extend position or status freshness.

`VehicleState.global_position_valid` gates the current `geographic_position`. `global_reference_valid` separately gates the estimator-origin reference fields. Local positions from two vehicles are not directly comparable. Use a shared geographic anchor before calculating inter-vehicle distances or shared geometry. A ROS namespace does not establish a coordinate transform.

## Navigation commands

Build and source the entire workspace after changing ROS interfaces. Launch the controlled simulation with flight commands enabled using the root guide. Standalone navigation requires an armed, airborne vehicle in Offboard or native Hold. Takeoff and navigation are separate commands:

```bash
ros2 action send_goal /mission_execution/takeoff swarm_interfaces/action/Takeoff \
  '{altitude_m: 3.0}' --feedback

ros2 action send_goal /mission_execution/navigate_to swarm_interfaces/action/NavigateTo \
  '{frame_id: relative_body, target_position: {x: 3.0, y: 0.0, z: 0.0}, position_tolerance_m: 0.5, timeout_sec: 30.0}' --feedback

ros2 action send_goal /mission_execution/navigate_to swarm_interfaces/action/NavigateTo \
  '{frame_id: relative_enu, target_position: {x: 0.0, y: 3.0, z: 0.0}, timeout_sec: 30.0}' --feedback
```

The second command moves 3 meters forward at the captured heading. The third moves 3 meters north from its own captured start. `z: 0.0` preserves current altitude for relative navigation.

Read the current geographic estimate:

```bash
ros2 topic echo /vehicle_state --once
```

Replace the following example latitude, longitude, and AMSL altitude with a reachable destination at the actual site. Geographic navigation is bounded to 1,000 horizontal meters and a 120-meter height change from the vehicle's current position:

```bash
ros2 action send_goal /mission_execution/navigate_to swarm_interfaces/action/NavigateTo \
  '{frame_id: wgs84, geographic_target: {latitude_deg: 47.3979711, longitude_deg: 8.5461637, altitude_amsl_m: 3.0}, position_tolerance_m: 0.5, timeout_sec: 30.0}' --feedback

ros2 action send_goal /mission_execution/land swarm_interfaces/action/Land \
  '{timeout_sec: 60.0}' --feedback
```

`NavigateTo.look_at_target` selects optional yaw control. WGS84 requests use `look_at_geographic_position`. Other public frames use `look_at_position` in the same frame as the target. `local_enu` remains available for existing tests and callers. `FollowTrajectory` and standalone `OrbitPoint` retain their local ENU contracts.

JSON examples include [relative ENU](../../src/mission_planner/examples/relative-fleet.json), [forward/left](../../src/mission_planner/examples/relative-body.json), [GPS](../../src/mission_planner/examples/gps-fleet.json), and [shared site geometry](../../src/mission_planner/examples/shared-origin.json). Geographic examples require site-specific positions and AMSL heights.

## Navigation cancellation

Navigation cancellation confirms position within the configured default tolerance and horizontal and vertical speeds at most 0.3 m/s for 0.5 seconds. Braking can move the vehicle beyond the position where cancellation was requested. The action reports canceled after hold confirmation, with a 12-second confirmation deadline. It reports aborted if hold is unconfirmed, telemetry is invalid, the local frame changes, or PX4 selects another mode. The current Offboard stream continues while the action confirms hold; the application does not override an operator-selected mode during cancellation.

## Localization and sensing limits

GPS coordinate representation does not provide position accuracy. Relative meter commands inherit the fused estimator's position error, and body-relative commands also inherit heading error. Acceptance radii are controller tolerances, not guaranteed physical accuracy. No route in this application detects or avoids an obstacle or another drone. Fleet assignments provide no automatic separation or synchronized execution.

All uploaded missions and all new relative navigation commands require valid geographic localization. The system does not invent geographic origins or bypass PX4 preflight checks when localization is unavailable. Flight commands remain disabled by default. The operator recovery path and PX4 failsafes retain control.

## Documentation and research basis

- [ROS REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst) defines SI units, ENU, body forward/left/up, and counterclockwise yaw from east. These conventions define the application's metric and relative frames.
- [ROS REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst) distinguishes robot-local, map, and Earth frames and documents transforms between robots. The shared site anchor and separate estimator-origin diagnostics follow these frame semantics. The application does not implement a full `map`/`odom` TF localization tree.
- [PX4 v1.17 Mission mode](https://docs.px4.io/v1.17/en/flight_modes_mc/mission) requires a global 3D position estimate. The [MAVLink mission protocol](https://mavlink.io/en/services/mission.html) defines geographic mission items. These requirements determine the compiled route format.
- [EGO-Swarm, Zhou et al., ICRA 2021](https://arxiv.org/html/2011.04183v3) uses metric trajectories and compensates inter-agent localization drift with visual observations. This project uses metric planning geometry and explicit shared anchors. Its visual drift correction and collision-avoidance algorithms are not implemented.
- [Omni-swarm, Xu et al., IEEE T-RO 2022](https://arxiv.org/html/2103.04131v5) estimates consistent swarm states using visual-inertial, visual, and UWB measurements. This project separates shared mission destinations from per-vehicle estimator frames. It does not implement those sensing or estimation algorithms.
