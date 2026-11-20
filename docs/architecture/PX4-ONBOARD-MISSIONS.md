# PX4 onboard waypoint missions

`execute_waypoint_mission` and `mission_execution/execute_flight_plan` execute waypoint routes as uploaded PX4 missions. Firmware and messages remain PX4 v1.17.0. MAVSDK 4.2.0 supplies the MAVLink mission protocol client.

## Execution changes

| Responsibility | Previous waypoint backend | Current waypoint backend |
| --- | --- | --- |
| Takeoff and waypoint dispatch | Takeoff child, then one NavigateTo child per waypoint | One internal FlyMission child |
| Route coordinates | Local ENU targets streamed as PX4 NED | Geographic latitude/longitude and AMSL mission items |
| Waypoint progression | ROS loop on the companion computer | PX4 mission executor |
| Ordinary route landing | Separate Land child | Uploaded Land item |
| Route motion setpoints | Application Offboard stream | PX4 generates motion |
| Cancellation | Streamed Offboard hold | Confirmed native Hold, or disarming if still landed |
| Optional finite circle | OrbitPoint child and optional Land | Retained OrbitPoint child and optional Land after uploaded takeoff/waypoints |

The old per-waypoint child-action loop and separate ordinary takeoff/landing transactions are removed from mission execution. The replacement adds raw mission construction, MAVLink upload/progress handling, and native-mode monitoring. It does not provide a net reduction in application source code.

Standalone Takeoff, NavigateTo, FollowTrajectory, OrbitPoint, and Land remain available. They serve individual commands and flight tests. Timed trajectories retain their timing and yaw contracts. Finite orbits retain measured revolution counting and Offboard recovery.

PX4 owns ordinary route execution after upload and arming. The companion retains input validation, route generation, command admission, monitoring, cancellation, and result reporting.

## Connection setup

Rebuild the image after changing the pinned dependencies:

```bash
./scripts/sim up --build -d simulation
./scripts/sim-shell
colcon build --symlink-install
source /workspace/install/setup.bash
```

Start the [controlled simulation](../../README.md#simulation-flight-procedure), including QGroundControl and passing PX4 preflight checks. Launch the application with:

```bash
ros2 launch drone_bringup single_vehicle.launch.py \
  enable_flight_commands:=true \
  mission_connection_url:=udpin://0.0.0.0:14540 \
  px4_system_id:=1
```

- The first local SITL vehicle sends its mission-capable MAVLink stream to UDP 14540. QGroundControl uses UDP 14550.
- Each vehicle requires a separate mission endpoint and matching system ID. ROS namespaces and DDS topic prefixes do not configure MAVLink routing.
- DDS remains required for status, position, command acknowledgments, and stop confirmation.
- A valid global reference is required even when public input uses relative ENU offsets. Local-only waypoint execution is unsupported by these mission endpoints.
- Only one uploader may control the stored mission during a run. Uploading or selecting mission items through QGroundControl during an active ROS mission invalidates the wrapper's progress assumptions.
- Hardware needs a configured MAVLink link in addition to DDS. The [Pi setup](../setup/HITL-PI-SETUP.md) describes the native build and serial-link requirements. The [remote simulation guide](../setup/pi-desktop-sim.md#configure-the-mission-link) describes desktop-to-Pi routing.

## Acceptance and completion

The upload contains Takeoff, waypoint, optional ROI setup/clear, and optional Land items. Latitude and longitude use MAVLink integer scaling. AMSL altitude is explicit in each compiled geographic destination. The captured geographic start supplies the vertical takeoff climb.

Vehicle control rejects non-finite values, more than 256 waypoints, heights above 120 meters from mission start, and waypoints beyond 1,000 horizontal meters from mission start. JSON defaults are 30 meters vertically and 100 meters horizontally. Takeoff is a vertical climb. Movement or a reference change before arming invalidates the start conditions.

Native item deadlines are at most 600 seconds. Zero on the public waypoint action selects the configured default. Companion timeout parameters and standalone operation timeouts are capped at 3,600 seconds.

Waypoint acceptance uses PX4's mission acceptance radius and a 0.5-second dwell. PX4 altitude and trajectory parameters also affect acceptance and motion. This differs from the former NavigateTo 3D distance and heading-settle conditions. Mission look-at uses ROI items and clears ROI after the waypoints. It does not promise standalone NavigateTo's measured heading tolerance.

Success requires confirmed mission completion. A landing route also requires fresh touchdown confirmation and disarmed state. A route without landing confirms native Hold. Hold requires global position and follows PX4's configured altitude policy.

Geographic destinations remain fixed after an estimator local-frame reset. Geographic distance feedback remains available across local revisions. A finite-orbit tail projects its geographic center into the current reference at handoff and checks subsequent local revisions.

## Cancellation, handoff, and process loss

Cancellation confirms native Hold, or disarming if the vehicle is still landed. An unconfirmed stop returns failure. Explicit landing cancels the active child and waits for its terminal result before dispatching Land. PX4 failsafes and externally selected flight modes retain control.

Standalone navigation, trajectories, and orbit accept native Hold. Vehicle control prestreams a current-position target, confirms Offboard, waits for hold settling, and then begins the requested motion.

The ROS API exposes no mission pause/resume or airborne route replacement. Cancellation ends the ROS action. The route remains stored on PX4; an external transition back to Mission can resume that stored route. A new public waypoint or JSON mission requires a landed, disarmed start.

Stopping the ROS launch is not a landing command. An uploaded mission can continue after the vehicle-control process exits. Flight-controller parameters and remaining communication links determine the response to companion or link loss. Use the [landing procedure](../setup/LINUX.md#stop-a-flight) before stopping application processes.

The [testing guide](../testing/README.md#validation-status) records current SITL evidence, unresolved failures, and hardware validation limits.
