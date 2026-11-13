# mission_planner

`mission_planner` compiles high-level JSON flight intent into bounded mission-start offsets for one vehicle. It creates no flight-action clients. `mission_execution` accepts JSON through `mission_execution/execute_flight_plan`, captures fresh vehicle state, calls `mission_planner/plan_flight`, validates the response, and executes takeoff, navigation or orbit, and landing through `vehicle_control`.

## Submit a file

Build and source the workspace. Start the [controlled simulation](../../README.md#simulation-flight-procedure), including the flight-enabled application. Submit the file through mission execution:

```bash
ros2 run mission_planner submit_flight_plan \
  /workspace/src/mission_planner/examples/relative-fleet.json
```

The root application selects `uav_1` by default. A namespaced application derives its `vehicle_id` from its namespace, replacing nested slashes with underscores. Set the launch argument `vehicle_id` for an explicit stable identifier. Submit the same fleet document separately to each vehicle's application namespace:

```bash
ros2 run mission_planner submit_flight_plan fleet.json --namespace /uav_1
ros2 run mission_planner submit_flight_plan fleet.json --namespace /uav_2
```

Each command waits for that vehicle's terminal mission result. Run the commands in separate terminals for concurrent submission. The file contents travel in the action goal; the executor does not read a caller-supplied server path. Ctrl-C requests mission cancellation and waits for the result. Cancellation drains the active child and leaves navigation in hold. Use `mission_execution/land` for an explicit recovery landing.

## Version 2 fleet schema

```json
{
  "version": 2,
  "missions": [
    {
      "vehicle_ids": ["uav_1"],
      "frame_id": "relative_enu",
      "takeoff_altitude_m": 3.0,
      "waypoint_acceptance_radius_m": 0.5,
      "path": {"type": "waypoints", "points": [[3, 0], [3, 3], [0, 0]]}
    }
  ]
}
```

The root contains only `version` and `missions`. Each assignment contains the five fields shown above. Coordinate-specific fields are listed below. Vehicle IDs contain 1 to 64 ASCII letters, digits, underscores, or hyphens and begin with a letter. Each ID occurs exactly once in the file. Unknown vehicles, duplicate assignments, unknown fields, and wildcard IDs fail validation. One assignment can name several IDs when the same intent is deliberate.

| Frame | Point order | Path altitude | Additional fields |
| --- | --- | --- | --- |
| `relative_enu` | `[east_m, north_m]` | `takeoff_altitude_m` above the captured vehicle start | None |
| `relative_enu` with shared origin | `[east_m, north_m]` from a fixed geographic anchor | `takeoff_altitude_m` above the anchor's AMSL altitude | `origin`: `latitude_deg`, `longitude_deg`, `altitude_amsl_m` |
| `wgs84` | `[latitude_deg, longitude_deg]` | Constant `altitude_amsl_m` | Required `altitude_amsl_m` |

GPS pairs use latitude first. This differs from GeoJSON. Altitudes are meters above mean sea level (AMSL), matching the estimator's published reference. Ellipsoid height and terrain-relative altitude are unsupported.

Version 2 `takeoff_altitude_m` always requests a vertical climb from that vehicle's captured starting position. Shared-origin and GPS path altitude can differ from the takeoff height. Vehicles launching at different elevations require their own climb values if the vertical departure must reach the same AMSL height. Every path destination must remain above that vehicle's mission-start altitude and within its configured envelope.

Relative coordinates without an origin translate with each vehicle's own starting position. Shared-origin and GPS routes require valid geographic-reference telemetry. They map to the same geographic positions even when vehicle estimator origins differ. GPS lawnmower geometry is expanded around a path-derived geographic anchor before conversion to each vehicle's local frame.

## Path types

All frames accept these path objects. Substitute GPS pairs for ENU pairs in `wgs84` paths. Distances, radii, spacing, and speed retain meter-based units.

| Type | Fields | Execution |
| --- | --- | --- |
| `waypoints` | `points`: ordered coordinate pairs | Navigate to each point at constant path altitude |
| `circle` | `center`, `radius_m`; optional `speed_m_s` (default 1) | One counterclockwise native orbit, then landing |
| `lawnmower` | `area`, `stripe_gap_m`; optional `stripe_heading_deg` | Expand alternating stripes into waypoints |

Lawnmower `area` is either `{"type":"polygon","vertices":[...]}` for a convex polygon, or `{"type":"circle","center":[...],"radius_m":...}`. Stripe heading is counterclockwise from east. Polygon heading defaults to the first polygon edge; circular-area heading defaults to east. Circle paths accept legacy `point_spacing_m` for compatibility; native orbit execution does not use it. Circle paths require geographic-reference telemetry at the vehicle-control boundary, including relative circles.

The examples directory contains [relative fleet](examples/relative-fleet.json), [GPS fleet](examples/gps-fleet.json), [shared origin](examples/shared-origin.json), [lawnmower](examples/lawnmower.json), and [legacy SAR triangle](examples/legacy-triangle.json). Replace geographic coordinates and AMSL heights with values for the actual launch site before submission.

## Validation and execution bounds

- JSON is UTF-8, at most 65,536 bytes. Duplicate keys, non-finite numbers, booleans in numeric fields, unknown versions, and malformed structures are rejected.
- Version 2 allows 1 to 64 assignments with 1 to 64 IDs each. Each route contains at most 256 generated waypoints; polygon areas contain at most 512 vertices.
- Default selected-vehicle bounds are 30 meters above mission start and 100 meters horizontally from mission start. The entire circle must fit the envelope. Hard configuration bounds are 120 meters vertically and 1,000 meters horizontally.
- Acceptance radius is positive and at most 10 meters. Circle radius is at least 1 meter, defaults to a maximum of 20 meters, and uses a default maximum speed of 3 meters per second. Centripetal acceleration must remain below 2 meters per second squared. Orbit centers also obey vehicle control's local-origin distance limit, default 100 meters.
- Orbit timeout is one revolution's nominal duration plus 30 seconds, with a minimum of 60 seconds and maximum of 600 seconds. Waypoint stages use the existing mission execution timeout defaults.
- Every assignment's schema and geometry is validated. Geographic-reference availability and the selected vehicle's envelope are checked for its own assignment.
- Compilation completes before takeoff. Mission execution independently validates translated instructions. Planner failure, unavailable service, timeout, invalid output, cancellation, and landing preemption cannot dispatch a late planning response.
- A changed local-frame revision or stale/invalid position aborts a translated mission and drains its active child. Route recovery requires a new plan submission after the vehicle state is usable.
- JSON missions start landed and disarmed and land after successful path completion. Failed or canceled paths retain the existing explicit recovery-landing contract.

The launch passes `maximum_plan_altitude_m` and `maximum_plan_distance_m` to both planner and executor. Orbit limits are shared with vehicle control. The executor owns admission, cancellation, landing priority, and all vehicle-action clients. The planner service performs computation without flight side effects.

## Version 1 compatibility

SAR files use root fields `version: 1`, `frame_id: "local_enu"`, `takeoff_altitude_m`, `waypoint_acceptance_radius_m`, and `path`. Coordinate pairs are absolute east/north positions from the estimator's local origin. `takeoff_altitude_m` is the absolute local Z for the path, so the executor's climb is that altitude minus its captured starting Z. The compiler preserves these meanings when producing mission-start offsets. Version 1 contains no vehicle assignment. Use version 2 for fleet files.

## Source and swarm scope

The parser, lawnmower geometry, CLI behavior, legacy example, and focused tests are adapted from `sar-drone` commit `6be553d`, from `src/sar_drone/sar_drone/{flight_plan,lawnmower,flight_plan_cli}.py` and its tests. The retained BSD license is [LICENSE.sar-drone](LICENSE.sar-drone). The SAR planner's direct mission submission, hardcoded `/sar` endpoints, and PX4 dependencies are replaced by the executor-owned handoff.

Fleet assignments are explicit. Automatic area partitioning, fleet broadcast, synchronized starts, inter-vehicle collision avoidance, peer health, allocation, and interrupted-route replanning are unimplemented. Sending overlapping geographic routes to several IDs does not provide separation. The [JSON architecture plan](../../docs/architecture/JSON-FLIGHT-PLANS.md) defines the implemented boundary and extension requirements.
