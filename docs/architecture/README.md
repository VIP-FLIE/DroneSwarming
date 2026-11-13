# Architecture diagrams

The Graphviz sources in this directory describe the active single-vehicle system and the planned swarm capabilities.

Read the [codebase tour](CODEBASE-TOUR.md) for a source-linked walkthrough of the ROS 2 nodes, action flows, PX4 adapter, coordinate conversion, and vehicle-control state flow.

The [mission admission and arbitration draft](MISSION-ADMISSION-AND-ARBITRATION.md) records the proposed handling of future swarm, operator, and perception requests. It compares actions, a separate admission service, and intent topics, with open decisions for source permissions, priority, preemption, and validation. The proposal is unimplemented.

| Diagram | Source | Description |
| --- | --- | --- |
| Current system | [`current-system.dot`](current-system.dot) | ROS 2 application nodes, action and state flows, PX4 integration, and simulation processes that run today. |
| Future system | [`future-system.dot`](future-system.dot) | Implemented JSON handoff within a planned multi-vehicle fleet, plus future intent arbitration, swarm coordination, operator instructions, and perception results. |

## Generate diagrams

Install Graphviz and GNU Make. Run `make diagrams` from the repository root to generate SVG and PNG files under `docs/architecture/generated/`.

```sh
make diagrams
```

The generated directory is ignored by Git. Run `make clean-diagrams` to remove the generated files.

To render one source file directly, run Graphviz from the repository root:

```sh
dot -Tsvg docs/architecture/current-system.dot \
  -o docs/architecture/generated/current-system.svg
dot -Tpng -Gdpi=180 docs/architecture/current-system.dot \
  -o docs/architecture/generated/current-system.png
```

## Add or modify a diagram

1. Add a `.dot` source file in `docs/architecture/`.
2. Add its base name to `DIAGRAMS` in the root `Makefile`.
3. Use the shared SVG and PNG rules to generate both formats with `make diagrams`.
4. Keep implementation details in the current diagram tied to package code and documentation.
5. Label planned behavior and provisional contracts as planned or provisional in the future diagram.
6. Keep node labels short. Put detailed descriptions in this guide or the package documentation.

Both diagrams use charcoal backgrounds, bright node fills, and high-contrast labels. They use 20-point node labels and 17-point edge labels. Set Graphviz `fontsize`, `nodesep`, and `ranksep` values in the source to control label size and spacing. SVG is resolution-independent. The PNG target renders at 180 DPI.

## Diagram status

`current-system.dot` shows implemented ROS application nodes and the external processes used in the single-vehicle simulation. It includes external mission callers, flight-test intent, and the runner's read-only telemetry path. It omits the empty `swarm_coordination` scaffold. `future-system.dot` shows the multi-vehicle search-and-rescue objective, including search-region assignment, peer health and failure monitoring, unfinished-task reassignment, and victim detection/reporting. It labels future behavior and leaves coordination topology and protocols unspecified where the repository does not define them.


## Active command ownership

External mission callers and the flight-test runner submit every motion request to `mission_execution`.
It consumes `VehicleState`, checks freshness and validity, reserves one ordinary command, and selects
vehicle operations. Only this application node creates clients for `vehicle_control/*` actions.
The public operation API reuses the existing action types to retain trajectory fields, orbit
progress, navigation cancellation, measured results, and feedback. `execute_waypoint_mission`
retains its public contract and derives all offsets from one mission-start position.

An explicit `mission_execution/land` request interrupts ordinary work, cancels its active child,
and waits for a terminal result before dispatching landing. A missing child terminal result blocks
further dispatch and produces a bounded failure. Endpoint scoping documents ownership; it does
not enforce DDS access control against external ROS participants.

`vehicle_control` validates and executes the selected operation through its PX4 adapter and publishes
vehicle state. The flight-test runner also reads that state for measurements. Any resulting test
intent reenters mission execution. The adapter owns all PX4 messages, topics, and ENU/NED conversion.
PX4 retains low-level control, preflight checks, and failsafes. Flight commands default to disabled.

## Planned inputs and arbitration

JSON flight planning is implemented. File callers submit content to `mission_execution/execute_flight_plan`. Mission execution captures fresh state and calls `mission_planner/plan_flight`; the planner selects the explicit vehicle assignment and returns translated instructions. The executor validates them and owns vehicle-action dispatch. See the [JSON handoff and extension plan](JSON-FLIGHT-PLANS.md) and [schema guide](../../src/mission_planner/README.md).

Swarm workload rebalances, operator special instructions, and perception/detection results that affect flight are planned inputs to mission execution. Their runtime arbitration is unimplemented.

The planned priority order is:

1. Safety conditions that require landing.
2. Urgent special goals or instructions, including a swarm workload rebalance, an operator request,
   or a future detection result requesting a circle around a located person.
3. The normal planned flight path.

Mission execution is the planned arbitration point. Vehicle control will execute the selected
operation. PX4 will retain low-level control and failsafes. The current explicit-landing preemption
implements only the present command contract. It does not implement a future safety classifier or
urgent-input arbitration.

Plan replacement, equal-priority ordering, source authorization, instruction expiry, detection confidence, interrupted-route resume, and response to conflicting future inputs remain undefined. These policies belong at mission admission before those sources are implemented. Fresh valid telemetry and an unchanged frame revision are required for translated JSON missions. Swarm protocols and perception nodes remain unimplemented.
