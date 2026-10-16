# Architecture diagrams

The Graphviz sources in this directory describe the active single-vehicle system and the planned swarm capabilities.

| Diagram | Source | Description |
| --- | --- | --- |
| Current system | [`current-system.dot`](current-system.dot) | ROS 2 application nodes, action and state flows, PX4 integration, and simulation processes that run today. |
| Future system | [`future-system.dot`](future-system.dot) | Planned multi-vehicle search and rescue, task assignment, failure recovery, and victim reporting. |

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

`current-system.dot` shows implemented ROS application nodes and the external processes used in the single-vehicle simulation. It omits the external mission caller and the empty `swarm_coordination` scaffold. `future-system.dot` shows the multi-vehicle search-and-rescue objective, including search-region assignment, peer health and failure monitoring, unfinished-task reassignment, and victim detection/reporting. It labels future behavior and leaves coordination topology and protocols unspecified where the repository does not define them.
