# Aerostack2 Migration Plan

Current environment: ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16.1, Ubuntu 24.04.
Aerostack2 requires ROS 2 Humble, Ubuntu 22.04.

## Architecture

Drones run three nodes. Aerostack2 runs on the ground station.

```
Drone 1                              Drone 2
┌──────────────────────┐            ┌──────────────────────┐
│ Mission Executor     │◄──────────►│ Mission Executor     │
│   ↓↑                 │ /swarm/state│   ↓↑               │
│ Pattern Planner      │            │ Pattern Planner      │
│   ↓↑                 │            │   ↓↑                 │
│ PX4 Interface        │            │ PX4 Interface        │
│   ↓↑                 │            │   ↓↑                 │
│ PX4 Flight Controller│            │ PX4 Flight Controller│
└──────────────────────┘            └──────────────────────┘
        ▲                                    ▲
        │ Mission upload                     │ Mission upload
        │                                    │
┌────────────────────────────────────────────────────┐
│ Aerostack2 Ground Control                          │
│ ├── Mission Planner                                │
│ └── Ground Station (subscribes to /swarm/state)    │
└────────────────────────────────────────────────────┘
```

### Drone nodes

| Node | Function |
|---|---|
| Mission Executor | Receives mission from planner. Manages mission state. Publishes/subscribes `/swarm/state`. Sends mission definition to Pattern Planner. Sends flight commands to PX4 Interface. |
| Pattern Planner | Receives mission definition. Generates flight plan: waypoints, trajectories, timing. Returns flight plan to Mission Executor. |
| PX4 Interface | Talks to PX4 via uXRCE-DDS. Arming, offboard mode, trajectory setpoints, telemetry. Same as current PX4 code in `formation_drone.cpp`. |

### Ground control

| Component | Function |
|---|---|
| Mission Planner | Define missions. Upload to each drone's Mission Executor. |
| Ground Station | Telemetry display. Subscribes to `/swarm/state`. |

### Data flow

1. Mission Planner uploads mission definition to Mission Executor on each drone.
2. Mission Executor sends mission definition to Pattern Planner.
3. Pattern Planner generates flight plan, returns it to Mission Executor.
4. Mission Executor executes flight plan: sends commands to PX4 Interface.
5. PX4 Interface sends offboard setpoints to PX4.
6. Mission Executor publishes state on `/swarm/state`. Other drones receive it.
7. Ground Station subscribes to `/swarm/state` for telemetry.

## What Aerostack2 provides

| Component | Package |
|---|---|
| Mission Planner | `as2_mission_planner` |
| Ground Station | `as2_alphanumeric_viewer` |

## What you write

| Package | Contents |
|---|---|
| `mission_executor` | Mission Executor node |
| `pattern_planner` | Pattern Planner node |
| `px4_interface` | PX4 Interface node (adapted from `formation_drone.cpp`) |

Drone nodes are compiled once. Missions change by re-running the mission
planner.

## Installation

### 1. Target platform

Aerostack2 requires ROS 2 Humble on Ubuntu 22.04. Options:

- Install Ubuntu 22.04 alongside 24.04.
- Run Ubuntu 22.04 in a VM or Docker container.
- Port Aerostack2 to ROS 2 Jazzy (unsupported).

### 2. Install Aerostack2

```bash
cd ~
git clone https://github.com/aerostack2/aerostack2.git
cd aerostack2
```

Follow install instructions at:
https://aerostack2.github.io/_00_getting_started/index.html

### 3. PX4 and Gazebo

Unchanged from current setup.

## Files touched

### New

| File | Purpose |
|---|---|
| `src/mission_executor/` | Mission Executor node |
| `src/pattern_planner/` | Pattern Planner node |
| `src/px4_interface/` | PX4 Interface node |
| `config/mission.yaml` | Mission definition |

### Replaced

| File | Replaced by |
|---|---|
| `src/swarm_comms/` | `src/mission_executor/` + `src/pattern_planner/` + `src/px4_interface/` |
| `src/swarm_naive/` | Above |
| `scripts/drone-gcs` | Aerostack2 ground station |

### Unchanged

| File |
|---|
| `src/cpp_single_hop/` |
| `src/py_single_square/` |
| `worlds/` |
| `scripts/drone-sim` |
| `scripts/install.sh` |

## Workflow

### Before

```
Terminal 1: MicroXRCEAgent udp4 -p 8888
Terminal 2: drone-gcs
Terminal 3: ros2 launch swarm_comms swarm.launch.py
```

### After

```
Terminal 1: MicroXRCEAgent udp4 -p 8888
Terminal 2: ros2 launch drone_stack swarm.launch.py
Terminal 3: ros2 launch aerostack2 ground_control.launch.py
```

Terminal 2 starts PX4 instances and the three drone nodes per vehicle.
Terminal 3 starts the mission planner and ground station. Define the mission
in the planner. Upload. Drones execute.

## Migration steps

1. Set up Ubuntu 22.04 with ROS 2 Humble for the ground station.
2. Install Aerostack2.
3. Extract PX4 interface code from `formation_drone.cpp` into `src/px4_interface/`.
4. Write `src/pattern_planner/`: receives mission definition, outputs flight plan.
5. Write `src/mission_executor/`: receives mission from planner, coordinates
   Pattern Planner and PX4 Interface, publishes/subscribes `/swarm/state`.
6. Write launch file for N drones.
7. Write Aerostack2 ground control launch file.
8. Run smoke test.

## Reference

- Aerostack2 repository: https://github.com/aerostack2/aerostack2
- Aerostack2 paper: https://arxiv.org/abs/2303.18237
- Architecture diagram: `docs/architecture/waypoints/renders/arch-waypoint.png`