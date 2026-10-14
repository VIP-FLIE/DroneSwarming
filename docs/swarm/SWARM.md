# Swarm stages

The current packages cover one vehicle, independent multi-vehicle routes, and a communicating leader/follower prototype.

## One vehicle

Start one PX4 SITL instance and one flight node:

```bash
ros2 launch swarm_naive swarm.launch.py count:=1
```

## Two independent vehicles

```bash
ros2 launch swarm_naive swarm.launch.py count:=2
```

Each node receives a route at launch and flies it independently. The nodes do not share position or coordinate their routes.

## Communication prototype

```bash
ros2 launch swarm_comms swarm.launch.py count:=2
```

Slot 0 flies a circular route. Each later slot follows the vehicle before it using `/swarm/state`. The example does not assign search sectors, avoid obstacles, or reassign a failed vehicle's route.

## Planned development order

1. Fly one real X500 programmatically.
2. Fly two X500s at the same time on independent routes.
3. Add obstacle detection and avoidance.
4. Assign search areas through inter-vehicle communication.
5. Reassign unfinished work when a vehicle stops reporting.

The simulator runs one PX4 process per vehicle and uses each PX4 instance's uXRCE-DDS namespace. Instance 0 uses `/fmu/...`; instance `N > 0` uses `/px4_N/fmu/...`. See [simulation limits](../simulation/SIMULATION-FIDELITY.md) before using SITL results for hardware decisions.
