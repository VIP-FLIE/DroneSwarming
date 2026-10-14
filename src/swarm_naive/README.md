# swarm_naive

Runs one PX4 SITL process and one `waypoint_drone` node per vehicle. Each node flies a route assigned at launch. The nodes do not exchange state.

Run from the project container:

```bash
ros2 launch swarm_naive swarm.launch.py count:=1
ros2 launch swarm_naive swarm.launch.py count:=2
```

The launch starts Micro XRCE-DDS Agent and PX4. The first vehicle uses PX4 instance 0, system ID 1, and `/fmu/...` topics. Later vehicles use instance `N`, system ID `N + 1`, and `/px4_N/fmu/...` topics.

| Argument | Default | Effect |
|---|---:|---|
| `count` | `3` | Number of PX4 instances and flight nodes |
| `pad_spacing_m` | `5.0` | East spacing between spawn points |
| `radius_m` | `8.0` | Radius of each route |
| `altitude_m` | `4.0` | First vehicle's altitude |
| `layer_m` | `2.0` | Altitude increase per slot |
| `laps` | `1` | Route laps |
| `world` | `default` | PX4 Gazebo world name. Defaults to its stock `default` world. |
| `headless` | `false` | Disable Gazebo rendering when `true` |

`swarm_naive` does not detect obstacles or coordinate routes. Use [swarm_comms](../swarm_comms/README.md) for the communication prototype.
