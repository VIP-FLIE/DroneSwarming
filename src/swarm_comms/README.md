# swarm_comms

Runs a leader/follower prototype. Slot 0 flies the route. Each later slot follows the vehicle before it using `/swarm/state`.

Run from the project container:

```bash
ros2 launch swarm_comms swarm.launch.py count:=2
```

The launch starts Micro XRCE-DDS Agent and PX4. PX4 instance 0 uses system ID 1 and `/fmu/...` topics. Later instances use their PX4 instance number as the namespace suffix and system ID plus one.

| Argument | Default | Effect |
|---|---:|---|
| `count` | `3` | Number of PX4 instances and flight nodes |
| `follow_distance_m` | `5.0` | Desired gap behind the preceding vehicle |
| `pad_spacing_m` | `5.0` | East spacing between spawn points |
| `radius_m` | `8.0` | Leader route radius |
| `altitude_m` | `4.0` | First vehicle's altitude |
| `layer_m` | `2.0` | Altitude increase per slot |
| `laps` | `1` | Leader route laps |
| `world` | `default` | PX4 Gazebo world name. Defaults to its stock `default` world. |
| `headless` | `false` | Disable Gazebo rendering when `true` |

This prototype shares vehicle state and follows the preceding vehicle. It does not allocate search areas, avoid obstacles, or reassign a failed vehicle's work.
