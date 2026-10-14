# Simulation

The Linux container pins PX4 SITL to `v1.18.0-beta1-660-g61907962c9` and includes ROS 2 Jazzy, Gazebo Harmonic, Micro XRCE-DDS Agent v2.4.3, and matching `px4_msgs`. QGroundControl runs on the host and uses PX4's MAVLink connection on UDP port 14550.

## Processes

| Process | Role |
|---|---|
| PX4 SITL | Autopilot software and vehicle state |
| Gazebo Harmonic | X500 model, simulated sensors, physics, and selected world |
| Micro XRCE-DDS Agent | Bridges PX4's uXRCE-DDS client to ROS 2 |
| ROS 2 flight node | Sends offboard setpoints and vehicle commands |
| QGroundControl | Ground control station and MAVLink telemetry |

The launch file starts the Agent before PX4. It starts one PX4 process for each vehicle. PX4 instance 0 uses the unprefixed `/fmu/...` topics and system ID 1. Later instances use `/px4_N/fmu/...` and system ID `N + 1`.

## Run

Follow the [Linux setup](../setup/LINUX.md), then start one drone:

```bash
ros2 launch swarm_naive swarm.launch.py count:=1
```

Start two drones with independent routes:

```bash
ros2 launch swarm_naive swarm.launch.py count:=2
```

The communicating prototype uses a shared ROS topic:

```bash
ros2 launch swarm_comms swarm.launch.py count:=2
```

The `swarm_comms` example demonstrates position sharing and leader/follower control. It does not allocate search areas or reassign work after a vehicle failure.

PX4 starts in its stock `default` world unless the `world` argument selects another world.
