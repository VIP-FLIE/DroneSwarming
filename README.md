# DroneSwarming

ROS 2 control software for PX4 X500 search and rescue drones. The Linux simulation uses PX4 SITL and Gazebo Harmonic in Docker. QGroundControl runs on the Linux host.

## Requirements

- Ubuntu 24.04
- Docker Engine with Docker Compose
- QGroundControl

The image pins PX4 SITL to `v1.18.0-beta1-660-g61907962c9` by tag and digest. It includes Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Micro XRCE-DDS Agent v2.4.3, and matching `px4_msgs`.

macOS and Windows setup is unverified. If you get the workflow running on either platform, document the steps in [docs/platforms](docs/platforms/README.md).

## Start

Build and start the container:

```bash
docker compose up --build -d
```

Start QGroundControl on the host. Open a shell in the container:

```bash
docker compose exec simulation bash
```

## Fly one drone

Run the launch file in the container shell:

```bash
ros2 launch swarm_naive swarm.launch.py count:=1
```

The launch starts the XRCE-DDS Agent, one PX4 SITL instance, Gazebo, and one flight node. PX4 sends MAVLink to QGroundControl on UDP port 14550.

## Fly two independent drones

```bash
ros2 launch swarm_naive swarm.launch.py count:=2
```

Each drone flies its assigned route. The nodes do not exchange swarm state.

## Edit code

The host `src/` directory is mounted at `/workspace/src`, so edits appear in the running container. Existing Python and launch file changes are available to new processes. Rebuild after C++ source changes, then restart the affected process:

```bash
colcon build --symlink-install
```

## Stop

Press Ctrl-C in the launch shell, then stop the container:

```bash
docker compose down
```

## Documentation

- [Linux setup](docs/setup/LINUX.md)
- [Simulation](docs/simulation/HOW-IT-WORKS.md)
- [Simulation limits](docs/simulation/SIMULATION-FIDELITY.md)
- [Gazebo world resources](worlds/README.md)
- [Swarm stages](docs/swarm/SWARM.md)
