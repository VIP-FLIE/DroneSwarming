# Linux setup

The primary workflow targets Ubuntu 24.04. ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic, the XRCE-DDS Agent, and `px4_msgs` run inside the project container. QGroundControl runs on the host.

## Requirements

- Ubuntu 24.04 desktop
- Docker Engine and the Docker Compose plugin
- QGroundControl
- X11 display access for Gazebo

Install Docker Engine with the [Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/). Install QGroundControl with the [Ubuntu instructions](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html). The host does not need ROS 2 or PX4.

## Build the image

Run from the repository root:

```bash
docker compose up --build -d
```

The build uses PX4's `v1.18.0-beta1-660-g61907962c9` ROS 2 Gazebo image, pinned by digest. The Dockerfile checks that the image reports Ubuntu 24.04 and ROS 2 Jazzy. It builds the repository packages with `colcon`.

Docker Compose mounts the host `src/` directory at `/workspace/src`. Edit source files with the host editor. Existing Python modules and launch files are available to new processes without rebuilding the image. Rebuild C++ packages in the container after C++ changes:

```bash
colcon build --symlink-install
```

Rebuild the image after changing the Dockerfile or container dependencies:

```bash
docker compose up --build -d
```

## Start a flight

Start QGroundControl on the host. Open a container shell:

```bash
docker compose exec simulation bash
```

Start one drone:

```bash
ros2 launch swarm_naive swarm.launch.py count:=1
```

Start two independent drones:

```bash
ros2 launch swarm_naive swarm.launch.py count:=2
```

The launch file starts `MicroXRCEAgent`, PX4 SITL, Gazebo, and the flight nodes. PX4 uses its default MAVLink connection to QGroundControl on UDP port 14550. Docker uses host networking so the connection uses the host network stack.

## Stop

Press Ctrl-C in the launch shell. Stop the container from the repository root:

```bash
docker compose down
```
