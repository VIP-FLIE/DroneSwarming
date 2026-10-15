# DroneSwarming

ROS 2 control software for PX4 X500 search and rescue drones. The Linux simulation uses PX4 SITL and Gazebo Harmonic in Docker. QGroundControl runs on the Linux host.

## Requirements

- Ubuntu 24.04
- Docker Engine with Docker Compose
- QGroundControl

Gazebo uses Qt's `xcb` display backend. On Wayland, this connects through XWayland; on X11, it connects to the active X server. The [Linux setup guide](docs/setup/LINUX.md) describes the display access command.

Gazebo rendering uses the host NVIDIA GPU through the NVIDIA Container Toolkit. See the [Linux setup guide](docs/setup/LINUX.md) for the host requirement.

The image builds PX4 SITL from the stable `v1.17.0` source tag and builds matching `px4_msgs` from its `v1.17.0` tag. It uses PX4's Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic toolchain image, pinned by digest. The base image includes Micro XRCE-DDS Agent v2.4.3. The Dockerfile builds PX4 from source because PX4 does not publish a stable `px4-sitl-gazebo` image for `v1.17.0`.

macOS and Windows setup is unverified. If you get the workflow running on either platform, document the steps in [docs/platforms](docs/platforms/README.md).

## Start

Build and start the container:

```bash
./scripts/sim up --build -d
```

The command grants display access before startup. The helper revokes access after the container stops or shuts down. The container is named `swarm-sim`.

### Optional useful flags

- `--progress=plain` prints full build output, including Dockerfile command traces. Example: `./scripts/sim --progress=plain up --build -d`.

Start QGroundControl on the host. Open a shell in the container:

```bash
./scripts/sim exec simulation bash
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
./scripts/sim down
```

## Documentation

- [Linux setup](docs/setup/LINUX.md)
- [Simulation](docs/simulation/HOW-IT-WORKS.md)
- [Simulation limits](docs/simulation/SIMULATION-FIDELITY.md)
- [Gazebo world resources](worlds/README.md)
- [Swarm stages](docs/swarm/SWARM.md)
