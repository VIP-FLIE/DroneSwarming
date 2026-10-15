# Linux setup

The primary workflow targets Ubuntu 24.04. ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic, the XRCE-DDS Agent, and `px4_msgs` run inside the project container. QGroundControl runs on the host.

## Requirements

- Ubuntu 24.04 desktop
- Docker Engine and the Docker Compose plugin
- NVIDIA Container Toolkit configured for Docker
- QGroundControl
- An X11 desktop session, or a Wayland session with XWayland enabled

The simulation container uses the host NVIDIA GPU for Gazebo rendering. Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), then configure Docker and restart its daemon:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

The NVIDIA display driver remains installed on the host; ROS 2, PX4, Gazebo, and the XRCE-DDS Agent run in the container.

Install Docker Engine with the [Ubuntu instructions](https://docs.docker.com/engine/install/ubuntu/). The host does not need ROS 2 or PX4.

## Install QGroundControl

QGroundControl runs on the Ubuntu host and receives simulated PX4 telemetry over MAVLink UDP port 14550. Download the stable Linux AppImage for your architecture from the [official QGroundControl installation guide](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html).

Install the AppImage runtime dependencies listed by QGroundControl:

```bash
sudo apt install libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
```

Save the downloaded file as `~/Downloads/QGroundControl.AppImage`, then make it executable and run it:

```bash
chmod +x ~/Downloads/QGroundControl.AppImage
~/Downloads/QGroundControl.AppImage
```

For USB or serial telemetry on a physical vehicle, follow the guide's additional serial-port and ModemManager instructions.

## Enable Gazebo display access

Gazebo Harmonic uses Qt's `xcb` backend to display its GUI. On Wayland, the GUI connects through XWayland. This uses the same container setup on Wayland and X11 sessions. Gazebo documents `xcb` as the workaround for Wayland GUI issues ([Gazebo Harmonic troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/#wayland-issues)).

The container runs as root. `scripts/sim` grants the root user display access before starting the container and revokes it after `stop` or `down`. It queries the X access list after each change to confirm the `SI:localuser:root` entry. If X access control is disabled, the script reports that display access remains open to all clients. Use `scripts/sim` for container lifecycle commands. Direct Docker or Docker Compose stop and removal commands bypass the display access cleanup.

The script uses `xhost` from Ubuntu's `x11-xserver-utils` package. Install the package if `xhost` is missing:

```bash
sudo apt install x11-xserver-utils
```

## Build the image

Run from the repository root:

```bash
./scripts/sim up --build -d
```

### Optional useful flags

- `--progress=plain` prints full build output, including Dockerfile command traces. Example: `./scripts/sim --progress=plain up --build -d`.

The build uses PX4's Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic toolchain image, pinned by OCI digest. It builds PX4 SITL from the stable `v1.17.0` source tag and uses its `bin/px4` executable directly. It builds `px4_msgs` from the matching `v1.17.0` tag, then builds the repository packages with `colcon`.

The container is named `swarm-sim`.

Docker Compose mounts the host `src/` directory at `/workspace/src`. Edit source files with the host editor. Existing Python modules and launch files are available to new processes without rebuilding the image. Rebuild C++ packages in the container after C++ changes:

```bash
colcon build --symlink-install
```

## Container lifecycle

Run these commands from the repository root.

| Action | Command | Effect |
|---|---|---|
| Stop | `./scripts/sim stop` | Stops the container and revokes display access. |
| Start | `./scripts/sim start` | Starts a stopped container and grants display access. |
| Restart | `./scripts/sim restart` | Restarts the container and grants display access. |
| Rebuild | `./scripts/sim up --build -d` | Rebuilds the image and starts the container. Use after Dockerfile or container dependency changes. |
| Shut down | `./scripts/sim down` | Stops and removes the container, then revokes display access. The image and source files remain. |

## Start a flight

Start QGroundControl on the host. Open a container shell:

```bash
./scripts/sim exec simulation bash
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

## Stop a flight

Press Ctrl-C in the launch shell to stop the flight. Use [Container lifecycle](#container-lifecycle) to stop, restart, rebuild, or remove the container.
