# How the container works

Design notes. To just get running, see [docs/SETUP.md](../docs/SETUP.md).

## One config

There is one `devcontainer.json`. The GUI always goes to the browser at
localhost:6080, on every OS.

There used to be three (browser, Linux native X11, Windows WSLg). Removed
because:

- Compose published ports 6080 and 5901 always, but the VNC server only started
  in browser mode. The native configs showed two forwarded ports with nothing
  behind them and no error.
- `devcontainer.json` also listed those ports in `forwardPorts`. Two mechanisms
  binding one host port fight: VS Code wins, compose then fails with `address
  already in use`. Never declare both.
- X11 access control differs between Wayland and Xorg sessions.
- Docker Desktop puts WSLg somewhere other than Microsoft's docs say.
- A picker means somebody picks wrong.

## VirtualGL

Xvfb only gives Mesa llvmpipe, even with a GPU in the container, because GLX in
a virtual framebuffer has no route to the card. VirtualGL intercepts GLX,
renders through EGL on the GPU, and hands frames back.

`gz sim` with `shapes.sdf` on an RTX 3070:

| | Renderer | RTF |
|---|---|---|
| Plain Xvfb | llvmpipe | 0.81 |
| `vglrun -d egl` | RTX 3070 | 1.00 |

Applied unconditionally because with no usable GPU `vglrun -d egl` falls back to
llvmpipe and exits 0. One code path, nothing to detect.

Wrappers at `/usr/local/bin/gz` and `/usr/local/bin/rviz2` route through it, so
nobody types `vglrun`. ROS's own gz at
`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` is wrapped too, because
`source /opt/ros/jazzy/setup.bash` puts that directory ahead of
`/usr/local/bin` and would silently drop acceleration. `DRONE_VGL=0` bypasses.

Only NVIDIA is accelerated. `/dev/dri` is not passed in, so AMD and Intel GPUs
get software rendering.

## Base image

`osrf/ros:jazzy-desktop-full` is amd64 only:

```console
$ docker manifest inspect osrf/ros:jazzy-desktop-full | grep architecture
    "architecture": "amd64"

$ docker manifest inspect ros:jazzy-ros-base | grep architecture
    "architecture": "amd64",
    "architecture": "arm64",
```

On Apple Silicon an amd64 base means qemu emulation, which makes PX4 plus
Gazebo unusable. So the image starts from `ros:jazzy-ros-base` and adds the
desktop with apt. `ros-jazzy-desktop` and `gz-harmonic` both ship arm64 for
noble, so Apple Silicon gets a native image.

## Pinned versions

| Component | Version | Why |
|---|---|---|
| ROS 2 | Jazzy | Current LTS, to May 2029. Humble expires May 2027 |
| Ubuntu | 24.04 | Ships with Jazzy. PX4's `ubuntu.sh` supports it |
| Gazebo | Harmonic | The version [Jazzy pairs with](https://github.com/gazebosim/ros_gz) |
| PX4 | v1.16.1 | Validated by this team. v1.17.0 is stable, upgrade as its own commit |
| px4_msgs | `release/1.16` | Matched to PX4. Mismatches fail confusingly |
| Micro XRCE-DDS Agent | v2.4.3 | v2.4.2 cannot build, its superbuild pins a deleted Fast DDS branch ([#370](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/370)). v2.4.3 pins Fast DDS 2.14.x and keeps the 2.x client protocol PX4 v1.16.1 speaks. Not v3.0.1, agent 3.x rejects clients older than 3.0.0 |

Versions live only in the `ARG` defaults at the top of the Dockerfile. Do not
add a `build.args` block for them, it overrides those defaults and the two
drift. `smoke-test.sh` checks for that.

## Startup

The container's own command starts the desktop, not a `postStartCommand` hook.
Hooks can silently fail to run, and the symptom was 6080 forwarded with nothing
behind it.

Scripts run from the bind-mounted repo in preference to the copies baked into
the image, so editing them takes effect on the next container start with no
rebuild. Falls back to `/opt`, then to bare `sleep infinity` so you always get
a shell.

`drone-gui` reports and restarts the desktop. `drone-sim` launches PX4 SITL.
`drone-gcs` provides the GCS link PX4 needs to arm.

## `.devcontainer/.env`

Gitignored, optional, per-machine. `scripts/doctor.sh --write` generates it.
See `.env.example`.

```ini
DOCKER_RUNTIME=nvidia          # only if `docker info` lists an nvidia runtime
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=all
```

`runtime:` rather than compose's `deploy.resources.devices`, because `devices`
makes the container fail to start on machines with no NVIDIA GPU, which is most
of the team's laptops and every Mac.

## Why `build/` looks empty

`compose.yaml` mounts named volumes over `build/`, `install/` and `log/`. On
macOS and Windows bind mounts cross a VM boundary and small-file writes are an
order of magnitude slower. A colcon build writes tens of thousands of them.

They are gitignored build artifacts. To wipe:

```bash
docker volume rm drone-vip_colcon-build drone-vip_colcon-install drone-vip_colcon-log
```

## Prebuilt image

CI builds multi-arch and pushes to GHCR. Then in `.devcontainer/.env`:

```ini
DRONE_IMAGE=ghcr.io/YOUR-ORG/DroneSwarming-dev:jazzy
```

The tag is a manifest list, so the same line is right on Intel and Apple
Silicon. Without it the image builds locally once.

## Layout

```
.devcontainer/
  devcontainer.json    the one config
  compose.yaml         the one service
  Dockerfile           multi-arch image
  .env.example         per-machine options, all optional
  scripts/
    container-entry.sh  container command: starts the desktop, stays alive
    start-display.sh    Xvfb + x11vnc + noVNC
    drone-gui           report and restart the desktop
    drone-sim           launch PX4 SITL with the x500
    drone-gcs           GCS heartbeat so PX4 can arm
    post-create.sh      first-run setup
    build-workspace.sh  compiles anything not baked in
scripts/
  doctor.sh, doctor.ps1   host-side check
```
