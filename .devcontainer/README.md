# How the container works

Design notes. To just get running, see [docs/SETUP.md](../docs/SETUP.md).

## The display path

Gazebo and RViz are X11 applications and a container has no screen, so the
container supplies one:

```
Xvfb            a screen that exists only in memory
  |
x11vnc          serves that screen on port 5901
  |
noVNC           carries 5901 over HTTP on port 6080
```

You attach to it at `localhost:6080` in a browser or at `localhost:5901` with a
VNC client. x11vnc runs with `-shared`, so several viewers at once is fine.

Identical on every OS. Nothing is installed or configured on the host.

`compose.yaml` publishes both ports, bound to `127.0.0.1`. `devcontainer.json`
must not also list them in `forwardPorts`. Two mechanisms binding one host port
conflict: VS Code takes it, compose then fails with `address already in use`
and the container does not start. Compose publishing is used rather than
`forwardPorts` because it also works for the devcontainer CLI and for editors
that do not implement `forwardPorts` at all.

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

The container's own command starts the desktop rather than a `postStartCommand`
hook. A hook that fails to run reports no error, which leaves the ports
published with nothing serving them.

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
DRONE_IMAGE=ghcr.io/vip-flie/droneswarming-dev:jazzy
```

The tag is a manifest list, so the same line is right on Intel and Apple
Silicon. Without it the image builds locally once.

Note the all-lowercase path. GitHub displays the repo as `VIP-FLIE/DroneSwarming`,
but container registries reject any reference whose repository part is not
lowercase, so the published image does not match the repo's display casing.

## Layout

```
.devcontainer/
  devcontainer.json    what VS Code opens
  compose.yaml         the service definition
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
  doctor.sh   host-side check
```
