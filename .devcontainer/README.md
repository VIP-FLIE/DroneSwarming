# How the dev container works

_This is the 'why is it built like this' doc. If you just want to get running, go to [docs/SETUP.md](../docs/SETUP.md) instead, you do not need any of this to use the thing._

One container, same toolchain for everybody, regardless of what laptop you have. The only things that end up on your actual machine are Docker and an editor.

## The three configs:
When you reopen in a container, VS Code asks which one you want. They all run the exact same image and the exact same code. The only thing that differs is how the GUI gets from the container to your eyeballs.

- 'Universal (browser GUI)' works on anything, and it is where you should start. The container runs its own X server and publishes it over HTTP, so the window shows up at <http://localhost:6080> and there is nothing whatsoever to install on your side. No XQuartz, no VcXsrv, no WSLg. This is the only option that works on a Mac and it is the one I would recommend on Windows too.
- 'Linux (native X11 + GPU)' draws straight onto your desktop using your real X server, with your real GPU. Works on X11 and Wayland sessions both. Fastest option if you are on Linux.
- 'Windows (WSLg native display)' is the equivalent optional fast path on Windows.

The last two are speed optimizations, nothing more. If the host display turns out to be unreachable they quietly fall back to the browser GUI rather than leaving you staring at nothing, so picking the wrong one costs you frame rate and not a working setup.

## Why the base image had to change:
The obvious base image for this stack is `osrf/ros:jazzy-desktop-full`, and the original container used it. We cannot, because it is amd64 only:

```console
$ docker manifest inspect osrf/ros:jazzy-desktop-full | grep architecture
    "architecture": "amd64"          # one entry, that is it

$ docker manifest inspect ros:jazzy-ros-base | grep architecture
    "architecture": "amd64",
    "architecture": "arm64",         # manifest list
```

On an Apple Silicon Mac an amd64 only base means the whole stack runs through qemu emulation, and PX4 SITL plus Gazebo physics under emulation is not 'a bit slow', it is unusable. That would have quietly made every M series laptop on the team a second class citizen, which is exactly the thing this whole effort was supposed to prevent.

So the image starts from `ros:jazzy-ros-base`, which is a proper manifest list, and layers the desktop tooling on with apt instead. Both `ros-jazzy-desktop` and `gz-harmonic` publish arm64 packages for noble, so Apple Silicon gets a native image.

## Pinned versions:
Every version is fixed deliberately. If you change one, you are signing up to check the others.

| Component | Version | Why this one |
|---|---|---|
| ROS 2 | Jazzy | Current LTS, supported to May 2029. Humble expires May 2027 |
| Ubuntu | 24.04 | Comes with Jazzy. PX4's `ubuntu.sh` supports 22.04 and 24.04 |
| Gazebo | Harmonic | The version [ROS 2 Jazzy officially pairs with](https://github.com/gazebosim/ros_gz) |
| PX4 | v1.16.1 | What this team has validated. v1.17.0 is stable now, see below |
| px4_msgs | `release/1.16` | [Matched to the PX4 version.](https://github.com/PX4/px4_msgs) Mismatched messages fail in genuinely confusing ways |
| Micro XRCE-DDS Agent | v2.4.3 | PX4's docs say v2.4.2, but that version cannot build. Its superbuild pins Fast DDS branch `2.12.x`, which eProsima deleted ([issue #370](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/370)). v2.4.3 pins Fast DDS `2.14.x` and Fast CDR `2.2.x`, which both still exist, and keeps the 2.x client protocol PX4 v1.16.1 speaks. Do NOT jump to v3.0.1, agent 3.x will not talk to clients older than v3.0.0 |

All of those live in exactly one place, the `ARG` defaults at the top of the Dockerfile. Do not add a `build.args` block to `devcontainer.json` or to compose for them, because those values silently override the Dockerfile defaults and then the two drift apart without anybody noticing. `smoke-test.sh` checks for that specific mistake.

On upgrading PX4: v1.17.0 has been stable since May 2026 and `px4_msgs` has a matching `release/1.17` branch. The pin was left at v1.16.1 on purpose so the cross platform work could be tested against a known good baseline. Upgrading is three lines in the Dockerfile, but do it as its own commit so that if something breaks we know which change broke it.

## How one image manages to serve every OS:
The GUI transport gets picked when the container starts, not when the image is built. That is the trick that makes a single image work everywhere, and it lives in `scripts/start-display.sh`:

```
container starts
      |
      +-- is DISPLAY set AND can xdpyinfo actually reach it?
      |         |
      |        yes ------> native mode. Draw on the host X server.
      |         |
      |         no ------> vnc mode. Start Xvfb + x11vnc + noVNC,
      |                    serve the desktop on port 6080.
      |
      +-- write /tmp/drone-display.env
                |
                +-- every shell sources that from ~/.bashrc, so
                    VS Code's integrated terminal gets the right
                    DISPLAY too, even though it starts much later.
```

Testing with `xdpyinfo` rather than just checking whether `DISPLAY` is set matters more than it looks. `DISPLAY` gets inherited from the host through `containerEnv` even when no socket was mounted, so the variable existing proves nothing at all. Actually trying the connection is what makes the fallback trustworthy.

## X11 access control on Linux:
If GUI apps come back with `cannot open display`, this is why. There are two mechanisms out there and the setup handles both, so you should not have to care, but here is what is going on:

1. `SI:localuser` is the modern default on Wayland and XWayland sessions. The X server authorizes based on the connecting process's uid, so no cookie exists and none is needed. The container gets in because VS Code remaps its user to your uid automatically. Run `xhost` and if you see `SI:localuser:<you>`, this is you.
2. `MIT-MAGIC-COOKIE` is the traditional Xorg way, and it does need the cookie file. `scripts/doctor.sh` pulls one out to `/tmp/.docker.xauth` and points `XAUTH_FILE` at it.

Do NOT use `xhost +local:` to work around a display problem. It is the advice you will find on every forum post about this, and it turns off X access control for every local process on your machine rather than just this container. What is set up here is strictly better.

## Per machine settings live in `.devcontainer/.env`:
That file is gitignored, which is the entire point. It lets the repo stay byte for byte identical for everybody while still adapting to each laptop. `scripts/doctor.sh --write` generates it and `.env.example` lists every option.

The GPU setting is the one that actually matters:

```ini
DOCKER_RUNTIME=nvidia          # ONLY if `docker info` lists an nvidia runtime
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=all
```

That is done with `runtime:` rather than compose's `deploy.resources.devices` on purpose. With `devices`, a machine that has no Nvidia GPU fails to start the container at all, and that is most of the team's laptops plus every Mac. With `runtime:` those machines just get the default `runc` and carry on.

## Why `build/` looks empty on your host:
`compose.yaml` mounts named volumes over `build/`, `install/` and `log/`.

This is a performance thing for Mac and Windows, where bind mounts have to cross a VM boundary and small file writes are roughly an order of magnitude slower than native. A colcon build writes tens of thousands of small files, so keeping them on a container local volume is the difference between a build that takes a minute and one people give up on.

The side effect is that those directories look empty from your host file manager. They are gitignored build artifacts, so there is nothing in there you need to read or commit. If you want to wipe them:

```bash
docker volume rm drone-vip_colcon-build drone-vip_colcon-install drone-vip_colcon-log
```

## Skipping the 30 minute build:
CI builds a multi arch image and pushes it to GHCR. Once that has run on the repo, put this in `.devcontainer/.env`:

```ini
DRONE_IMAGE=ghcr.io/YOUR-ORG/DroneSwarming-dev:jazzy
```

Now reopening in a container pulls a finished image with PX4 already compiled instead of building one. The tag is a manifest list, so that same line is correct on Intel and on Apple Silicon, nobody has to think about it.

Without that line the image builds locally the first time you use it. That works fine, it just costs you 20 to 40 minutes once.

## When something breaks:
Run the smoke test first. It checks the toolchain, the version pins, ROS 2, a headless SITL launch and the display path, and it will tell you which one is unhappy:

```bash
./smoke-test.sh
```

| Symptom | What is going on |
|---|---|
| No window, nothing at :6080 | `cat /tmp/drone-display.log`, then `bash /opt/drone-vip/scripts/start-display.sh` |
| `cannot open display` on Linux | X access control, see above. Re-run `./scripts/doctor.sh --write` |
| Gazebo very slow | Check section 5 of the smoke test for the renderer. `llvmpipe` means software rendering. Use `HEADLESS=1` for heavy runs |
| 3D view is blank grey | Not a fault. PX4's default world is untextured grey and the camera starts far out. Scroll to zoom, and check the Entity Tree lists `x500_0` |
| Container will not start, mentions nvidia | You have `DOCKER_RUNTIME=nvidia` in `.env` without the NVIDIA Container Toolkit. Delete that line |
| colcon permission errors | Named volume ownership. Re-run `bash .devcontainer/scripts/post-create.sh` |
| Build dies, machine swaps | `JOBS=2 bash .devcontainer/scripts/build-workspace.sh` |
| Smoke test says the image is stale | Somebody changed a version pin. Rebuild the container |
| Windows WSLg config shows no window | Check WSLg itself first with `wsl -- xclock`. If that fails, use the Universal config |

## What is where:
```
.devcontainer/
  devcontainer.json              Universal (browser GUI), the default
  linux-x11/devcontainer.json    Linux native display
  windows-wslg/devcontainer.json Windows native display
  compose.yaml                   shared service definition
  compose.linux-x11.yaml         X socket + /dev/dri overlay
  compose.windows-wslg.yaml      WSLg overlay
  Dockerfile                     the multi arch image
  .env.example                   every per machine option
  scripts/
    start-display.sh             picks the GUI transport at runtime
    post-create.sh               first run setup
    build-workspace.sh           compiles anything not baked into the image
scripts/
  doctor.sh / doctor.ps1         host side environment check
```

One gotcha worth knowing: `scripts/` also gets copied into the image at `/opt/drone-vip/scripts/`, and `postStartCommand` runs the copy from there so the display still comes up even if the workspace mount is not ready yet. So if you edit `start-display.sh`, you need to rebuild the container for it to take effect, or run your edited copy directly to test it.
