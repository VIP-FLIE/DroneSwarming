# Dev Container Setup

One container, same toolchain for everyone, regardless of your laptop's OS or editor. Nothing gets installed on your machine except Docker and an editor of your choice.

---

## Pinned versions

Every version is fixed on purpose. Changing one means checking the others.

| Component | Version | Why this one |
|---|---|---|
| ROS 2 | Jazzy | Current LTS, supported to May 2029. Humble expires May 2027 |
| Ubuntu | 24.04 | Comes with Jazzy. PX4's `ubuntu.sh` supports 22.04 and 24.04 |
| Gazebo | Harmonic | The version [ROS 2 Jazzy officially pairs with](https://github.com/gazebosim/ros_gz). Installed via `ros-jazzy-ros-gz` |
| PX4 | v1.16.1 | Latest [stable release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.16.1) |
| px4_msgs | `release/1.16` | [Matched to the PX4 version.](https://github.com/PX4/px4_msgs) Mismatched messages fail in confusing ways |
| Micro XRCE-DDS Agent | **v2.4.3** | PX4's docs say v2.4.2, but that version cannot build. Its superbuild pins Fast DDS branch `2.12.x`, which eProsima deleted ([issue #370](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/370)). v2.4.3 pins Fast DDS `2.14.x` and Fast CDR `2.2.x`, both of which exist, and keeps the 2.x client protocol PX4 v1.16.1 speaks. **Do not use v3.0.1**: agent 3.x is incompatible with clients older than v3.0.0 |

**All versions live in one place: the `ARG` defaults at the top of the Dockerfile.** Do not add a `build.args` block to `devcontainer.json`. Those values override the Dockerfile's `ARG` defaults, and the two drift apart silently. `smoke-test.sh` now checks for exactly this.

**Gazebo pinning matters.** PX4's docs note that on Ubuntu 24.04 the "latest installed release" of Gazebo is used by default, which could be Harmonic, Ionic, or Jetty. We install `gz-harmonic` explicitly so everyone runs the same thing.

**One known tension, disclosed.** PX4's ROS 2 User Guide still names ROS 2 Humble on Ubuntu 22.04 as its supported platform. Their `ubuntu.sh` setup script supports 24.04 and installs Gazebo Harmonic, and px4_msgs CI covers Jazzy, so the combination works. If Jazzy causes trouble, switching is a two-line change in the Dockerfile: set the base image to `osrf/ros:humble-desktop-full` and `PX4_MSGS_BRANCH` stays the same. That flexibility is the point of using a container.

---

## Setup by operating system

### Linux (easiest)

1. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and run the [post-install steps](https://docs.docker.com/engine/install/linux-postinstall/) so you can use Docker without `sudo`
2. Install [VS Code](https://code.visualstudio.com/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
3. Allow the container to open windows on your display:
   ```bash
   xhost +local:docker
   ```
4. **NVIDIA GPU owners:** install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), then pick the **GPU** config in step 5
5. Open the repo in VS Code, press `F1`, run **Dev Containers: Reopen in Container**. Choose "UD Drone VIP (NVIDIA GPU)" if you did step 4, otherwise the portable one

### Windows

1. Install [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install) with Ubuntu
2. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and enable the WSL 2 backend
3. Install [VS Code](https://code.visualstudio.com/) and the Dev Containers extension
4. **Clone the repo inside WSL**, not on the Windows filesystem. Something under `\\wsl$\` or `/home/you/` in the WSL terminal. Cloning onto `C:\` makes file access painfully slow
5. Open in VS Code from the WSL terminal with `code .`, then **Reopen in Container**

GUI windows work through WSLg with no extra setup. Gazebo Harmonic now supports GPU acceleration under WSLg, so the old `LIBGL_ALWAYS_SOFTWARE=1` workaround is no longer needed.

Known WSLg issues: some laptops with Intel integrated graphics report black windows, and corporate VPN software can break WSL networking.

### macOS

Works, with caveats. There is no GPU passthrough, so Gazebo runs on software rendering.

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/)
2. Install [XQuartz](https://www.xquartz.org/), then in XQuartz preferences under Security, enable **Allow connections from network clients**. Log out and back in
3. Before opening the container:
   ```bash
   xhost + 127.0.0.1
   ```
4. Open in VS Code and **Reopen in Container** (portable config)

**Try it before assuming it is too slow.** One drone in Gazebo on an M-series chip may be perfectly usable, which covers weeks 1 through 6 of the plan. If it is not, you have two options: run headless for development and use someone else's machine for visuals, or ask Dan for access to his box.

---

---

## Editors other than VS Code

The [Dev Container spec](https://containers.dev/) is an open standard, not a VS Code feature. Our config works with any tool that implements it.

### Zed

Supported since **Zed v0.218 (7 January 2026)**. See [Zed's dev container docs](https://zed.dev/docs/dev-containers) and the [announcement post](https://zed.dev/blog/dev-containers).

1. Install Docker and make sure `docker` is in your PATH
2. Open the repo in Zed. It prompts to open in the container
3. Or use the command palette: **Project: Open Remote**, then **Connect Dev Container**

Zed delegates the build to the official `devcontainer` CLI, so `runArgs`, `mounts`, `containerEnv`, `build.dockerfile`, and `postCreateCommand` all apply exactly as they do under VS Code. GUI apps like Gazebo work the same way, because the X11 socket is mounted by the container runtime and has nothing to do with which editor attached.

Known Zed limitations as of v0.218:

| Limitation | Impact on us |
|---|---|
| `forwardPorts` not supported (upstream CLI issue) | None. We do not use it |
| Zed extensions in `devcontainer.json` not yet honored | Minor. Install your Zed extensions normally |
| Editing `devcontainer.json` does not trigger a rebuild | Restart the container manually after changes |
| Only reads `.devcontainer/devcontainer.json` | The GPU config in `gpu/` may not be offered. Use the CLI method below to select it |

To use the GPU config from Zed, start the container with the CLI first, then attach:

```bash
devcontainer up --workspace-folder . --config .devcontainer/gpu/devcontainer.json
```

### Any editor, or no editor

The reference [`devcontainer` CLI](https://github.com/devcontainers/cli) works standalone:

```bash
npm install -g @devcontainers/cli

devcontainer up --workspace-folder .           # build and start
devcontainer exec --workspace-folder . bash    # get a shell inside
```

From there use Neovim, Emacs, or a plain terminal. To pick the GPU config, add `--config .devcontainer/gpu/devcontainer.json`.

### Other supported tools

Per the [spec's support list](https://containers.dev/supporting): VS Code, Visual Studio 2022, IntelliJ IDEA, Emacs via [devcontainer.el](https://github.com/emacs-devcontainer/devcontainer.el), plus GitHub Codespaces, DevPod, CodeSandbox, and Ona.

**Team position: use whatever editor you like.** The container is the shared thing, not the editor.

## First run

Two phases, deliberately separated.

**Phase 1, image build: roughly 5 to 10 minutes.** Downloads the base image, installs Gazebo and the XRCE agent, clones PX4 and px4_msgs. No compiling happens here.

**Phase 2, first container start: 10 to 30 minutes.** `postCreateCommand` runs `.devcontainer/build-workspace.sh`, which compiles PX4 SITL and the ROS 2 workspace. Progress prints live in the VS Code terminal with elapsed-time markers.

**Why compiling is not in the Dockerfile.** BuildKit caps each RUN step's output at 1 MiB. PX4's build blows past that, and hitting the cap terminates the build with exit code 7 while hiding the output that would explain why ([docker/buildx#484](https://github.com/docker/buildx/issues/484)). A failure also discards the entire image. Building at first start gives live output, no cap, and a retry that costs minutes instead of an hour:

```bash
bash .devcontainer/build-workspace.sh          # resumes, skips what is done
bash .devcontainer/build-workspace.sh --force  # rebuild everything
```

**Parallel jobs are capped by RAM, not core count.** PX4's C++ build is memory hungry, and an over-parallel build swaps itself to death in a way that looks exactly like a hang. The script allocates roughly one job per 2 GB and prints what it chose.

Once phase 2 finishes, verify the whole stack:

```bash
./smoke-test.sh
```

Twelve checks covering toolchain presence, version pinning, ROS 2 messages, and an actual headless SITL launch. All twelve should pass.

---

## Set your ROS_DOMAIN_ID

Ask Dan for your number. Edit `.devcontainer/devcontainer.json`:

```json
"ROS_DOMAIN_ID": "your number here"
```

Rebuild the container. If two people run simulations under the same ID on the same network, their nodes discover each other and behave inexplicably. [Background](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html).

---

## Fly a drone

Three terminals inside the container.

```bash
# Terminal 1: the PX4 <-> ROS 2 bridge
MicroXRCEAgent udp4 -p 8888

# Terminal 2: the simulator
cd ~/PX4-Autopilot
make px4_sitl gz_x500
# add HEADLESS=1 in front of make to skip the 3D window

# Terminal 3: confirm ROS 2 sees the drone
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_local_position
```

In the PX4 console (terminal 2):

```
commander takeoff
commander land
```

If that works, your environment is finished and you can start on the tutorials.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `cannot open display` or no Gazebo window | Linux: `xhost +local:docker`. macOS: `xhost + 127.0.0.1` with XQuartz running. Windows: should be automatic, check WSLg is working with `wsl --update` |
| Gazebo window is black | Usually a graphics driver problem. Try `LIBGL_ALWAYS_SOFTWARE=1 gz sim` to confirm it renders at all, then report your GPU to Dan |
| **Wayland desktop** (Pop!_OS, Fedora, recent Ubuntu) | Already handled. `QT_QPA_PLATFORM=xcb` is set in `devcontainer.json`, which forces Qt through XWayland. Gazebo's own [troubleshooting page](https://gazebosim.org/docs/harmonic/troubleshooting/) documents an Ogre and Qt interaction that breaks native Wayland, so this is the supported path. You still need `xhost +local:docker` |
| Build fails with `usermod: group 'render' does not exist` | Old Dockerfile. The base image has no `render` group. The current version creates any missing group before adding the user |
| Build fails inside `Tools/setup/ubuntu.sh` with a `usermod` usage error | Old Dockerfile. PX4's setup script runs `sudo usermod -aG dialout $USER`, and Docker leaves `USER` unset. The current version sets `ENV USER` explicitly |
| Build fails at the XRCE agent with `invalid reference: 2.12.x` | Agent v2.4.2 is unbuildable: eProsima deleted the Fast DDS `2.12.x` branch it pins ([upstream issue #370](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/370)). You need v2.4.3. Check the top of the build log for `--build-arg XRCE_AGENT_VERSION=`. If it says v2.4.2, something is passing a stale `build.args`, which overrides the Dockerfile |
| Build fails downloading packages | Campus network or VPN interference. Try off the VPN |
| Everything is extremely slow on Windows | You cloned onto `C:\` instead of inside WSL. Re-clone in the WSL filesystem |
| `px4_msgs` not found | Run `source ~/ros2_ws/install/setup.bash`, which `.bashrc` should already do |
| Out of disk space | The image is large. `docker system prune -a` clears old layers |
| Build appears frozen with no output for a long time | If during the **image build**, you may be hitting BuildKit's 1 MiB log cap. The current Dockerfile does no compiling, so this should not happen. If during **first container start**, check the elapsed-time markers: the PX4 stage genuinely runs 10 to 30 minutes |
| `postCreateCommand` failed | The container still exists and `post-create.sh` always exits 0, so you get a shell. Run `bash .devcontainer/build-workspace.sh` to see the real error and retry |
| A script prints its header then dies with exit 1 | Classic `set -u` bug. ROS 2 `setup.bash` references unset variables internally, and under `set -u` that terminates the shell **even with `\|\| true` attached**. Neither of our scripts uses `set -u`. If you add a script, do not use it, or guard the source with `set +u` |
| Machine becomes unresponsive during the build | Too many parallel jobs for available RAM. The script caps at one job per 2 GB, but you can force lower: `JOBS=2 bash .devcontainer/build-workspace.sh --force` |
| Container starts as root | You changed `remoteUser`. Set it back to `ros` |

---

## Structure

```
.devcontainer/
  Dockerfile            the image. All version pins live here
  devcontainer.json     portable config. Works everywhere, no GPU
  gpu/devcontainer.json NVIDIA config. Linux and WSL2 with the container toolkit
  post-create.sh        prints versions on first start
  README.md             this file
smoke-test.sh           verifies the stack
```

---

## Later: prebuilt images

Right now everyone builds the image locally, which takes 20 to 40 minutes once. When the team grows or the image changes often, build it once in GitHub Actions, push to [GitHub Container Registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry), and change `devcontainer.json` from `"build"` to `"image": "ghcr.io/<org>/<repo>:latest"`. Students then pull a finished image instead of compiling PX4.

Not worth setting up until the current approach hurts.
