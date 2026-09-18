# Setup

ROS 2 Jazzy, Gazebo Harmonic and PX4 run in a container. You install Docker and
VS Code. Nothing else, on any OS.

This is the dev container guide. To install the same stack directly on Ubuntu
24.04 instead, use [NATIVE-SETUP.md](NATIVE-SETUP.md).

Check [COMPATIBILITY.md](../COMPATIBILITY.md) for your machine first.
Needs 15 GB disk and 8 GB RAM. The first build takes 20 to 40 minutes once.

## 1. Install

All platforms: [VS Code](https://code.visualstudio.com/) plus the
[Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

**Windows.** [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/),
keep the WSL 2 backend checked.
- Installer complains about WSL: run `wsl --install` in an Administrator
  PowerShell, reboot, retry.
- Installer complains about virtualization: enable Intel VT-x / AMD-V / SVM in
  your BIOS.
- NVIDIA card: recent Game Ready or Studio driver, then enable GPU support in
  Docker Desktop under Settings > Resources. Do not install a Linux NVIDIA
  driver inside WSL, it breaks it.

**macOS.** [Docker Desktop](https://docs.docker.com/desktop/install/mac-install/)
for your chip. Set Settings > Resources > Memory to 8 GB or more. Too little
RAM looks like a hung build. No GPU passthrough exists on macOS.

**Linux.** [Docker Engine](https://docs.docker.com/engine/install/) plus the
[post-install steps](https://docs.docker.com/engine/install/linux-postinstall/)
so `docker` works without `sudo`. Log out and back in. NVIDIA card: install the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

You do not need XQuartz, VcXsrv, Xming, WSLg, or any X server.

## 2. Clone and check

```bash
git clone https://github.com/VIP-FLIE/DroneSwarming.git
cd DroneSwarming
./scripts/doctor.sh --write      # Linux and macOS
.\scripts\doctor.ps1 -Write      # Windows PowerShell
```

The doctor checks Docker is healthy and enables your GPU if you have one. It is
optional. If PowerShell blocks it, use
`powershell -ExecutionPolicy Bypass -File .\scripts\doctor.ps1 -Write`.

## 3. Open the container

In VS Code: `F1`, then `Dev Containers: Reopen in Container`.

The first open builds the image, which takes 20 to 40 minutes. It is compiling
PX4 during that time. It has not frozen.

To skip the build, put a published image in `.devcontainer/.env` before this
step:

```ini
DRONE_IMAGE=ghcr.io/vip-flie/droneswarming-dev:jazzy
```

Then verify:

```bash
./smoke-test.sh
```

You want `Stack is good.`

## 4. The GUI

Gazebo does not open a window on your desktop. The container runs a desktop of
its own, and you connect to it from your machine. There are two ways to
connect. They show the same desktop, so use whichever you prefer. If you have a
GPU, VirtualGL renders on it either way.

### Option A: your browser

Open <http://localhost:6080>. Nothing to install.

VS Code usually forwards the port for you. The Ports tab has it otherwise. If
the page is blank, run `drone-gui` in the container. It reports each piece of
the desktop and restarts anything that is stopped.

### Option B: a VNC client

This puts the desktop in a normal application window instead of a browser tab.
The container publishes it on port **5901**. Install a client once, then
connect to `localhost:5901`.

**Linux**

```bash
sudo apt install tigervnc-viewer
xtigervncviewer localhost:5901
```

**Windows**

Download `vncviewer64` from
[TigerVNC releases](https://github.com/TigerVNC/tigervnc/releases), or install
[RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/). Run it
and enter `localhost:5901`.

**macOS**

A client is already installed, but macOS Screen Sharing refuses to connect to a
desktop that has no password, so set one first. Put this in
`.devcontainer/.env`:

```ini
DRONE_VNC_PASSWORD=whateveryouwant
```

Apply it with `F1`, then `Dev Containers: Rebuild Container`. The container
reads `.env` when it starts, so the password does nothing until you do this.
Then open Terminal and run:

```bash
open vnc://localhost:5901
```

You can also use Finder, `Go > Connect to Server`, and type
`vnc://localhost:5901`.

The password only protects the desktop inside your own machine. Ports 6080 and
5901 are bound to `127.0.0.1`, so nobody else on your network can reach either
one.

## 5. Fly

Four terminals, `Terminal > New Terminal`. You are already in the container.

```bash
MicroXRCEAgent udp4 -p 8888     # 1. PX4 to ROS 2 bridge
drone-gcs                       # 2. ground control station link
drone-sim                       # 3. simulator, gives you a pxh> prompt
ros2 topic list | grep fmu      # 4. your side
```

Then at the `pxh>` prompt:

```
pxh> commander takeoff
pxh> commander land
```

`drone-gcs` is required. PX4's x500 airframe arms its datalink-loss failsafe,
so it will not arm without a GCS, same as a real aircraft. Without it you get
`Arming denied: Resolve system health failures first`. Do not fix that with
`param set NAV_DLL_ACT 0`, which deletes the failsafe instead of supplying the
link. See [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md).

`drone-sim` wraps `make px4_sitl gz_x500`. Bare `gz sim` does not start the
drone, it opens Gazebo's world chooser.

[QUICKSTART.md](../QUICKSTART.md) covers writing your first node.

## Problems

Run `./smoke-test.sh` first. It names the broken piece.

| Symptom | Fix |
|---|---|
| `Arming denied: Resolve system health failures first` | Run `drone-gcs` |
| Gazebo shows a chooser of demo worlds | You ran `gz sim`. Run `drone-sim` |
| Blank grey 3D view | Normal. PX4's world is untextured grey, the camera starts far out. Scroll to zoom |
| Nothing at localhost:6080 | `drone-gui` |
| VNC client cannot connect to 5901 | Run `drone-gui`. The desktop has to be running before a client can attach |
| macOS Screen Sharing asks for a password and rejects a blank one | Set `DRONE_VNC_PASSWORD` in `.devcontainer/.env`, then `Dev Containers: Rebuild Container` |
| Port 5901 already taken on your machine | Set `DRONE_VNC_PORT` in `.devcontainer/.env` and connect to that port instead |
| Container will not start, `address already in use` | A stale VS Code window holds port 6080. Reopen VS Code, or set `DRONE_NOVNC_PORT` in `.devcontainer/.env` |
| Container will not start, mentions nvidia | Delete `DOCKER_RUNTIME=nvidia` from `.devcontainer/.env` |
| Doctor says Docker did not answer | Docker Desktop is still starting. Wait, rerun |
| Build looks frozen on Mac or Windows | Docker memory. Settings > Resources > 8 GB+ |
| Build dies, machine swaps | `JOBS=2 bash .devcontainer/scripts/build-workspace.sh` |
| Gazebo choppy | Check renderer in smoke test section 5. `llvmpipe` is software. Use `HEADLESS=1 make px4_sitl gz_x500` for heavy runs |
| Smoke test says image is stale | `Dev Containers: Rebuild Container` |

Still stuck: open an Issue with `./smoke-test.sh` output.
