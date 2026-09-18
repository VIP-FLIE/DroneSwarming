# Running natively

ROS 2 Jazzy, Gazebo Harmonic and PX4 installed directly on your machine, with
no container. Use this if you already run the ROS 2 stack natively or you want
the GUI on your own desktop and your own GPU.

The dev container in [SETUP.md](SETUP.md) stays supported and is the right
choice on Windows, macOS, and any Linux that is not Ubuntu 24.04. Both paths
install the same pinned versions and produce the same flight behavior. See
[Keeping the two identical](#keeping-the-two-identical).

## Requirements

| | |
|---|---|
| OS | Ubuntu 24.04 LTS (noble), or a derivative on the same base: Pop!_OS 24.04, Linux Mint 22, elementary OS 8 |
| Architecture | x86_64 or arm64 |
| Disk | 20 GB free in your home directory |
| RAM | 8 GB minimum, 16 GB comfortable |
| Install time | About 20 minutes on a machine with none of it installed, mostly downloads |

ROS 2 Jazzy and Gazebo Harmonic publish packages for noble only. On any other
distro, use the container.

## What gets installed

| Component | Version | Location | Source |
|---|---|---|---|
| ROS 2 Jazzy desktop | Jazzy | `/opt/ros/jazzy` | apt, packages.ros.org |
| Gazebo Harmonic | gz-sim 8 | `/usr` | apt, packages.osrfoundation.org |
| `ros_gz` and ROS's vendored Harmonic | Jazzy | `/opt/ros/jazzy/opt` | apt, packages.ros.org |
| Micro XRCE-DDS Agent | v2.4.3 | `/usr/local` | built from source |
| PX4 | v1.16.1 | `~/PX4-Autopilot` | git, built from source |
| `px4_msgs` | `release/1.16` | `~/ros2_ws` | git, built with colcon |
| This repo's packages | | `./build`, `./install` | built with colcon |

The version numbers come from the `ARG` defaults in
`.devcontainer/Dockerfile`, which is the only place this repo pins them. The
installer reads them from that file.

PX4 is set up with `--no-nuttx`, so the tree builds SITL only and cannot
compile firmware for a Pixhawk. This matches the container.

## Install

```bash
git clone https://github.com/VIP-FLIE/DroneSwarming.git
cd DroneSwarming
./scripts/native/install.sh
```

It asks for your sudo password for the apt steps. Measured on a 6 core laptop
with nothing installed beforehand:

| Step | Time |
|---|---|
| apt packages, including ROS 2 and Gazebo | 5 min |
| PX4 clone, 1.7 GB | 4 min |
| Micro XRCE-DDS Agent, built from source | 3 min |
| PX4 SITL compile | 2 min |
| `px4_msgs` | 5 min |
| This repo's packages | under 1 min |

The script is safe to re-run. Each step checks whether it is already done and
skips it, so an install interrupted partway through resumes where it stopped.

| Flag | Effect |
|---|---|
| `--rebuild-px4` | Recompile PX4 SITL |
| `--rebuild-msgs` | Rebuild `px4_msgs` |
| `--rebuild-agent` | Rebuild and reinstall the XRCE-DDS Agent |
| `--skip-apt` | Skip every apt step, for a machine where you manage packages yourself |
| `JOBS=n` | Cap build parallelism. The default is one job per 2 GB of RAM |

If you already have ROS 2 Jazzy or Gazebo Harmonic installed, the script
detects them and moves on.

## Set up your shell

```bash
echo "source $PWD/scripts/native/env.sh" >> ~/.bashrc
```

Open a new terminal after adding it. Every terminal that runs any part of the
stack needs this. It sources ROS 2, `px4_msgs` and this repo's packages, puts
`drone-sim` and `drone-gcs` on your `PATH`, and sets the environment listed
under [Keeping the two identical](#keeping-the-two-identical).

## Verify

```bash
./scripts/native/smoke-test.sh
```

You want `Stack is good.` The run takes a few minutes because section 5
launches PX4 headless and commands a takeoff.

Sections 1 through 4 check the toolchain, the versions against the Dockerfile
pins, which Gazebo installation is in play, and ROS 2. Section 6 names the GL
renderer your machine actually got.

## Fly

Four terminals:

```bash
MicroXRCEAgent udp4 -p 8888     # 1. PX4 to ROS 2 bridge
drone-gcs                       # 2. ground control station link
drone-sim                       # 3. simulator, gives you a pxh> prompt
ros2 topic list | grep fmu      # 4. your side
```

Gazebo opens a window on your desktop. Terminal 4 printing nothing means the
agent is not connected.

At the `pxh>` prompt:

```
pxh> commander takeoff
pxh> commander land
```

`drone-gcs` is required. PX4's x500 airframe sets `NAV_DLL_ACT 2`, return on
datalink loss, so it will not arm without a ground control station. Without it
you get `Arming denied: Resolve system health failures first`. Supply the link
rather than setting `param set NAV_DLL_ACT 0`, which deletes the failsafe. See
[SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md).

`drone-sim` wraps `make px4_sitl gz_x500`. Bare `gz sim` opens Gazebo's world
chooser and does not start the drone. For swarms and long runs use
`HEADLESS=1 drone-sim`.

Writing nodes, the QoS rules, offboard mode and multi-vehicle launching are all
in [QUICKSTART.md](../QUICKSTART.md), which applies here as written. See
[Reading the other docs](#reading-the-other-docs) for the handful of
container-only commands in it.

## Rebuilding your code

```bash
cd /path/to/DroneSwarming
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means editing a Python node takes effect on the next
`ros2 run`. Rebuild after adding files or changing `setup.py`.

## Keeping the two identical

Four things decide whether a native machine simulates what the container
simulates. The installer sets all four, and `smoke-test.sh` checks them.

**Versions.** PX4, `px4_msgs` and the XRCE-DDS Agent are read from
`.devcontainer/Dockerfile` at install time. Section 2 of the native smoke test
compares what is installed against that file, so a pin bump that has not
reached your machine reports itself.

**Which Gazebo runs.** Two complete copies of Harmonic end up on a native
machine: the apt one under `/usr`, and the copy ROS 2 vendors under
`/opt/ros/jazzy/opt/gz_*_vendor`, which `ros_gz` depends on. Sourcing ROS 2
points `GZ_CONFIG_PATH` and `LD_LIBRARY_PATH` at the vendored copy, and that is
what decides which libraries load. The container installs the same two packages
and sources ROS 2 the same way, so it loads the same ones. Section 3 of the
smoke test prints which one is in play.

The installer and the smoke test report different paths for that library, and
both are correct. The installer checks right after the build, with ROS 2 out of
the environment, so it names the apt copy under `/usr/lib`. The smoke test runs
with ROS 2 sourced, the way you actually run the simulator, so it names the
vendored copy under `/opt/ros/jazzy/opt`. Both are Harmonic.

A different Gazebo release on the machine, such as Ionic or Jetty, does change
the answer. PX4 calls a bare `gz sim` and takes whatever responds, and its own
check only requires version 8.0.0 or newer, so Ionic passes it. Section 1 of
the smoke test fails when more than one version is installed.

**How PX4 was built.** The installer compiles PX4 with ROS 2 removed from the
environment and `GZ_DISTRO=harmonic` set:

- The container builds PX4 in a Docker `RUN` step, which never sources ROS, so
  it links against the apt Gazebo. Stripping ROS from the build environment
  reproduces that. A build with ROS sourced links against the vendored copy
  instead.
- `GZ_DISTRO=harmonic` narrows PX4's cmake search to the Harmonic library
  versions. Without it PX4 searches for the Ionic versions first
  (`gz-transport14`, `gz-sim9`) and uses them when they are installed.

Rebuild PX4 with `./scripts/native/install.sh --rebuild-px4` rather than a bare
`make` in `~/PX4-Autopilot`, so those two conditions hold. Section 3 of the
smoke test reads the Gazebo transport library the `px4` binary loads.

**Network scope.** Native ROS 2 discovers nodes across your whole subnet. Two
people running this in the same room see each other's `/fmu/` topics, and a
node subscribes to whichever vehicle answers first. Docker's network already
isolated the container from that. `env.sh` sets
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, which restores the isolation.

ROS 2 Jazzy sets that variable to `SUBNET` itself whenever it is unset, from an
environment hook in `ros_environment`. Anything that changes it has to run
after `/opt/ros/jazzy/setup.bash`, which is what `env.sh` does.

For deliberate multi-machine work, set `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
and give every machine a different `ROS_DOMAIN_ID`.

The rest of the environment `env.sh` sets:

| Variable | Value | Reason |
|---|---|---|
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | The container sets it explicitly. Cyclone DDS is installed but not selected |
| `ROS_DOMAIN_ID` | `0` | Same as the container |
| `PX4_DIR` | `~/PX4-Autopilot` | Where the installer put PX4 |
| `GZ_SIM_RESOURCE_PATH` | PX4's models and worlds | Gazebo finds the x500 |
| `QT_QPA_PLATFORM` | `xcb` | The container runs Gazebo's GUI and rviz2 on xcb. A Wayland session exports `wayland;xcb` on its own, so this one is set unconditionally |
| `PX4_GZ_STANDALONE` | unset | PX4 reads it as present or absent, so any value makes it wait for a Gazebo server nothing started |

Every variable above except `QT_QPA_PLATFORM` keeps a value you exported before
sourcing `env.sh`.

## Differences from the container

| | Container | Native |
|---|---|---|
| Gazebo GUI | Browser at `localhost:6080` or a VNC client on 5901 | A window on your desktop |
| Rendering | VirtualGL through the GPU, NVIDIA only, needs the Container Toolkit | Your GL driver directly, any vendor |
| Camera sensors | Measured at 28.5 Hz of a configured 30 Hz on NVIDIA, 11.5 Hz on software rendering | Same GPU with no VirtualGL in the path. Not yet measured |
| Repo path | `/workspaces/DroneSwarming` | Wherever you cloned it |
| colcon output | Named Docker volumes | `./build`, `./install`, `./log` on disk |
| ROS 2 discovery | Isolated by Docker's network | `env.sh` limits it to localhost |
| Version upgrade | Rebuild the image | Re-run `install.sh` |
| Firmware builds | Not possible, `--no-nuttx` | Not possible, `--no-nuttx` |

The two do not share build output, so switching between them costs nothing.
The container mounts its own volumes over `build/`, `install/` and `log/`, which
hides the native output rather than overwriting it.

## Reading the other docs

[QUICKSTART.md](../QUICKSTART.md), [HOW-IT-WORKS.md](HOW-IT-WORKS.md),
[NEW-PACKAGE.md](NEW-PACKAGE.md) and
[SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md) apply to both setups and call
out the places where the two differ. [SETUP.md](SETUP.md) is the dev container's
own installation guide, and this document is its native counterpart.

Two commands from the container's guide have native equivalents:

| Dev container | Native |
|---|---|
| `./smoke-test.sh` | `./scripts/native/smoke-test.sh` |
| `bash .devcontainer/scripts/build-workspace.sh` | `./scripts/native/install.sh` |

Anything naming `.devcontainer/.env`, `drone-gui`, or
`Dev Containers: Rebuild Container` is container-only and has no native
equivalent.

## Problems

Run `./scripts/native/smoke-test.sh` first. It names the broken piece.

| Symptom | Cause | Fix |
|---|---|---|
| Installer stops on the distro check | Not Ubuntu 24.04 or a noble derivative | Use the container, [SETUP.md](SETUP.md) |
| `ERROR: Gazebo simulation dependencies not found` during the PX4 build | Gazebo was not installed when PX4 was configured | `./scripts/native/install.sh --rebuild-px4` |
| Smoke test section 1 lists two Gazebo versions | Ionic or Jetty is also installed | Remove the others, or accept that PX4 takes the first one listed |
| Smoke test section 3 says PX4 is not bound to `gz-transport13` | PX4 was built with ROS 2 sourced, or against Ionic | `./scripts/native/install.sh --rebuild-px4` |
| `Arming denied: Resolve system health failures first` | No GCS link | Run `drone-gcs`. Not `NAV_DLL_ACT 0` |
| Gazebo shows demo worlds, no drone | You ran `gz sim` | `drone-sim` |
| No `/fmu/` topics | Agent not running, or started after PX4 | Start `MicroXRCEAgent`, restart PX4 |
| `MicroXRCEAgent: command not found` right after install | `/usr/local/bin` not on `PATH`, or the linker cache is stale | `sudo ldconfig /usr/local/lib`, then open a new terminal |
| Callback never fires but `ros2 topic echo` shows data | QoS mismatch | Use `PX4_QOS`, see QUICKSTART |
| `ros2 run drone_vip` not found | Overlay not sourced | `source install/setup.bash`, or source `env.sh` |
| Drones you did not launch appear on your topics | Subnet-wide discovery | `source scripts/native/env.sh` |
| `colcon build` fails with paths under `/workspaces` | Build output left from a dev container run | `rm -rf build install log`, then rebuild |
| PX4 build dies and the machine swaps | Too many parallel jobs | `JOBS=2 ./scripts/native/install.sh --rebuild-px4` |
| Gazebo window is blank or misplaced on Wayland | Qt platform plugin | `export QT_QPA_PLATFORM=xcb`, which `env.sh` does |
| `Standalone PX4 launch, waiting for Gazebo` | `PX4_GZ_STANDALONE` is set | `unset PX4_GZ_STANDALONE` |
| Gazebo at a few frames per second | Software rendering | Smoke test section 6 names the renderer. Install your GPU driver |
| `Failed to load system plugin [libGstCameraSystem.so]` in Gazebo's output | PX4's gstreamer camera plugin is not built, because neither setup installs the gstreamer development packages | Harmless. The x500 does not use it, and the dev container behaves the same way |
| Smoke test reports the PX4 tree as modified | Something other than the two removed NuttX submodule directories changed | `cd ~/PX4-Autopilot && git status --short`, then `git checkout` whatever it names |
| Another ROS package on this machine fails with a numpy ABI error after installing | PX4's requirements put a newer numpy in `~/.local`, which takes precedence over the apt one | `pip uninstall --break-system-packages numpy` once PX4 is built |

Still stuck: open an Issue with the smoke test output, your distro, and your
GPU.
