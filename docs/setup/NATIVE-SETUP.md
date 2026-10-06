# Setup

ROS 2 Jazzy, Gazebo Harmonic and PX4 install on Ubuntu 24.04 (noble) and derivatives on the same base: Pop!_OS 24.04, Linux Mint 22, elementary OS 8, Zorin OS 18. Other distros need a VM or WSL2.

## Requirements

| | |
|---|---|
| OS | Ubuntu 24.04 or noble derivative |
| Architecture | x86_64 or arm64 |
| Disk | 20 GB free in home directory |
| RAM | 8 GB minimum, 16 GB comfortable |
| Install time | ~20 minutes, mostly downloads |

## Compatibility

Status: **Verified** (built, smoke test passed), **Untested** (should work, test and open an Issue), **No** (does not work).

| Host OS | Arch | Status |
|---|---|---|
| Pop!_OS 24.04 | x86_64 | Verified, NVIDIA RTX 3070 |
| Ubuntu 24.04 | x86_64 | Untested |
| Linux Mint 22, elementary OS 8, Zorin OS 18 | x86_64 | Untested |
| Ubuntu 24.04 | arm64 | Untested |
| Ubuntu 22.04 or older, Debian, Fedora, Arch | any | No |
| macOS, Ubuntu 24.04 in UTM VM | arm64 | Untested |
| Windows 11, Ubuntu 24.04 under WSL2 | x86_64 | Untested |

### GPU

| GPU | Rendering | Status |
|---|---|---|
| NVIDIA, Linux | Hardware | Verified on RTX 3070 |
| AMD, Linux | Hardware | Untested |
| Intel integrated, Linux | Hardware | Untested |
| None | Software | Verified |
| Any GPU in a UTM VM | Software | macOS does not pass a GPU to a VM |

Run `./scripts/smoke-test.sh`. Section 6 prints the renderer. `llvmpipe` means software.

### Software rendering cost

| | Software | NVIDIA GPU |
|---|---|---|
| Flight dynamics | identical | identical |
| RTF, 1 vehicle, GUI open | 0.99 | 0.99 |
| Gazebo GUI framerate | low | normal |
| Camera at 30 Hz configured | 11.5 Hz | 28.5 Hz |

Headless multi-vehicle does not render. 8 vehicles held RTF 0.999, including when limited to 4 CPU cores. PX4 runs in lockstep: physics does not change with hardware. Camera sensor rate does. Vision work needs a GPU. See [../simulation/SIMULATION-FIDELITY.md](../simulation/SIMULATION-FIDELITY.md).

### macOS

UTM runs Ubuntu 24.04 arm64 on Apple Silicon.

1. Install UTM from <https://mac.getutm.app/>.
2. Download the Ubuntu 24.04 arm64 image from <https://ubuntu.com/download>.
3. Create a VM with 8 GB RAM and 40 GB disk.
4. Install Ubuntu inside the VM.
5. Follow the install section below inside the VM.

Gazebo opens on the VM's desktop. Rendering is software (`llvmpipe`). Flight behavior is unaffected. Camera sensor rate is lower.

### Windows

Run Ubuntu 24.04 under WSL2. Install the distribution, then follow the install section below inside it. Untested. Open an Issue with smoke test output.

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

Version numbers come from `scripts/versions.env`.

PX4 is built with `--no-nuttx`: SITL only, no Pixhawk firmware.

## Install

```bash
git clone https://github.com/VIP-FLIE/DroneSwarming.git
cd DroneSwarming
./scripts/install.sh
```

Asks for sudo password for apt steps. Times on a 6-core laptop with nothing pre-installed:

| Step | Time |
|---|---|
| apt packages (ROS 2, Gazebo) | 5 min |
| PX4 clone, 1.7 GB | 4 min |
| Micro XRCE-DDS Agent build | 3 min |
| PX4 SITL compile | 2 min |
| `px4_msgs` | 5 min |
| This repo's packages | < 1 min |

Safe to re-run. Each step checks whether done and skips.

| Flag | Effect |
|---|---|
| `--rebuild-px4` | Recompile PX4 SITL |
| `--rebuild-msgs` | Rebuild `px4_msgs` |
| `--rebuild-agent` | Rebuild XRCE-DDS Agent |
| `--skip-apt` | Skip all apt steps |
| `JOBS=n` | Cap build parallelism. Default: one job per 2 GB RAM |

## Shell setup

```bash
echo "source $PWD/scripts/env.sh" >> ~/.bashrc
```

Open a new terminal. Every terminal running the stack needs this. It sources ROS 2, `px4_msgs`, this repo's packages, puts `drone-sim` and `drone-gcs` on `PATH`, and sets the environment below.

## Verify

```bash
./scripts/smoke-test.sh
```

Output should end with `Stack is good.` Takes a few minutes (section 5 launches PX4 headless and commands a takeoff). Sections 1-4 check the toolchain, versions, Gazebo installation, and ROS 2. Section 6 names the GL renderer.

## Fly

Four terminals:

```bash
MicroXRCEAgent udp4 -p 8888     # 1. PX4 to ROS 2 bridge
drone-gcs                       # 2. ground control station link
drone-sim                       # 3. simulator, gives a pxh> prompt
ros2 topic list | grep fmu      # 4. your side
```

Gazebo opens a window. Terminal 4 printing nothing means the agent is not connected.

At the `pxh>` prompt:

```
pxh> commander takeoff
pxh> commander land
```

`drone-gcs` is required. PX4's x500 airframe sets `NAV_DLL_ACT 2` (return on datalink loss). It will not arm without a GCS. Without it: `Arming denied: Resolve system health failures first`. Supply the link. `param set NAV_DLL_ACT 0` deletes the failsafe. See [../simulation/SIMULATION-FIDELITY.md](../simulation/SIMULATION-FIDELITY.md).

`drone-sim` runs `make px4_sitl gz_x500`. The same launch without the script:

```bash
cd ~/PX4-Autopilot
PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

Bare `gz sim` opens Gazebo's world chooser without a drone. For swarms and long runs: `HEADLESS=1 drone-sim`.

QoS rules, offboard mode and multi-vehicle launching: [../simulation/HOW-IT-WORKS.md](../simulation/HOW-IT-WORKS.md).

## Rebuilding your code

```bash
cd /path/to/DroneSwarming
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install`: editing a Python node takes effect on next `ros2 run`. Rebuild after adding files or changing `setup.py`.

## Environment

**Versions.** PX4, `px4_msgs` and the XRCE-DDS Agent are read from `scripts/versions.env` at install time. Smoke test section 2 compares installed versions against that file.

**Which Gazebo runs.** Two copies of Harmonic exist: the apt one under `/usr`, and the copy ROS 2 vendors under `/opt/ros/jazzy/opt/gz_*_vendor`. Sourcing ROS 2 points `GZ_CONFIG_PATH` and `LD_LIBRARY_PATH` at the vendored copy. Smoke test section 3 prints which one loads.

The installer and smoke test report different library paths. Both are correct. The installer checks with ROS 2 out of the environment (apt copy, `/usr/lib`). The smoke test runs with ROS 2 sourced (vendored copy, `/opt/ros/jazzy/opt`). Both are Harmonic.

A different Gazebo release (Ionic, Jetty) changes which library loads. PX4 calls bare `gz sim` and takes whatever responds. Its own check requires version ≥ 8.0.0. Smoke test section 1 fails when more than one version is installed.

**How PX4 was built.** The installer compiles PX4 with ROS 2 removed from the environment and `GZ_DISTRO=harmonic` set:

- Sourcing ROS puts its vendored Gazebo on `CMAKE_PREFIX_PATH` and `LD_LIBRARY_PATH`. Building with ROS out of the environment links PX4 against the apt Gazebo under `/usr`.
- `GZ_DISTRO=harmonic` narrows PX4's cmake search to Harmonic library versions. Without it, PX4 searches for Ionic versions first (`gz-transport14`, `gz-sim9`).

Rebuild PX4 with `./scripts/install.sh --rebuild-px4`. Smoke test section 3 reads the Gazebo transport library the `px4` binary loads.

**Network scope.** ROS 2 discovers nodes across the subnet. Two machines in the same room see each other's `/fmu/` topics. `env.sh` sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

ROS 2 Jazzy sets that variable to `SUBNET` when unset, from an environment hook in `ros_environment`. `env.sh` runs after `/opt/ros/jazzy/setup.bash`.

For multi-machine work: `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` and a different `ROS_DOMAIN_ID` per machine.

**Other variables `env.sh` sets:**

| Variable | Value | Reason |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | Fast DDS derives ports from it |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | Cyclone DDS is installed but not selected |
| `PX4_DIR` | `~/PX4-Autopilot` | Installer location |
| `GZ_SIM_RESOURCE_PATH` | PX4's models and worlds | Gazebo finds the x500 |
| `QT_QPA_PLATFORM` | `xcb` | Gazebo GUI and rviz2 run on xcb. Set unconditionally |
| `PX4_GZ_STANDALONE` | unset | PX4 reads it as present or absent. Any value makes it wait for a Gazebo server nothing started |

Every variable except `QT_QPA_PLATFORM` keeps a value you exported before sourcing `env.sh`.

## Problems

Run `./scripts/smoke-test.sh` first.

| Symptom | Cause | Fix |
|---|---|---|
| Installer stops on distro check | Not Ubuntu 24.04 or noble derivative | Use a VM or WSL2 |
| `ERROR: Gazebo simulation dependencies not found` during PX4 build | Gazebo not installed when PX4 was configured | `./scripts/install.sh --rebuild-px4` |
| Smoke test section 1 lists two Gazebo versions | Ionic or Jetty also installed | Remove the others, or accept PX4 takes the first listed |
| Smoke test section 3: PX4 not bound to `gz-transport13` | PX4 built with ROS 2 sourced, or against Ionic | `./scripts/install.sh --rebuild-px4` |
| `Arming denied: Resolve system health failures first` | No GCS link | Run `drone-gcs`. Not `NAV_DLL_ACT 0` |
| Gazebo shows demo worlds, no drone | Ran `gz sim` | `drone-sim` |
| No `/fmu/` topics | Agent not running, or started after PX4 | Start `MicroXRCEAgent`, restart PX4 |
| `MicroXRCEAgent: command not found` after install | `/usr/local/bin` not on `PATH`, or stale linker cache | `sudo ldconfig /usr/local/lib`, new terminal |
| Callback never fires, `ros2 topic echo` shows data | QoS mismatch | See [../simulation/HOW-IT-WORKS.md](../simulation/HOW-IT-WORKS.md) |
| `ros2 run py_single_square` not found | Overlay not sourced | `source install/setup.bash` or `source env.sh` |
| Drones you didn't launch appear on your topics | Subnet-wide discovery | `source scripts/env.sh` |
| PX4 build dies, machine swaps | Too many parallel jobs | `JOBS=2 ./scripts/install.sh --rebuild-px4` |
| Gazebo window blank or misplaced on Wayland | Qt platform plugin | `export QT_QPA_PLATFORM=xcb` (`env.sh` does this) |
| `Standalone PX4 launch, waiting for Gazebo` | `PX4_GZ_STANDALONE` set | `unset PX4_GZ_STANDALONE` |
| Gazebo at few fps | Software rendering | Smoke test section 6 names the renderer. Install GPU driver |
| `Failed to load system plugin [libGstCameraSystem.so]` in Gazebo output | gstreamer camera plugin not built (install doesn't add gstreamer dev packages) | Harmless. x500 doesn't use it |
| Smoke test reports PX4 tree as modified | Something other than the two removed NuttX submodule directories changed | `cd ~/PX4-Autopilot && git status --short`, `git checkout` |
| Another ROS package fails with numpy ABI error after install | PX4's requirements put newer numpy in `~/.local` | `pip uninstall --break-system-packages numpy` after PX4 build |

## Adding a compatibility row

If your setup is Untested, run `./scripts/smoke-test.sh` and open an Issue with the output and your OS, arch, and GPU.