# HITL Pi Setup

This procedure uses Ubuntu 24.04 on arm64 with ROS 2 Jazzy. It installs `rosdep`, builds the matching `px4_msgs` package, builds the Micro XRCE-DDS Agent, and builds DroneSwarming.

Before installing packages, add `noble-updates noble-backports` to the `Suites:` line in `/etc/apt/sources.list.d/ubuntu.sources`. Then update and upgrade the Ubuntu packages:

```bash
sudo apt update
sudo apt upgrade
```

The ROS 2 Jazzy installation, `colcon`, Git, CMake, and Make must be available before continuing. This guide does not install ROS 2 itself. Follow [these](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) docs from ros to install ROS2 Jazzy Base

## Install GCC

Install the GNU C and C++ compilers. The project uses `gcc` for C and `g++` for C++.

```bash
sudo apt install -y g++ python3-venv
gcc --version
g++ --version
```

## Install Ruff

Install the project-pinned Ruff version in a Python virtual environment. Run these commands from the DroneSwarming checkout root:

```bash
source ./.env
sudo apt install python3.12-venv
python3 -m venv "$HOME/.venvs/drone-swarming-ruff"
"$HOME/.venvs/drone-swarming-ruff/bin/python" -m pip install "ruff==${RUFF_VERSION}"
"$HOME/.venvs/drone-swarming-ruff/bin/ruff" --version
```

Activate the virtual environment to run `ruff` from `PATH`:

```bash
source "$HOME/.venvs/drone-swarming-ruff/bin/activate"
```

Run DroneSwarming build commands from the root of the checkout you want to build. Source `.env` in each build terminal:

```bash
source ./.env
export CC CXX
source /opt/ros/jazzy/setup.bash
```

The `.env` file sets `CC=gcc` and `CXX=g++`. Export both variables before building so CMake selects GCC.

## Install and initialize `rosdep`

`rosdep` reads ROS package manifests and installs missing system and ROS dependencies. Install the Ubuntu package, initialize the default rosdep sources list, and download the dependency rules:

```bash
sudo apt install -y python3-rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep --version
```

Run `sudo rosdep init` once on the Raspberry Pi. The file check skips initialization when the default sources list already exists. Run `rosdep update` as the regular user, without `sudo`.

`rosdep install` must run from a workspace directory and use that workspace's `src` directory. It reads each package's `package.xml`, skips packages found in source with `--ignore-src`, and installs the external dependencies listed by the manifests. It does not build ROS packages.

## Build `px4_msgs`

PX4 firmware and `px4_msgs` must use matching releases. The project pins both to v1.17.0 in `.env`. PX4 documents this requirement in the [v1.17 ROS 2 guide](https://docs.px4.io/v1.17/en/ros2/user_guide).

```bash
mkdir -p "$HOME/px4_ros2/src"
cd "$HOME/px4_ros2/src"
git clone --depth 1 --branch "$PX4_MSGS_VERSION" \
  https://github.com/PX4/px4_msgs.git
cd "$HOME/px4_ros2"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
  --dependency-types build --dependency-types build_export \
  --dependency-types buildtool --dependency-types buildtool_export \
  --dependency-types exec
colcon build --symlink-install
```

Source the PX4 message overlay before building DroneSwarming:

```bash
source "$HOME/px4_ros2/install/setup.bash"
```

## Build Micro XRCE-DDS Agent

The PX4 v1.17 client requires Micro XRCE-DDS Agent v2.4.3. PX4 documents the standalone source build in the [uXRCE-DDS guide](https://docs.px4.io/v1.17/en/middleware/uxrce_dds).

```bash
mkdir -p "$HOME/src"
cd "$HOME/src"
git clone --depth 1 --branch "$MICRO_XRCE_DDS_AGENT_VERSION" \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make -j2
sudo make install
sudo ldconfig /usr/local/lib/
```

## Build DroneSwarming nodes

Run these commands from the root of the DroneSwarming checkout:

```bash
source ./.env
export CC CXX
source /opt/ros/jazzy/setup.bash
source "$HOME/px4_ros2/install/setup.bash"
```

An existing CMake build directory keeps its original compiler selection. When switching a checkout from Clang to GCC, remove the DroneSwarming build and install directories before rebuilding:

```bash
rm -rf build install
```

Continue the build from the DroneSwarming root:

```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
  --dependency-types build --dependency-types build_export \
  --dependency-types buildtool --dependency-types buildtool_export \
  --dependency-types exec
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```
