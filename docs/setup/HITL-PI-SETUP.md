# HITL Pi Setup

This procedure assumes Ubuntu 24.04 on arm64, ROS 2 Jazzy, `colcon`, initialized `rosdep`, Git, CMake, Make, and a C++ compiler are installed.

Before building, add `noble-updates noble-backports` to the `Suites:` line in `/etc/apt/sources.list.d/ubuntu.sources`. Run `sudo apt update && sudo apt upgrade` after editing the file.

Run DroneSwarming build commands from the root of the checkout you want to build. Source `.env` in each build terminal:

```bash
source ./.env
source /opt/ros/jazzy/setup.bash
```

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
source /opt/ros/jazzy/setup.bash
source "$HOME/px4_ros2/install/setup.bash"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
  --dependency-types build --dependency-types build_export \
  --dependency-types buildtool --dependency-types buildtool_export \
  --dependency-types exec
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```
