# =============================================================================
# Environment for running the stack natively. Source it, do not execute it.
#
#   source /path/to/DroneSwarming/scripts/native/env.sh
#
# Put that line in ~/.bashrc to get it in every terminal.
#
# This sets up the same environment the dev container gives its shells, so a
# node behaves the same either way. Everything here respects a value you set
# yourself first, so `ROS_DOMAIN_ID=7 source env.sh` keeps 7.
# =============================================================================

if [ -n "$BASH_SOURCE" ] && [ "$BASH_SOURCE" = "$0" ]; then
    echo "env.sh has to be sourced, not run:  source $0" >&2
    exit 1
fi

_drone_env_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)

# --- ROS 2 ------------------------------------------------------------------
# Read before sourcing ROS. ROS 2 Jazzy ships an environment hook
# (share/ros_environment/environment/1.ros_discovery.dsv) that sets
# ROS_AUTOMATIC_DISCOVERY_RANGE to SUBNET when it is unset, so after the source
# below the variable is always set and a default applied there would never win.
_drone_discovery="${ROS_AUTOMATIC_DISCOVERY_RANGE:-}"

source /opt/ros/jazzy/setup.bash

# px4_msgs, then this repo's packages on top of it.
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$_drone_env_root/install/setup.bash" ] && source "$_drone_env_root/install/setup.bash"

# The middleware and the discovery scope both change what your node talks to,
# so they are set rather than left to whatever the machine defaults to.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# --- Which slot you are on --------------------------------------------------
# On your own machine this is 0, and slot 0 reproduces the single-user setup
# exactly: domain 0, instances 1-10, agent on 8888, station on 14550. Nothing
# below changes anything for you.
#
# On a shared machine, where several people run this at once, every person gets
# their own slot and these five variables are the whole of what keeps two
# simultaneous simulations apart. Set it before sourcing this file, normally
# from ~/.bashrc:
#
#     export DRONE_SLOT=1
#
# docs/LAB-SERVER.md has the slot table, and the list of exactly which fixed
# resources in the stack collide without this.
export DRONE_SLOT="${DRONE_SLOT:-0}"
case "$DRONE_SLOT" in
    [0-9]) ;;
    *) echo "env.sh: DRONE_SLOT must be a single digit 0-9, got '$DRONE_SLOT'." >&2
       echo "env.sh: falling back to 0, which is the single-user setup." >&2
       DRONE_SLOT=0 ;;
esac

# ROS 2. Fast DDS derives its ports from the domain id, so different domains
# are genuinely separate networks rather than a naming convention.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DRONE_SLOT}"

# Gazebo transport. This is the one that bites hardest without it. PX4's
# px4-rc.gzsim looks for an already-running world with
# `gz topic -l | grep -m 1 "^/world/.*/clock"`, takes the first answer, and
# spawns into it. On a shared machine that is somebody else's flight. A
# partition scopes every gz topic and service to you, so the search finds only
# your own worlds. Verified present in libgz-transport13, which is what
# Harmonic loads.
export GZ_PARTITION="${GZ_PARTITION:-${USER:-$(id -un)}}"

# PX4 SITL instance numbers. PX4 derives the MAVLink GCS port from the instance
# (18570 + instance, in ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink), so
# two people both flying instance 1 collide on UDP 18571. The second one's
# `mavlink start` cannot bind, that vehicle never gets a GCS link, and
# NAV_DLL_ACT 2 then refuses to arm it with the same "Arming denied" message
# you get when no station is running at all.
#
# Ten instances per slot. Slot 0 gets 1-10, slot 1 gets 11-20, and so on.
export PX4_INSTANCE_BASE="${PX4_INSTANCE_BASE:-$((DRONE_SLOT * 10))}"

# The PX4 to ROS 2 bridge. PX4 reads this in rcS and passes it to
# `uxrce_dds_client start -t udp -p`, so the agent has to listen on the same
# port. Start it with:  MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT
export PX4_UXRCE_DDS_PORT="${PX4_UXRCE_DDS_PORT:-$((8888 + DRONE_SLOT))}"

# Where drone-gcs listens for a vehicle that reaches out on its own, the way
# QGroundControl does. Read by drone-gcs as the default for --listen-port.
export DRONE_GCS_PORT="${DRONE_GCS_PORT:-$((14550 + DRONE_SLOT))}"

# Native ROS 2 discovers nodes across the whole subnet. In a lab where several
# people run this at once, that means your node subscribes to someone else's
# drone. The dev container never had the problem because Docker's network
# already isolated it. LOCALHOST restores that. Raise it to SUBNET when you
# deliberately want to fly across machines, and give everyone a different
# ROS_DOMAIN_ID when you do.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${_drone_discovery:-LOCALHOST}"
unset _drone_discovery

# --- PX4 and Gazebo ---------------------------------------------------------
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

# PX4 treats this as present-or-absent rather than 1-or-0, so any value at all
# makes it wait for a Gazebo server that nothing started.
unset PX4_GZ_STANDALONE

# Gazebo's GUI and rviz2 are Qt applications, and the container runs them on
# xcb. Set unconditionally because a Wayland desktop session exports
# QT_QPA_PLATFORM=wayland;xcb of its own accord, which is the case this is here
# to override. Export it again after sourcing this file to use something else.
export QT_QPA_PLATFORM=xcb

# --- PATH -------------------------------------------------------------------
# drone-sim and drone-gcs.
case ":$PATH:" in
    *":$_drone_env_root/scripts/native/bin:"*) ;;
    *) export PATH="$_drone_env_root/scripts/native/bin:$PATH" ;;
esac
# PX4's build tooling installs here.
case ":$PATH:" in
    *":$HOME/.local/bin:"*) ;;
    *) export PATH="$HOME/.local/bin:$PATH" ;;
esac

unset _drone_env_root
