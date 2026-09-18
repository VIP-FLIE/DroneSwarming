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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

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
