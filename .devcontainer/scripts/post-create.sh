#!/usr/bin/env bash
# Runs once after the container is created.
#
# NOTE: no `set -u`. ROS 2 setup.bash references unset variables internally,
# and under `set -u` that terminates the shell even with `|| true` attached.
#
# This script ALWAYS exits 0. A failed build must not stop the container from
# coming up, or you cannot get a shell to debug the failure.
set -o pipefail

WS=/workspaces/DroneSwarming

echo "=============================================="
echo " UD Drone VIP dev container"
echo "=============================================="

source /opt/ros/jazzy/setup.bash 2>/dev/null

# ---------------------------------------------------------------------------
# Named-volume ownership.
#
# compose.yaml mounts named volumes over build/, install/, log/ and the ccache
# dir. Docker creates a named volume root-owned when the underlying image path
# does not exist, so without this the `ros` user cannot write to its own build
# directory and colcon fails with a permission error that looks like nonsense.
# ---------------------------------------------------------------------------
for d in "$WS/build" "$WS/install" "$WS/log" "$HOME/.cache/ccache"; do
    if [ -d "$d" ] && [ ! -w "$d" ]; then
        echo "  fixing ownership of $d"
        sudo chown -R "$(id -u):$(id -g)" "$d"
    fi
done

echo "ROS 2 distro : ${ROS_DISTRO:-unknown}"
echo "Gazebo       : $(gz sim --versions 2>/dev/null | head -1 || echo 'not found')"
echo "Architecture : $(uname -m)"
echo "PX4 source   : ${BUILT_PX4_VERSION:-unknown}"
echo "px4_msgs     : ${BUILT_PX4_MSGS_BRANCH:-unknown}"
echo "XRCE agent   : ${BUILT_XRCE_AGENT_VERSION:-unknown}"
echo

if bash "$(dirname "$0")/build-workspace.sh"; then
    echo
    echo "=============================================="
    echo " Setup complete."
    echo
    if [ "${DRONE_DISPLAY_ACTIVE:-}" = "native" ]; then
        echo " GUI: drawing on your host display."
    else
        echo " GUI: open http://localhost:6080 in your browser."
    fi
    echo " Verify everything with:  ./smoke-test.sh"
    echo "=============================================="
else
    echo
    echo "=============================================="
    echo " BUILD FAILED. The container is still usable."
    echo
    echo " Open a terminal here and retry to see the error:"
    echo "   bash .devcontainer/scripts/build-workspace.sh"
    echo
    echo " If the machine ran out of memory, use fewer jobs:"
    echo "   JOBS=2 bash .devcontainer/scripts/build-workspace.sh"
    echo "=============================================="
fi

exit 0
