#!/usr/bin/env bash
# Runs once after the container is created.
#
# NOTE: no `set -u`. ROS 2 setup.bash references unset variables internally,
# and under `set -u` that terminates the shell even with `|| true` attached.
#
# This script ALWAYS exits 0. A failed build must not stop the container from
# coming up, or you cannot get a shell to debug the failure.
set -o pipefail

echo "=============================================="
echo " UD Drone VIP dev container"
echo "=============================================="

source /opt/ros/jazzy/setup.bash 2>/dev/null

echo "ROS 2 distro : ${ROS_DISTRO:-unknown}"
echo "Gazebo       : $(gz sim --versions 2>/dev/null | head -1 || echo 'not found')"
echo "PX4 source   : ${BUILT_PX4_VERSION:-unknown}"
echo "px4_msgs     : ${BUILT_PX4_MSGS_BRANCH:-unknown}"
echo "XRCE agent   : ${BUILT_XRCE_AGENT_VERSION:-unknown}"
echo
echo "Now compiling PX4 and the ROS 2 workspace."
echo "First run takes 10 to 30 minutes. Progress appears below."
echo "=============================================="
echo

if bash "$(dirname "$0")/build-workspace.sh"; then
    echo
    echo "=============================================="
    echo " Setup complete. Run ./smoke-test.sh to verify."
    echo "=============================================="
else
    echo
    echo "=============================================="
    echo " BUILD FAILED. The container is still usable."
    echo
    echo " Open a terminal here and retry to see the error:"
    echo "   bash .devcontainer/build-workspace.sh"
    echo
    echo " If the machine ran out of memory, use fewer jobs:"
    echo "   JOBS=2 bash .devcontainer/build-workspace.sh"
    echo "=============================================="
fi

exit 0
