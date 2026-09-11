#!/usr/bin/env bash
# Compiles anything that is not already baked into the image.
#
# With the published image (PREBUILD_PX4=1) PX4 and px4_msgs are already built,
# so this only compiles THIS repo's packages and finishes in seconds. If you
# built the image yourself with PREBUILD_PX4=0, this is where PX4 gets built.
#
# NOTE: no `set -u`. ROS 2 setup.bash references unset variables internally,
# and under `set -u` that terminates the shell even with `|| true` attached.
#
# Safe to re-run. Already-built targets are skipped unless you pass --force.
set -eo pipefail

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
REPO_WS="${REPO_WS:-/workspaces/DroneSwarming}"
START=$(date +%s)

# Parallel jobs capped by memory, not core count. PX4's C++ build is memory
# hungry and an over-parallel build swaps itself to death, which looks exactly
# like a hang. Roughly one job per 2 GB. Override with JOBS=n.
CPUS=$(nproc)
MEM_GB=$(awk '/MemTotal/{printf "%d", $2/1024/1024}' /proc/meminfo)
if [ -z "${JOBS:-}" ]; then
    JOBS=$(( MEM_GB / 2 ))
    [ "$JOBS" -lt 1 ] && JOBS=1
    [ "$JOBS" -gt "$CPUS" ] && JOBS=$CPUS
fi

stage() { printf '\n\033[1;36m==> %s\033[0m  (%ds elapsed)\n' "$1" "$(( $(date +%s) - START ))"; }

stage "Build environment"
echo "    CPUs available : $CPUS"
echo "    RAM            : ${MEM_GB} GB"
echo "    Parallel jobs  : $JOBS  (capped by RAM. override with JOBS=n)"
echo "    Architecture   : $(uname -m)"

# ROS setup files are not safe under `set -e` or `set -u`. Guard the source.
set +eu
source /opt/ros/jazzy/setup.bash
set -e

# ---------------------------------------------------------------------------
stage "PX4 SITL"
if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] && [ "$FORCE" -eq 0 ]; then
    echo "    Already built (baked into the image). Pass --force to rebuild."
else
    cd "$PX4_DIR"
    echo "    This is the long one. Expect 10 to 30 minutes."
    make px4_sitl_default -j"$JOBS"
    echo "    PX4 built."
fi

# ---------------------------------------------------------------------------
stage "px4_msgs"
if [ -f "$WS_DIR/install/setup.bash" ] && [ "$FORCE" -eq 0 ]; then
    echo "    Already built (baked into the image). Pass --force to rebuild."
else
    cd "$WS_DIR"
    colcon build --symlink-install --parallel-workers "$JOBS" \
        --event-handlers console_cohesion+
    echo "    px4_msgs built."
fi

# ---------------------------------------------------------------------------
stage "This repo's packages"
set +eu
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
set -e
if [ -d "$REPO_WS/src" ]; then
    cd "$REPO_WS"
    colcon build --symlink-install --parallel-workers "$JOBS" \
        --event-handlers console_cohesion+
    echo "    Repo packages built."
else
    echo "    No src/ directory at $REPO_WS, nothing to build."
fi

stage "Done"
TOTAL=$(( $(date +%s) - START ))
printf '    Total build time: %dm %ds\n\n' $(( TOTAL / 60 )) $(( TOTAL % 60 ))
echo "    Next: run ./smoke-test.sh to verify the whole stack."
echo
