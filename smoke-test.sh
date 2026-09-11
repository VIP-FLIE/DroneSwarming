#!/usr/bin/env bash
# Verifies the dev container stack end to end. Run inside the container:
#   ./smoke-test.sh
#
# NOTE: no `set -u`. ROS 2's setup.bash chain expands variables that may be
# unset (AMENT_TRACE_SETUP_FILES and friends). Under `set -u` that terminates
# the shell on the spot, and if the source is wrapped in 2>/dev/null you get a
# section header and nothing else. Same reason build-workspace.sh omits it.
set -o pipefail

PASS=0; FAIL=0; SKIP=0
ok()   { echo "  PASS  $1"; PASS=$((PASS+1)); }
no()   { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP  $1"; SKIP=$((SKIP+1)); }

# A sourced setup file can terminate this shell outright (an `exit` inside it,
# an unbound variable under `set -u`, a syntax error). When that happens you get
# a section header and no further output, which is impossible to diagnose. This
# guard makes that shape report itself instead of dying silently.
FINISHED=0
SECTION="startup"
finish_guard() {
    local rc=$?
    [ "$FINISHED" -eq 1 ] && return 0
    echo
    echo "  smoke-test.sh stopped early during: $SECTION  (exit $rc)"
    echo "  A sourced setup file most likely terminated the shell."
    echo "  Diagnose with:  bash -x ./smoke-test.sh 2>&1 | tail -40"
}
trap finish_guard EXIT

SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DOCKERFILE="$SELF_DIR/.devcontainer/Dockerfile"

# Reads an ARG default out of the Dockerfile, e.g. arg_default PX4_VERSION
arg_default() {
    [ -f "$DOCKERFILE" ] || return 1
    sed -n "s/^ARG $1=\(.*\)$/\1/p" "$DOCKERFILE" | head -1
}

# ---------------------------------------------------------------------------
SECTION="toolchain check"
echo "=== 1. Toolchain present ==="
echo "  info  architecture: $(uname -m)"

ROS_SETUP=/opt/ros/jazzy/setup.bash
if [ -f "$ROS_SETUP" ]; then
    # Errors captured rather than discarded, so a real problem is visible.
    if ! source "$ROS_SETUP" 2>/tmp/ros-source.err; then
        no "sourcing $ROS_SETUP failed:"
        sed 's/^/        /' /tmp/ros-source.err
    fi
else
    no "$ROS_SETUP missing. This is not a ROS 2 Jazzy image."
fi

[ "${ROS_DISTRO:-}" = "jazzy" ] && ok "ROS 2 Jazzy" || no "ROS 2 Jazzy (got '${ROS_DISTRO:-none}')"
command -v gz >/dev/null && ok "gz binary" || no "gz binary"
# NOTE: `cmd | grep -q` is unsafe under `set -o pipefail`. grep -q exits on the
# first match, the upstream command takes SIGPIPE and dies with 141, and
# pipefail reports that as the pipeline's status. A match reads as a failure.
# Capture first, match against the variable.
GZ_VERSIONS=$(gz sim --versions 2>/dev/null)
grep -q '^8\.' <<< "$GZ_VERSIONS" && ok "Gazebo Harmonic (gz-sim 8.x)" \
  || no "Gazebo Harmonic, found: $(head -1 <<< "$GZ_VERSIONS")"
command -v MicroXRCEAgent >/dev/null && ok "Micro XRCE-DDS Agent" || no "Micro XRCE-DDS Agent"
[ -d "$HOME/PX4-Autopilot" ] && ok "PX4 source" || no "PX4 source"

PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4"
if [ -x "$PX4_BIN" ]; then
    ok "PX4 SITL built"
else
    no "PX4 SITL not built. Run: bash .devcontainer/scripts/build-workspace.sh"
fi

# ---------------------------------------------------------------------------
SECTION="version pinning check"
echo "=== 2. Version pinning ==="
# Expected values come from the Dockerfile, so bumping a pin there does not
# leave this script asserting a stale number. Comparing the Dockerfile against
# the BUILT_* env baked into the image is also how a stale image gets caught:
# someone changes a pin, does not rebuild the container, and keeps running the
# old one.

if [ -f "$DOCKERFILE" ]; then
    WANT_PX4=$(arg_default PX4_VERSION)
    WANT_MSGS=$(arg_default PX4_MSGS_BRANCH)
    WANT_XRCE=$(arg_default XRCE_AGENT_VERSION)

    if [ "${BUILT_PX4_VERSION:-}" = "$WANT_PX4" ] \
    && [ "${BUILT_PX4_MSGS_BRANCH:-}" = "$WANT_MSGS" ] \
    && [ "${BUILT_XRCE_AGENT_VERSION:-}" = "$WANT_XRCE" ]; then
        ok "image matches Dockerfile pins"
    else
        no "image is stale. Rebuild the container (Dev Containers: Rebuild Container)."
        printf '        %-18s image=%-12s Dockerfile=%s\n' \
          PX4_VERSION      "${BUILT_PX4_VERSION:-unset}"      "$WANT_PX4"
        printf '        %-18s image=%-12s Dockerfile=%s\n' \
          PX4_MSGS_BRANCH  "${BUILT_PX4_MSGS_BRANCH:-unset}"  "$WANT_MSGS"
        printf '        %-18s image=%-12s Dockerfile=%s\n' \
          XRCE_AGENT_VER   "${BUILT_XRCE_AGENT_VERSION:-unset}" "$WANT_XRCE"
    fi
else
    skip "Dockerfile not found at $DOCKERFILE, comparing against image values only"
    WANT_PX4="${BUILT_PX4_VERSION:-}"
    WANT_MSGS="${BUILT_PX4_MSGS_BRANCH:-}"
    WANT_XRCE="${BUILT_XRCE_AGENT_VERSION:-}"
fi

PX4_TAG=$(cd "$HOME/PX4-Autopilot" 2>/dev/null && git describe --tags 2>/dev/null)
[ -n "$WANT_PX4" ] && [ "$PX4_TAG" = "$WANT_PX4" ] && ok "PX4 checkout at $WANT_PX4" \
  || no "PX4 tag is '${PX4_TAG:-unknown}', expected '${WANT_PX4:-unknown}'"

MSGS_BR=$(cd "$HOME/ros2_ws/src/px4_msgs" 2>/dev/null && git rev-parse --abbrev-ref HEAD 2>/dev/null)
[ -n "$WANT_MSGS" ] && [ "$MSGS_BR" = "$WANT_MSGS" ] && ok "px4_msgs on $WANT_MSGS" \
  || no "px4_msgs branch is '${MSGS_BR:-unknown}', expected '${WANT_MSGS:-unknown}'"

# ---------------------------------------------------------------------------
SECTION="ROS 2 check"
echo "=== 3. ROS 2 works ==="

WS_SETUP="$HOME/ros2_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    if ! source "$WS_SETUP" 2>/tmp/ws-source.err; then
        no "sourcing $WS_SETUP failed:"
        sed 's/^/        /' /tmp/ws-source.err
    fi
else
    echo "  note  $WS_SETUP missing, workspace not built yet"
fi

ROS_PKGS=$(ros2 pkg list 2>/dev/null)
grep -q '^px4_msgs$' <<< "$ROS_PKGS" && ok "px4_msgs importable" \
  || no "px4_msgs not built. Run: bash .devcontainer/scripts/build-workspace.sh"
ros2 interface show px4_msgs/msg/VehicleLocalPosition >/dev/null 2>&1 \
  && ok "VehicleLocalPosition message" || no "VehicleLocalPosition message"

# ---------------------------------------------------------------------------
SECTION="SITL launch test"
echo "=== 4. Headless SITL launches ==="
# Guarded on the binary existing. `make px4_sitl gz_x500` builds as well as
# launches, so without this guard a smoke test on an unbuilt tree would start
# a 20 minute compile and then have `timeout` kill it partway through.

if [ ! -x "$PX4_BIN" ]; then
    skip "PX4 not built, skipping launch test"
else
    SITL_BUDGET=${SITL_BUDGET:-180}   # seconds. Override for a slow machine.
    export PX4_DIR="$HOME/PX4-Autopilot"
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
    cd "$PX4_DIR" || exit 1
    # setsid puts the whole tree (make -> px4 -> gz sim) in its own process
    # group so cleanup can target exactly that group. The previous
    # `pkill -f px4` matched by name and would have killed a PX4 instance the
    # student had running in another terminal.
    # Do NOT set PX4_GZ_STANDALONE here. PX4 treats the variable as present or
    # absent rather than 1 or 0, so even PX4_GZ_STANDALONE=0 puts it in
    # standalone mode, where it waits forever for a gz-server that this script
    # never starts ("INFO [init] Standalone PX4 launch, waiting for Gazebo").
    # Leaving it unset is what makes PX4 launch Gazebo itself.
    : > /tmp/sitl.log
    if command -v setsid >/dev/null; then
        # </dev/null matters: PX4 drops into its interactive `pxh>` shell and
        # redraws that prompt forever when stdin is a tty. Left attached, a
        # 3 minute smoke test writes a multi-hundred-megabyte log file.
        setsid timeout "$SITL_BUDGET" env HEADLESS=1 \
            make px4_sitl gz_x500 </dev/null > /tmp/sitl.log 2>&1 &
        SITL_PID=$!
        SITL_GROUP=1
    else
        timeout "$SITL_BUDGET" env HEADLESS=1 \
            make px4_sitl gz_x500 </dev/null > /tmp/sitl.log 2>&1 &
        SITL_PID=$!
        SITL_GROUP=0
    fi

    # Poll rather than sleeping a flat 45s. Gazebo has to load world and model
    # assets on first run, which is slower on a cold cache and on a machine
    # doing software rendering, and a fixed sleep either wastes time or reports
    # a false failure.
    READY=0
    DIED=0
    SITL_T0=$(date +%s)
    for _ in $(seq 1 $(( SITL_BUDGET / 2 ))); do
        if grep -qiE "Ready for takeoff|INFO\s+\[commander\]" /tmp/sitl.log; then
            READY=1; break
        fi
        # Stop waiting the moment the process is gone, rather than sitting out
        # the rest of the budget for a sim that already crashed.
        if ! kill -0 "$SITL_PID" 2>/dev/null; then DIED=1; break; fi
        sleep 2
    done
    SITL_ELAPSED=$(( $(date +%s) - SITL_T0 ))

    if [ "$READY" -eq 1 ]; then
        ok "PX4 SITL reached ready state (${SITL_ELAPSED}s)"
    else
        if [ "$DIED" -eq 1 ]; then
            no "PX4 SITL exited after ${SITL_ELAPSED}s without becoming ready (full log: /tmp/sitl.log)"
        else
            no "PX4 SITL still not ready after ${SITL_BUDGET}s. Re-run with SITL_BUDGET=300 if this machine is slow. (full log: /tmp/sitl.log)"
        fi
        if grep -q "Standalone PX4 launch" /tmp/sitl.log; then
            echo "        PX4 is in standalone mode and is waiting for a Gazebo server"
            echo "        that nothing started. Unset PX4_GZ_STANDALONE in your shell."
        fi
        tail -20 /tmp/sitl.log | sed 's/^/        /'
    fi
    if [ "$SITL_GROUP" -eq 1 ]; then
        kill -TERM -- -"$SITL_PID" 2>/dev/null
        sleep 2
        kill -KILL -- -"$SITL_PID" 2>/dev/null
    else
        kill -TERM "$SITL_PID" 2>/dev/null
        sleep 2
        kill -KILL "$SITL_PID" 2>/dev/null
    fi
    wait "$SITL_PID" 2>/dev/null
fi

# ---------------------------------------------------------------------------
SECTION="display check"
echo "=== 5. GUI display path ==="
# The whole point of this project's container setup is that a GUI reaches you
# on any OS. This section reports WHICH transport is live and proves it works,
# so "Gazebo opens no window" is a diagnosis instead of a mystery.

[ -f /tmp/drone-display.env ] && source /tmp/drone-display.env

MODE="${DRONE_DISPLAY_ACTIVE:-unknown}"
case "$MODE" in
    native) echo "  info  mode: native (drawing on the host X server)" ;;
    vnc)    echo "  info  mode: vnc (browser desktop)" ;;
    *)      echo "  info  mode: unknown - start-display.sh may not have run" ;;
esac

if [ -z "${DISPLAY:-}" ]; then
    no "DISPLAY is not set"
elif timeout 10 xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    RES=$(xdpyinfo -display "$DISPLAY" 2>/dev/null | awk '/dimensions:/{print $2; exit}')
    ok "X server reachable on DISPLAY=$DISPLAY (${RES:-unknown})"
else
    no "cannot reach an X server on DISPLAY=$DISPLAY"
    echo "        Try:  bash /opt/drone-vip/scripts/start-display.sh"
    echo "        Log:  /tmp/drone-display.log"
fi

# Which renderer are we actually getting? This is the single most useful number
# for "why is it slow" questions, and it is invisible unless you ask.
if command -v glxinfo >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    RENDERER=$(timeout 20 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
    GLVER=$(timeout 20 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL version string: //p' | cut -d' ' -f1)
    if [ -n "$RENDERER" ]; then
        ok "OpenGL renderer: $RENDERER"
        case "$RENDERER" in
            *llvmpipe*|*softpipe*|*swrast*)
                echo "        Software rendering. Physics still runs at full speed;"
                echo "        only GUI framerate is affected. Expected on macOS and"
                echo "        on any machine without a GPU passed into the container." ;;
            *)
                echo "        Hardware accelerated." ;;
        esac
        # ogre2, Gazebo's render engine, requires OpenGL 3.3 or newer.
        if [ -n "$GLVER" ]; then
            if awk -v v="$GLVER" 'BEGIN{split(v,a,"."); exit !(a[1]>3 || (a[1]==3 && a[2]>=3))}'; then
                ok "OpenGL $GLVER (Gazebo ogre2 needs 3.3+)"
            else
                no "OpenGL $GLVER is below the 3.3 that Gazebo's ogre2 requires"
            fi
        fi
    else
        skip "could not query the GL renderer"
    fi
fi

if [ "$MODE" = "vnc" ]; then
    # Prove the browser really has something to connect to.
    if timeout 10 bash -c 'exec 3<>/dev/tcp/127.0.0.1/6080' 2>/dev/null; then
        ok "noVNC is listening on 6080  ->  http://localhost:6080"
    else
        no "noVNC is NOT listening on 6080 (see /tmp/drone-display.log)"
    fi
fi

# ---------------------------------------------------------------------------
FINISHED=1
TOTAL=$(( PASS + FAIL + SKIP ))
echo
echo "=============================================="
printf '  %d/%d passed, %d failed, %d skipped\n' "$PASS" "$TOTAL" "$FAIL" "$SKIP"
if [ "$FAIL" -eq 0 ] && [ "$SKIP" -eq 0 ]; then
    echo "  Stack is good."
elif [ "$FAIL" -eq 0 ]; then
    echo "  No failures, but some checks were skipped. See above."
else
    echo "  See failures above."
fi
echo "=============================================="
exit "$FAIL"
