#!/usr/bin/env bash
# Verifies the dev container stack end to end. Run inside the container:
#   ./smoke-test.sh
#
# NOTE: no `set -u`. ROS 2's setup.bash chain expands variables that may be
# unset, and under `set -u` that terminates the shell on the spot. Same reason
# build-workspace.sh omits it.
set -o pipefail

PASS=0; FAIL=0; SKIP=0
ok()   { echo "  PASS  $1"; PASS=$((PASS+1)); }
no()   { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP  $1"; SKIP=$((SKIP+1)); }

# A sourced setup file can terminate this shell outright (an `exit` inside it,
# an unbound variable under `set -u`, a syntax error). The guard makes that
# shape report itself instead of dying silently.
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
# `cmd | grep -q` is unsafe under `set -o pipefail`: grep -q exits on the first
# match and the upstream command takes SIGPIPE. Capture first.
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
grep -q '^px4_msgs$' <<< "$ROS_PKGS" \
  && ok "px4_msgs importable" \
  || no "px4_msgs not built. Run: bash .devcontainer/scripts/build-workspace.sh"


# The drone-gcs bookkeeping that flying a vehicle cannot exercise: whether
# MAVLink this pymavlink cannot decode gets miscounted as a vehicle, whether a
# lost link is noticed, and how long to wait for a swarm to assemble. Needs no
# simulator and takes under a second, so it runs before the flight test below.
GCS_TEST="$SELF_DIR/scripts/test-drone-gcs.py"
if [ -x "$GCS_TEST" ]; then
    if GCS_TEST_OUT=$("$GCS_TEST" 2>&1); then
        ok "drone-gcs logic ($(grep -c '  PASS  ' <<< "$GCS_TEST_OUT") checks)"
    else
        no "drone-gcs logic checks failed. Run $GCS_TEST to see them all:"
        grep -E '  FAIL  ' <<< "$GCS_TEST_OUT" | sed 's/^/        /'
    fi
else
    skip "scripts/test-drone-gcs.py not found, cannot check drone-gcs logic"
fi

# ---------------------------------------------------------------------------
SECTION="SITL launch test"
echo "=== 4. Headless SITL launches ==="
# Guarded on the binary existing: `make px4_sitl gz_x500` builds as well as
# launches, so without the guard an unbuilt tree would start a 20 minute
# compile and then have `timeout` kill it partway through.

if [ ! -x "$PX4_BIN" ]; then
    skip "PX4 not built, skipping launch test"
else
    SITL_BUDGET=${SITL_BUDGET:-180}   # seconds. Override for a slow machine.
    export PX4_DIR="$HOME/PX4-Autopilot"
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
    cd "$PX4_DIR" || exit 1
    # PX4_GZ_STANDALONE is read as present-or-absent, so even =0 makes PX4 wait
    # for a Gazebo server this script never starts. Leave it unset.
    : > /tmp/sitl.log

    # The x500 airframe arms NAV_DLL_ACT 2, so without a GCS the vehicle
    # refuses to arm. Supply the link rather than disabling the failsafe; see
    # docs/simulation/SIMULATION-FIDELITY.md.
    GCS_PID=""
    if command -v drone-gcs >/dev/null 2>&1; then
        drone-gcs > /tmp/smoke-gcs.log 2>&1 &
        GCS_PID=$!
    fi

    # PX4 gets a FIFO as stdin instead of /dev/null so the test can command a
    # takeoff. A background holder keeps the write end open so PX4 never sees
    # EOF. setsid puts make, px4 and gz in their own process group so cleanup
    # targets exactly that group, not a PX4 a student has running elsewhere.
    PXH_FIFO=/tmp/smoke-pxh
    rm -f "$PXH_FIFO"; mkfifo "$PXH_FIFO"
    tail -f /dev/null > "$PXH_FIFO" 2>/dev/null &
    FIFO_HOLDER=$!

    if command -v setsid >/dev/null; then
        setsid timeout "$SITL_BUDGET" env HEADLESS=1 \
            make px4_sitl gz_x500 < "$PXH_FIFO" > /tmp/sitl.log 2>&1 &
        SITL_PID=$!
        SITL_GROUP=1
    else
        timeout "$SITL_BUDGET" env HEADLESS=1 \
            make px4_sitl gz_x500 < "$PXH_FIFO" > /tmp/sitl.log 2>&1 &
        SITL_PID=$!
        SITL_GROUP=0
    fi

    # Poll rather than sleeping a flat delay: Gazebo asset loading is slower on
    # a cold cache, and a fixed sleep either wastes time or reports false failure.
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

        # The claim that matters is arming and takeoff; "PX4 started" alone
        # passed while every arming attempt was being denied.
        if [ -n "$GCS_PID" ]; then
            FLEW=0
            # The EKF needs settled sensor data before preflight checks pass,
            # so retry rather than judging once.
            for _ in $(seq 1 12); do
                echo "commander takeoff" > "$PXH_FIFO" 2>/dev/null
                sleep 5
                if tr '\r' '\n' < /tmp/sitl.log | grep -q "Takeoff detected"; then
                    FLEW=1; break
                fi
            done
            if [ "$FLEW" -eq 1 ]; then
                ok "vehicle armed and took off"
            else
                no "vehicle never left the ground"
                if tr '\r' '\n' < /tmp/sitl.log | grep -q "ground control station"; then
                    echo "        PX4 is refusing to arm because it sees no GCS link."
                    echo "        Run drone-gcs. Do NOT set NAV_DLL_ACT 0, that deletes"
                    echo "        the datalink-loss failsafe instead of supplying the link."
                fi
                tr '\r' '\n' < /tmp/sitl.log | grep -E "Arming denied|Preflight Fail" \
                    | sort -u | tail -4 | sed 's/^/        /'
            fi
        else
            skip "drone-gcs not found, cannot test arming"
        fi
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
    [ -n "$GCS_PID" ] && kill "$GCS_PID" 2>/dev/null
    [ -n "${FIFO_HOLDER:-}" ] && kill "$FIFO_HOLDER" 2>/dev/null
    rm -f "$PXH_FIFO"
fi

# ---------------------------------------------------------------------------
SECTION="display check"
echo "=== 5. GUI display path ==="
# 6080 serves the browser, 5901 a VNC client. A published port with nothing
# listening behind it reports no error on its own, so probe each.

[ -f /tmp/drone-display.env ] && source /tmp/drone-display.env

if [ -z "${DISPLAY:-}" ]; then
    no "DISPLAY is not set"
elif timeout 10 xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    RES=$(xdpyinfo -display "$DISPLAY" 2>/dev/null | awk '/dimensions:/{print $2; exit}')
    ok "X server up on DISPLAY=$DISPLAY (${RES:-unknown})"
else
    no "cannot reach an X server on DISPLAY=$DISPLAY"
    echo "        Try:  bash /opt/drone-vip/scripts/start-display.sh"
    echo "        Log:  /tmp/drone-display.log"
fi

NOVNC_PORT=6080                                # inside the container
HOST_PORT="${DRONE_NOVNC_HOST_PORT:-6080}"     # what it maps to on your machine
if timeout 10 bash -c "exec 3<>/dev/tcp/127.0.0.1/$NOVNC_PORT" 2>/dev/null; then
    ok "noVNC is serving  ->  http://localhost:$HOST_PORT"
else
    no "nothing is listening on $NOVNC_PORT (see /tmp/drone-display.log)"
    echo "        The browser GUI will not load until this is fixed."
fi

# 5901 is the same desktop for anyone who prefers a VNC client to a browser.
VNC_PORT=5901                                  # inside the container
VNC_HOST_PORT="${DRONE_VNC_HOST_PORT:-5901}"   # what it maps to on your machine
if timeout 10 bash -c "exec 3<>/dev/tcp/127.0.0.1/$VNC_PORT" 2>/dev/null; then
    ok "x11vnc is serving  ->  localhost:$VNC_HOST_PORT for a VNC client"
else
    no "nothing is listening on $VNC_PORT (see /tmp/drone-display.log)"
    echo "        A VNC client will not connect until this is fixed."
fi

# The renderer that matters is the one seen through vglrun, because the gz and
# rviz2 wrappers on PATH route through it; plain glxinfo always reports
# llvmpipe inside a virtual framebuffer. Check the resolved file, not the path,
# because whichever of /usr/local/bin/gz or ROS's vendored gz wins must route
# through VirtualGL.
GZ_RESOLVED=$(command -v gz 2>/dev/null)
if [ "${DRONE_VGL:-1}" != "1" ]; then
    skip "VirtualGL bypassed by DRONE_VGL=${DRONE_VGL}, so software rendering is expected"
elif [ -n "$GZ_RESOLVED" ] && grep -q 'vglrun' "$GZ_RESOLVED" 2>/dev/null; then
    ok "gz routes through VirtualGL ($GZ_RESOLVED)"
elif [ -n "$GZ_RESOLVED" ]; then
    no "gz at $GZ_RESOLVED does NOT route through VirtualGL"
    echo "        Gazebo would run without GPU acceleration. Rebuild the container."
fi

if command -v glxinfo >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    # With DRONE_VGL=0 the wrappers skip vglrun, so query without it too.
    if [ "${DRONE_VGL:-1}" = "1" ] && command -v vglrun >/dev/null 2>&1; then
        RENDERER=$(timeout 25 vglrun -d egl glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
    else
        RENDERER=$(timeout 20 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
    fi
    if [ -n "$RENDERER" ]; then
        case "$RENDERER" in
            *llvmpipe*|*softpipe*|*swrast*)
                ok "renderer: $RENDERER"
                echo "        Software rendering. Flight behavior is unaffected, physics is"
                echo "        CPU work and PX4 runs lockstep. Cameras are the exception, see"
                echo "        docs/simulation/SIMULATION-FIDELITY.md." ;;
            *)
                ok "renderer: $RENDERER (hardware accelerated)" ;;
        esac
    else
        skip "could not query the GL renderer"
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
