#!/usr/bin/env bash
# =============================================================================
# Verifies the native stack, and that it matches what the dev container runs.
#
#   ./scripts/native/smoke-test.sh
#   SITL_BUDGET=300 ./scripts/native/smoke-test.sh    on a slow machine
#
# The container equivalent is ./smoke-test.sh, which checks the container's own
# display stack instead of sections 3 and 6 here. Both compare the installed
# versions against the pins in .devcontainer/Dockerfile.
#
# NOTE: no `set -u`. ROS 2's setup.bash chain expands variables that may be
# unset, and under `set -u` that terminates the shell on the spot.
# =============================================================================
set -o pipefail

PASS=0; FAIL=0; SKIP=0
ok()   { echo "  PASS  $1"; PASS=$((PASS+1)); }
no()   { echo "  FAIL  $1"; FAIL=$((FAIL+1)); }
skip() { echo "  SKIP  $1"; SKIP=$((SKIP+1)); }
note() { echo "        $1"; }

FINISHED=0
SECTION="startup"
finish_guard() {
    local rc=$?
    [ "$FINISHED" -eq 1 ] && return 0
    echo
    echo "  smoke-test.sh stopped early during: $SECTION  (exit $rc)"
    echo "  A sourced setup file most likely terminated the shell."
    echo "  Diagnose with:  bash -x $0 2>&1 | tail -40"
}
trap finish_guard EXIT

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
DOCKERFILE="$REPO_ROOT/.devcontainer/Dockerfile"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
STAMP="$HOME/.cache/drone-vip/native-versions"

pin() { [ -f "$DOCKERFILE" ] && sed -n "s/^ARG $1=\(.*\)$/\1/p" "$DOCKERFILE" | head -1; }

# ---------------------------------------------------------------------------
SECTION="toolchain check"
echo "=== 1. Toolchain present ==="
echo "  info  architecture: $(uname -m), $( . /etc/os-release && echo "$PRETTY_NAME")"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    if ! source /opt/ros/jazzy/setup.bash 2>/tmp/ros-source.err; then
        no "sourcing /opt/ros/jazzy/setup.bash failed:"
        sed 's/^/        /' /tmp/ros-source.err
    fi
else
    no "/opt/ros/jazzy/setup.bash missing. ROS 2 Jazzy is not installed."
fi
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash" 2>/dev/null
[ -f "$REPO_ROOT/install/setup.bash" ] && source "$REPO_ROOT/install/setup.bash" 2>/dev/null

[ "${ROS_DISTRO:-}" = "jazzy" ] && ok "ROS 2 Jazzy" || no "ROS 2 Jazzy (got '${ROS_DISTRO:-none}')"

command -v gz >/dev/null && ok "gz binary" || no "gz binary"
# `cmd | grep -q` is unsafe under pipefail: grep exits on the first match, the
# upstream command takes SIGPIPE, and a match reads as a failure. Capture first.
GZ_VERSIONS=$(gz sim --versions 2>/dev/null)
if grep -q '^8\.' <<< "$GZ_VERSIONS"; then
    ok "Gazebo Harmonic (gz-sim $(head -1 <<< "$GZ_VERSIONS"))"
else
    no "Gazebo Harmonic, found: $(head -1 <<< "$GZ_VERSIONS")"
    note "PX4 runs whatever a bare 'gz sim' resolves to. Section 3 shows where it came from."
fi
if [ "$(wc -l <<< "$GZ_VERSIONS")" -gt 1 ]; then
    no "more than one Gazebo version is installed: $(tr '\n' ' ' <<< "$GZ_VERSIONS")"
    note "PX4 takes the first. Remove the others, or the simulation is not the one you think."
fi

command -v MicroXRCEAgent >/dev/null && ok "Micro XRCE-DDS Agent" || no "Micro XRCE-DDS Agent"
[ -d "$PX4_DIR" ] && ok "PX4 source at $PX4_DIR" || no "PX4 source at $PX4_DIR"
[ -x "$PX4_BIN" ] && ok "PX4 SITL built" \
  || no "PX4 SITL not built. Run: ./scripts/native/install.sh"

# ---------------------------------------------------------------------------
SECTION="version pinning check"
echo "=== 2. Versions match the container ==="
# Every pin comes from the Dockerfile, the one place this repo sets them, so
# native and container cannot drift apart without this section saying so.

WANT_PX4=$(pin PX4_VERSION)
WANT_MSGS=$(pin PX4_MSGS_BRANCH)
WANT_XRCE=$(pin XRCE_AGENT_VERSION)
if [ -n "$WANT_PX4" ]; then
    ok "read the pins from .devcontainer/Dockerfile"
else
    skip "cannot read $DOCKERFILE, nothing to compare against"
fi

PX4_TAG=$(cd "$PX4_DIR" 2>/dev/null && git describe --tags 2>/dev/null)
[ -n "$WANT_PX4" ] && [ "$PX4_TAG" = "$WANT_PX4" ] && ok "PX4 checkout at $WANT_PX4" \
  || no "PX4 tag is '${PX4_TAG:-unknown}', expected '${WANT_PX4:-unknown}'"

MSGS_BR=$(cd "$WS_DIR/src/px4_msgs" 2>/dev/null && git rev-parse --abbrev-ref HEAD 2>/dev/null)
[ -n "$WANT_MSGS" ] && [ "$MSGS_BR" = "$WANT_MSGS" ] && ok "px4_msgs on $WANT_MSGS" \
  || no "px4_msgs branch is '${MSGS_BR:-unknown}', expected '${WANT_MSGS:-unknown}'"

if [ -f "$STAMP" ]; then
    HAVE_XRCE=$(sed -n 's/^XRCE_AGENT_VERSION=//p' "$STAMP")
    [ "$HAVE_XRCE" = "$WANT_XRCE" ] && ok "XRCE-DDS Agent $WANT_XRCE" \
      || no "XRCE-DDS Agent is $HAVE_XRCE, expected $WANT_XRCE. Run: ./scripts/native/install.sh --rebuild-agent"
else
    skip "no install stamp at $STAMP, cannot tell which agent version is installed"
fi

# The fidelity claim in docs/SIMULATION-FIDELITY.md is that nothing in the
# simulation was changed. This is that claim, checked.
if [ -d "$PX4_DIR/.git" ]; then
    # PX4's worlds and models sit in a submodule, Tools/simulation/gz, and git
    # reports a submodule whose contents changed as one line naming the
    # submodule, which says nothing about what changed inside it. So read the
    # top level with submodule working trees ignored, which still reports a
    # submodule moved to a different commit, then ask every submodule for its
    # own status and put its path back on the front of each line.
    PX4_STATUS=$(cd "$PX4_DIR" && {
        git status --short --ignore-submodules=dirty
        git submodule --quiet foreach --recursive \
            'git status --short | sed "s|^\(...\)|\1$displaypath/|"'
    } 2>/dev/null)
    PX4_READ=$?

    PX4_DIRTY=""
    LINKED=0
    FOREIGN=""
    while IFS= read -r entry; do
        [ -n "$entry" ] || continue
        state=${entry:0:2}
        path=${entry:3}

        # The two NuttX submodule trees are removed on purpose by the installer
        # and by the container image, so git reports them deleted in both.
        case "$path" in
            platforms/nuttx/NuttX/nuttx|platforms/nuttx/NuttX/apps) continue ;;
        esac

        # install-worlds.sh links this repo's worlds and models in beside PX4's
        # own, because PX4 loads a world by name out of its own directory and
        # there is no other way in. See that script's header. Those links add
        # files, they change none of PX4's, so they are not what this check is
        # looking for. Only an untracked symlink is let through: editing one of
        # PX4's own worlds leaves a tracked file modified, and a modified file
        # still lands in the list below.
        if [ "$state" = "??" ] && [ -L "$PX4_DIR/$path" ]; then
            case "$path" in
                Tools/simulation/gz/worlds/*|Tools/simulation/gz/models/*)
                    target=$(readlink -f "$PX4_DIR/$path")
                    case "$target" in
                        "$REPO_ROOT"/*) LINKED=$((LINKED+1)) ;;
                        *) FOREIGN="$FOREIGN$path -> $target"$'\n' ;;
                    esac
                    continue ;;
            esac
        fi

        PX4_DIRTY="$PX4_DIRTY$entry"$'\n'
    done <<< "$PX4_STATUS"

    # A check that cannot read the tree has not checked anything, so say so
    # rather than reporting the empty list it just got as a clean tree.
    if [ "$PX4_READ" -ne 0 ]; then
        skip "git could not read the state of the PX4 tree at $PX4_DIR"
    elif [ -n "$PX4_DIRTY" ]; then
        no "the PX4 tree has local modifications"
        printf '%s' "$PX4_DIRTY" | head -10 | sed 's/^/        /'
    else
        ok "PX4 tree unmodified, apart from the removed NuttX submodules"
        [ "$LINKED" -gt 0 ] && \
          note "plus $LINKED world/model link(s) from this repo, which add files rather than change PX4's"
    fi
    if [ -n "$FOREIGN" ]; then
        note "linked into PX4 from outside $REPO_ROOT, so install-worlds.sh here did not put them there:"
        printf '%s' "$FOREIGN" | sed 's/^/          /'
    fi
fi

# ---------------------------------------------------------------------------
SECTION="gazebo wiring check"
echo "=== 3. Which Gazebo is actually in play ==="
# Two complete copies of Harmonic end up on a native machine: the one from
# packages.osrfoundation.org under /usr, and the one ROS 2 vendors under
# /opt/ros/jazzy/opt/gz_*_vendor, pulled in by ros-gz. Sourcing ROS puts the
# vendored copy first, which is also what happens inside the container. This
# reports it rather than leaving it to chance.

GZ_RESOLVED=$(command -v gz 2>/dev/null)
[ -n "$GZ_RESOLVED" ] && note "gz on PATH        : $GZ_RESOLVED"
note "GZ_CONFIG_PATH    : $(cut -d: -f1 <<< "${GZ_CONFIG_PATH:-unset}")${GZ_CONFIG_PATH:+ (first entry)}"

if [ -x "$PX4_BIN" ]; then
    GZ_LINKED=$(ldd "$PX4_BIN" 2>/dev/null | sed -n 's/.*=> \(.*libgz-transport[0-9]*\.so[^ ]*\).*/\1/p' | head -1)
    if [ -n "$GZ_LINKED" ]; then
        note "px4 loads         : $GZ_LINKED"
        case "$GZ_LINKED" in
            *gz-transport13*) ok "PX4 is bound to Harmonic's transport library" ;;
            *) no "PX4 loads $GZ_LINKED, which is not Harmonic (gz-transport13)"
               note "Rebuild it against Harmonic: ./scripts/native/install.sh --rebuild-px4" ;;
        esac
    else
        skip "could not read the Gazebo library PX4 links against"
    fi
fi

# ---------------------------------------------------------------------------
SECTION="ROS 2 check"
echo "=== 4. ROS 2 works ==="
ROS_PKGS=$(ros2 pkg list 2>/dev/null)
grep -q '^px4_msgs$' <<< "$ROS_PKGS" && ok "px4_msgs importable" \
  || no "px4_msgs not built. Run: ./scripts/native/install.sh --rebuild-msgs"
ros2 interface show px4_msgs/msg/VehicleLocalPosition >/dev/null 2>&1 \
  && ok "VehicleLocalPosition message" || no "VehicleLocalPosition message"
grep -q '^drone_vip$' <<< "$ROS_PKGS" && ok "drone_vip built" \
  || no "drone_vip not built. Run: colcon build --symlink-install"
grep -q '^drone_swarm$' <<< "$ROS_PKGS" && ok "drone_swarm built" \
  || no "drone_swarm not built. Run: colcon build --symlink-install"

note "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-unset (defaults to rmw_fastrtps_cpp)}"
note "ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID:-unset (0)}"
case "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" in
    LOCALHOST|OFF) ok "discovery limited to this machine (${ROS_AUTOMATIC_DISCOVERY_RANGE})" ;;
    "") no "ROS_AUTOMATIC_DISCOVERY_RANGE is unset, so discovery reaches your whole subnet"
        note "A teammate running this at the same time appears as extra drones on your topics."
        note "Fix: source scripts/native/env.sh" ;;
    *)  skip "discovery range is ${ROS_AUTOMATIC_DISCOVERY_RANGE}, which reaches other machines" ;;
esac

# ---------------------------------------------------------------------------
SECTION="SITL launch test"
echo "=== 5. Headless SITL launches and flies ==="
if [ ! -x "$PX4_BIN" ]; then
    skip "PX4 not built, skipping the launch test"
else
    SITL_BUDGET=${SITL_BUDGET:-180}
    export PX4_DIR
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
    unset PX4_GZ_STANDALONE
    cd "$PX4_DIR" || exit 1
    : > /tmp/sitl.log

    # PX4 refuses to arm without a ground control station, so supply one. The
    # failsafe stays at NAV_DLL_ACT 2, which is the point.
    GCS_BIN=$(command -v drone-gcs 2>/dev/null)
    [ -z "$GCS_BIN" ] && GCS_BIN="$REPO_ROOT/scripts/native/bin/drone-gcs"
    GCS_PID=""
    if [ -x "$GCS_BIN" ]; then
        "$GCS_BIN" > /tmp/smoke-gcs.log 2>&1 &
        GCS_PID=$!
    fi

    # A FIFO on stdin so the test can command a takeoff. With /dev/null it could
    # only prove PX4 starts, which is a much weaker claim than "it flies".
    PXH_FIFO=/tmp/smoke-pxh
    rm -f "$PXH_FIFO"; mkfifo "$PXH_FIFO"
    tail -f /dev/null > "$PXH_FIFO" 2>/dev/null &
    FIFO_HOLDER=$!

    # setsid puts make, px4 and gz in their own process group, so cleanup can
    # target exactly that group and not a PX4 you have running in another
    # terminal.
    setsid timeout "$SITL_BUDGET" env HEADLESS=1 \
        make px4_sitl gz_x500 < "$PXH_FIFO" > /tmp/sitl.log 2>&1 &
    SITL_PID=$!

    READY=0; DIED=0
    SITL_T0=$(date +%s)
    for _ in $(seq 1 $(( SITL_BUDGET / 2 ))); do
        if grep -qiE "Ready for takeoff|INFO\s+\[commander\]" /tmp/sitl.log; then READY=1; break; fi
        if ! kill -0 "$SITL_PID" 2>/dev/null; then DIED=1; break; fi
        sleep 2
    done
    SITL_ELAPSED=$(( $(date +%s) - SITL_T0 ))

    if [ "$READY" -eq 1 ]; then
        ok "PX4 SITL reached ready state (${SITL_ELAPSED}s)"
        if [ -n "$GCS_PID" ]; then
            FLEW=0
            # The EKF needs a few seconds of settled sensor data before the
            # preflight checks pass, so retry rather than judging once.
            for _ in $(seq 1 12); do
                echo "commander takeoff" > "$PXH_FIFO" 2>/dev/null
                sleep 5
                if tr '\r' '\n' < /tmp/sitl.log | grep -q "Takeoff detected"; then FLEW=1; break; fi
            done
            if [ "$FLEW" -eq 1 ]; then
                ok "vehicle armed and took off"
                RTF=$(tr '\r' '\n' < /tmp/sitl.log | sed -n 's/.*[Rr]eal.time factor[: ]*\([0-9.]*\).*/\1/p' | tail -1)
                [ -n "$RTF" ] && note "real-time factor reported: $RTF"
            else
                no "vehicle never left the ground"
                if tr '\r' '\n' < /tmp/sitl.log | grep -q "ground control station"; then
                    note "PX4 is refusing to arm because it sees no GCS link. Run drone-gcs."
                    note "Do NOT set NAV_DLL_ACT 0, that deletes the failsafe instead of supplying the link."
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
            no "PX4 SITL still not ready after ${SITL_BUDGET}s. Retry with SITL_BUDGET=300 on a slow machine. (full log: /tmp/sitl.log)"
        fi
        if grep -q "Standalone PX4 launch" /tmp/sitl.log; then
            note "PX4 is waiting for a Gazebo server nothing started. unset PX4_GZ_STANDALONE."
        fi
        tail -20 /tmp/sitl.log | sed 's/^/        /'
    fi

    kill -TERM -- -"$SITL_PID" 2>/dev/null
    sleep 2
    kill -KILL -- -"$SITL_PID" 2>/dev/null
    wait "$SITL_PID" 2>/dev/null
    [ -n "$GCS_PID" ] && kill "$GCS_PID" 2>/dev/null
    [ -n "${FIFO_HOLDER:-}" ] && kill "$FIFO_HOLDER" 2>/dev/null
    rm -f "$PXH_FIFO"
    cd "$REPO_ROOT" || exit 1
fi

# ---------------------------------------------------------------------------
SECTION="display check"
echo "=== 6. Graphics ==="
# Native runs on your own session and your own GPU driver. There is no VNC
# desktop and no VirtualGL in the path, which is the one place native beats the
# container outright.

if [ -n "${WAYLAND_DISPLAY:-}" ]; then
    ok "Wayland session (${WAYLAND_DISPLAY}), Qt set to ${QT_QPA_PLATFORM:-unset}"
    [ "${QT_QPA_PLATFORM:-}" = "xcb" ] || note "Set QT_QPA_PLATFORM=xcb if the Gazebo window misbehaves. env.sh does this."
elif [ -n "${DISPLAY:-}" ]; then
    ok "X11 session on DISPLAY=$DISPLAY"
else
    skip "no DISPLAY and no WAYLAND_DISPLAY. Headless is fine, use HEADLESS=1 drone-sim"
fi

if command -v glxinfo >/dev/null 2>&1 && { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; }; then
    RENDERER=$(timeout 20 glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p')
    case "$RENDERER" in
        "") skip "could not query the GL renderer" ;;
        *llvmpipe*|*softpipe*|*swrast*)
            ok "renderer: $RENDERER"
            note "Software rendering. Flight behavior is unaffected, physics is CPU work and"
            note "PX4 runs lockstep. Cameras are the exception, see docs/SIMULATION-FIDELITY.md." ;;
        *) ok "renderer: $RENDERER (hardware accelerated)" ;;
    esac
else
    skip "glxinfo not installed (sudo apt install mesa-utils), cannot name the renderer"
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
