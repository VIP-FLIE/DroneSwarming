#!/usr/bin/env bash
# =============================================================================
# Installs the drone stack directly on Ubuntu 24.04 (noble) and derivatives.
#
#   ./scripts/native/install.sh              install or resume
#   ./scripts/native/install.sh --rebuild-px4
#   ./scripts/native/install.sh --skip-apt   skip every apt step
#   JOBS=4 ./scripts/native/install.sh       cap build parallelism
#
# Safe to re-run. Every step checks whether it is already done and skips.
#
# The versions installed here are read out of .devcontainer/Dockerfile, which
# is the only place this repo pins them. Native and container stay on the same
# PX4, px4_msgs and XRCE-DDS Agent because they read the same numbers.
#
# NOTE: no `set -u`. ROS 2 setup files reference unset variables internally and
# would terminate the shell.
# =============================================================================
set -o pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
DOCKERFILE="$REPO_ROOT/.devcontainer/Dockerfile"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
STAMP_DIR="$HOME/.cache/drone-vip"
STAMP="$STAMP_DIR/native-versions"
START=$(date +%s)

REBUILD_PX4=0
REBUILD_AGENT=0
REBUILD_MSGS=0
SKIP_APT=0
for arg in "$@"; do
    case "$arg" in
        --rebuild-px4)   REBUILD_PX4=1 ;;
        --rebuild-agent) REBUILD_AGENT=1 ;;
        --rebuild-msgs)  REBUILD_MSGS=1 ;;
        --skip-apt)      SKIP_APT=1 ;;
        -h|--help)
            cat <<'USAGE'
Installs the drone stack natively on Ubuntu 24.04 (noble) and derivatives.

  ./scripts/native/install.sh                 install, or resume an interrupted one
  ./scripts/native/install.sh --rebuild-px4   recompile PX4 SITL
  ./scripts/native/install.sh --rebuild-msgs  rebuild px4_msgs
  ./scripts/native/install.sh --rebuild-agent rebuild the XRCE-DDS Agent
  ./scripts/native/install.sh --skip-apt      skip every apt step
  JOBS=4 ./scripts/native/install.sh          cap build parallelism

Safe to re-run. Every step checks whether it is already done and skips.
Versions come from the ARG defaults in .devcontainer/Dockerfile.
Full guide: docs/setup/NATIVE-SETUP.md
USAGE
            exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

stage() { printf '\n\033[1;36m==> %s\033[0m  (%ds elapsed)\n' "$1" "$(( $(date +%s) - START ))"; }
info()  { printf '    %s\n' "$*"; }
warn()  { printf '    \033[33mWARNING\033[0m %s\n' "$*"; }
die()   { printf '\n\033[31mSTOPPED\033[0m %s\n\n' "$*" >&2; exit 1; }

# -----------------------------------------------------------------------------
# Build PX4 with ROS 2 out of the environment.
#
# ROS 2 Jazzy ships its own complete copy of Gazebo Harmonic under
# /opt/ros/jazzy/opt/gz_*_vendor, and sourcing ROS puts those on CMAKE_PREFIX_PATH
# and LD_LIBRARY_PATH. PX4's cmake would then link against the vendored copy.
# The container image builds PX4 in a Docker RUN step, which never sources ROS,
# so it links against the system Gazebo from packages.osrfoundation.org. This
# reproduces that.
# -----------------------------------------------------------------------------
strip_ros_paths() {
    local out=""
    local IFS=:
    for entry in $1; do
        case "$entry" in
            /opt/ros/*) continue ;;
            "") continue ;;
        esac
        out="${out:+$out:}$entry"
    done
    printf '%s' "$out"
}

ros_free() {
    local clean_ld args
    clean_ld=$(strip_ros_paths "$LD_LIBRARY_PATH")
    # Every -u has to come before the first VAR=VALUE, or env reads the flag as
    # the name of the command to run.
    args=(env
        -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH
        -u AMENT_CURRENT_PREFIX -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION
        -u ROS_PACKAGE_PATH -u PYTHONPATH -u GZ_CONFIG_PATH
        -u GZ_SIM_SYSTEM_PLUGIN_PATH -u GZ_SIM_RESOURCE_PATH)
    # An empty LD_LIBRARY_PATH is not the same as no LD_LIBRARY_PATH: some
    # loaders read it as the current directory.
    [ -z "$clean_ld" ] && args+=(-u LD_LIBRARY_PATH)
    args+=(PATH="$(strip_ros_paths "$PATH")")
    [ -n "$clean_ld" ] && args+=(LD_LIBRARY_PATH="$clean_ld")
    "${args[@]}" "$@"
}

# Reads an ARG default out of the Dockerfile, e.g. pin PX4_VERSION
pin() { sed -n "s/^ARG $1=\(.*\)$/\1/p" "$DOCKERFILE" | head -1; }

# Records what is installed, one KEY=VALUE per line, read back by
# scripts/native/smoke-test.sh. Written as each step finishes rather than only
# at the end, so a run that stops partway does not lose what it already did.
stamp_set() {
    mkdir -p "$STAMP_DIR"
    touch "$STAMP"
    grep -v "^$1=" "$STAMP" > "$STAMP.new"
    echo "$1=$2" >> "$STAMP.new"
    mv "$STAMP.new" "$STAMP"
}

# =============================================================================
stage "Checking this machine"

[ -f "$DOCKERFILE" ] || die "cannot find $DOCKERFILE. Run this from inside the repo."

PX4_VERSION=$(pin PX4_VERSION)
PX4_MSGS_BRANCH=$(pin PX4_MSGS_BRANCH)
XRCE_AGENT_VERSION=$(pin XRCE_AGENT_VERSION)
[ -n "$PX4_VERSION" ] && [ -n "$PX4_MSGS_BRANCH" ] && [ -n "$XRCE_AGENT_VERSION" ] \
    || die "could not read the version pins out of $DOCKERFILE"

. /etc/os-release
CODENAME="${VERSION_CODENAME:-$(lsb_release -cs 2>/dev/null)}"
if [ "$CODENAME" != "noble" ]; then
    die "this installs Ubuntu 24.04 (noble) packages, and this machine is '$CODENAME'.
        ROS 2 Jazzy and Gazebo Harmonic only publish debs for noble.
        On any other distro use the dev container: docs/setup/SETUP.md"
fi

ARCH=$(uname -m)
case "$ARCH" in
    x86_64|aarch64) ;;
    *) die "unsupported architecture: $ARCH" ;;
esac

command -v sudo >/dev/null || die "sudo is required"

MEM_GB=$(awk '/MemTotal/{printf "%d", $2/1024/1024}' /proc/meminfo)
DISK_GB=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
CPUS=$(nproc)
if [ -z "${JOBS:-}" ]; then
    # PX4's C++ build is memory hungry. Roughly one job per 2 GB, because an
    # over-parallel build swaps itself to death and that looks like a hang.
    JOBS=$(( MEM_GB / 2 ))
    [ "$JOBS" -lt 1 ] && JOBS=1
    [ "$JOBS" -gt "$CPUS" ] && JOBS=$CPUS
fi

info "distro         : $PRETTY_NAME ($CODENAME, $ARCH)"
info "CPUs / RAM     : $CPUS / ${MEM_GB} GB"
info "free disk      : ${DISK_GB} GB in $HOME"
info "parallel jobs  : $JOBS  (override with JOBS=n)"
info "PX4            : $PX4_VERSION"
info "px4_msgs       : $PX4_MSGS_BRANCH"
info "XRCE agent     : $XRCE_AGENT_VERSION"

[ "${DISK_GB:-0}" -lt 20 ] && warn "under 20 GB free. PX4 plus its build output needs about 15 GB."
[ "${MEM_GB:-0}" -lt 8 ]   && warn "under 8 GB RAM. The PX4 build will be slow."

# =============================================================================
stage "System packages"
if [ "$SKIP_APT" -eq 1 ]; then
    info "skipped (--skip-apt)"
else
    # The same set the container image installs, minus its display stack.
    sudo apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        git curl wget lsb-release gnupg ca-certificates \
        build-essential cmake ninja-build ccache \
        python3-pip python3-venv python3-dev \
        libeigen3-dev libopencv-dev protobuf-compiler libprotobuf-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
        cppzmq-dev libzmq3-dev \
        mesa-utils \
        || die "apt failed on the base packages"
    info "base packages installed"
fi

# =============================================================================
stage "ROS 2 Jazzy"
if [ -d /opt/ros/jazzy ]; then
    info "already installed at /opt/ros/jazzy"
elif [ "$SKIP_APT" -eq 1 ]; then
    die "ROS 2 Jazzy is not installed and --skip-apt was given"
else
    info "adding the ROS 2 apt repository"
    # The ros2-apt-source package carries the signing key and the source list,
    # and updates the key through apt when it rotates.
    APT_SRC_VERSION=$(curl -sf https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    if [ -n "$APT_SRC_VERSION" ] && curl -sfL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${APT_SRC_VERSION}/ros2-apt-source_${APT_SRC_VERSION}.${CODENAME}_all.deb"; then
        sudo apt-get install -y /tmp/ros2-apt-source.deb || die "installing ros2-apt-source failed"
        rm -f /tmp/ros2-apt-source.deb
    else
        warn "could not fetch ros2-apt-source, falling back to the keyring method"
        sudo curl -sfL -o /usr/share/keyrings/ros-archive-keyring.gpg \
            https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            || die "could not fetch the ROS signing key"
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    fi
    sudo apt-get update -qq
fi

if [ "$SKIP_APT" -eq 0 ]; then
    # colcon is not part of ros-jazzy-desktop. The container inherits it from
    # the ros:jazzy-ros-base image, so it has to be asked for explicitly here.
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        || die "apt failed on ros-jazzy-desktop"
    info "ros-jazzy-desktop and colcon present"
fi

# =============================================================================
stage "Gazebo Harmonic"
if [ "$SKIP_APT" -eq 0 ]; then
    if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
        info "adding the packages.osrfoundation.org repository"
        sudo curl -sfL -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
            https://packages.osrfoundation.org/gazebo.gpg \
            || die "could not fetch the Gazebo signing key"
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $CODENAME main" \
            | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
        sudo apt-get update -qq
    fi
    # gz-harmonic is the simulator PX4 links against. ros-gz is the ROS 2 side
    # bridge, and installing it also brings ROS's vendored copy of Harmonic,
    # which is what actually runs once ROS is sourced. The container has both.
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        gz-harmonic ros-jazzy-ros-gz \
        || die "apt failed on gz-harmonic"
    info "gz-harmonic and ros-jazzy-ros-gz installed"
fi

# Another major version of Gazebo on the same machine is the one thing that
# silently changes what you are simulating: PX4 calls a bare `gz sim` and takes
# whatever answers.
OTHER_GZ=$(dpkg -l 2>/dev/null | awk '/^ii +(gz-ionic|gz-jetty|gz-garden|gz-fortress)/{print $2}')
if [ -n "$OTHER_GZ" ]; then
    warn "another Gazebo release is installed: $(echo "$OTHER_GZ" | tr '\n' ' ')"
    warn "run ./scripts/native/smoke-test.sh, section 1 reports which one PX4 gets"
fi

# =============================================================================
stage "Python packages for the PX4 build"
if [ "$SKIP_APT" -eq 1 ]; then
    info "skipped (--skip-apt)"
else
    # Same list and same pin as the container image. empy must stay below 4:
    # both PX4's and ROS 2's code generators break on the 4.x API.
    python3 -m pip install --break-system-packages --user -q -U \
        empy==3.3.4 pyros-genmsg setuptools packaging jinja2 jsonschema \
        pyserial numpy toml pyyaml kconfiglib future lxml \
        || die "pip failed"
    info "installed into ~/.local"
fi
case ":$PATH:" in
    *":$HOME/.local/bin:"*) ;;
    *) warn "$HOME/.local/bin is not on PATH. scripts/native/env.sh adds it." ;;
esac

# =============================================================================
stage "Micro XRCE-DDS Agent $XRCE_AGENT_VERSION"
# The agent binary does not report which tag it was built from, so the stamp
# this script writes is the record. An agent installed by some other route has
# no stamp and gets replaced with the pinned build.
STAMPED_XRCE=$(sed -n 's/^XRCE_AGENT_VERSION=//p' "$STAMP" 2>/dev/null)
if command -v MicroXRCEAgent >/dev/null 2>&1 \
   && [ "$STAMPED_XRCE" = "$XRCE_AGENT_VERSION" ] && [ "$REBUILD_AGENT" -eq 0 ]; then
    info "already installed at $(command -v MicroXRCEAgent). Pass --rebuild-agent to redo."
else
    if command -v MicroXRCEAgent >/dev/null 2>&1; then
        info "an agent is installed but not recorded as $XRCE_AGENT_VERSION, rebuilding it"
    fi
    SRC=/tmp/xrce-agent-build
    rm -rf "$SRC"
    git clone -q -b "$XRCE_AGENT_VERSION" --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$SRC" \
        || die "could not clone the XRCE-DDS Agent"
    info "building (this pulls and compiles Fast DDS, give it a few minutes)"
    cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release >/dev/null \
        || die "cmake failed for the XRCE-DDS Agent"
    cmake --build "$SRC/build" -j"$JOBS" >/dev/null \
        || die "the XRCE-DDS Agent failed to build"
    sudo cmake --install "$SRC/build" >/dev/null || die "installing the agent failed"
    sudo ldconfig /usr/local/lib/
    rm -rf "$SRC"
    stamp_set XRCE_AGENT_VERSION "$XRCE_AGENT_VERSION"
    info "installed to /usr/local/bin/MicroXRCEAgent"
fi

# =============================================================================
stage "PX4 $PX4_VERSION"
if [ -d "$PX4_DIR/.git" ]; then
    HAVE_TAG=$(cd "$PX4_DIR" && git describe --tags 2>/dev/null)
    if [ "$HAVE_TAG" != "$PX4_VERSION" ]; then
        die "$PX4_DIR is at '$HAVE_TAG', not $PX4_VERSION.
        Move it aside and re-run, or check it out yourself:
          cd $PX4_DIR && git fetch --depth 1 origin tag $PX4_VERSION && git checkout $PX4_VERSION"
    fi
    info "source already at $PX4_VERSION"
else
    info "cloning (about 1.7 GB, several minutes)"
    # --shallow-submodules keeps roughly 1 GB of submodule history out of the
    # clone. It is also why the NuttX trees are removed below rather than left
    # in place: px_update_git_header.py crashes on a present-but-tagless NuttX
    # checkout, and skips cleanly over an absent one.
    git clone --depth 1 -b "$PX4_VERSION" --recursive --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" \
        || die "the PX4 clone failed"
fi

if [ -d "$PX4_DIR/platforms/nuttx/NuttX/nuttx" ]; then
    info "removing the NuttX submodules (firmware toolchain, unused by SITL)"
    rm -rf "$PX4_DIR/platforms/nuttx/NuttX/nuttx" "$PX4_DIR/platforms/nuttx/NuttX/apps"
fi

if [ "$SKIP_APT" -eq 0 ]; then
    info "running PX4's own dependency script"
    # --no-nuttx skips the ARM cross toolchain, so this tree builds SITL only.
    # --no-sim-tools skips PX4's Gazebo install, which is already done above.
    ( cd "$PX4_DIR" && bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools ) \
        || die "PX4's Tools/setup/ubuntu.sh failed"
fi

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [ -x "$PX4_BIN" ] && [ "$REBUILD_PX4" -eq 0 ]; then
    info "already built. Pass --rebuild-px4 to redo."
else
    info "compiling PX4 SITL with $JOBS jobs. A few minutes on a recent laptop,"
    info "longer on older hardware. Build output is hidden unless it fails."
    # GZ_DISTRO=harmonic narrows PX4's cmake search to the Harmonic library
    # versions (gz-sim8, gz-transport13, gz-sensors8, gz-plugin2). Without it
    # PX4 looks for the Ionic versions first and takes them if they are present.
    if ! ( cd "$PX4_DIR" && ros_free env GZ_DISTRO=harmonic make px4_sitl_default -j"$JOBS" ); then
        die "the PX4 build failed. If the machine ran out of memory, retry with fewer jobs:
          JOBS=2 ./scripts/native/install.sh --rebuild-px4"
    fi
    info "PX4 built"
fi

# The gz_bridge module is what talks to Gazebo, and PX4 drops it silently when
# cmake cannot find Gazebo. The symptom otherwise shows up much later as a
# simulator that starts and never spawns a vehicle.
GZ_LINKED=$(ldd "$PX4_BIN" 2>/dev/null | sed -n 's/.*=> \(.*libgz-transport[0-9]*\.so[^ ]*\).*/\1/p' | head -1)
case "$GZ_LINKED" in
    *gz-transport13*) info "linked against Harmonic: $GZ_LINKED" ;;
    "") warn "the px4 binary does not link any Gazebo transport library."
        warn "It was built without Gazebo support. Re-run with --rebuild-px4 once gz-harmonic is installed." ;;
    *)  warn "the px4 binary links $GZ_LINKED, which is not Harmonic (gz-transport13)." ;;
esac

# =============================================================================
stage "px4_msgs ($PX4_MSGS_BRANCH)"
if [ -d "$WS_DIR/src/px4_msgs/.git" ]; then
    HAVE_BRANCH=$(cd "$WS_DIR/src/px4_msgs" && git rev-parse --abbrev-ref HEAD 2>/dev/null)
    if [ "$HAVE_BRANCH" != "$PX4_MSGS_BRANCH" ]; then
        die "$WS_DIR/src/px4_msgs is on '$HAVE_BRANCH', not $PX4_MSGS_BRANCH.
        Message definitions that do not match the PX4 build decode to garbage.
        Fix it with:
          cd $WS_DIR/src/px4_msgs && git fetch origin $PX4_MSGS_BRANCH && git checkout $PX4_MSGS_BRANCH"
    fi
    info "source already on $PX4_MSGS_BRANCH"
else
    mkdir -p "$WS_DIR/src"
    git clone -q --depth 1 -b "$PX4_MSGS_BRANCH" \
        https://github.com/PX4/px4_msgs.git "$WS_DIR/src/px4_msgs" \
        || die "the px4_msgs clone failed"
fi

# `set -e` is never enabled in this script, which is what makes sourcing ROS
# setup files here safe: they expand variables that may be unset.
source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ] && [ "$REBUILD_MSGS" -eq 0 ]; then
    info "already built. Pass --rebuild-msgs to redo."
else
    info "building px4_msgs"
    ( cd "$WS_DIR" && colcon build --symlink-install --parallel-workers "$JOBS" ) \
        || die "px4_msgs failed to build"
    info "px4_msgs built"
fi

# =============================================================================
stage "This repo's packages"
# Build output from a previous dev container run names container paths and
# cannot be reused here. It is all regenerated, so clearing it costs nothing.
if grep -rqs "/workspaces/" "$REPO_ROOT/build" "$REPO_ROOT/install" 2>/dev/null; then
    info "clearing build output left over from the dev container"
    rm -rf "$REPO_ROOT/build" "$REPO_ROOT/install" "$REPO_ROOT/log"
fi

source "$WS_DIR/install/setup.bash"
( cd "$REPO_ROOT" && colcon build --symlink-install --parallel-workers "$JOBS" ) \
    || die "this repo's packages failed to build"
info "py_single_square and cpp_single_hop built"

# =============================================================================
stamp_set PX4_VERSION "$PX4_VERSION"
stamp_set PX4_MSGS_BRANCH "$PX4_MSGS_BRANCH"
stamp_set XRCE_AGENT_VERSION "$XRCE_AGENT_VERSION"
stamp_set INSTALLED_ON "$(date -Iseconds)"

TOTAL=$(( $(date +%s) - START ))
stage "Done"
printf '    Total time: %dm %ds\n\n' $(( TOTAL / 60 )) $(( TOTAL % 60 ))
cat <<NEXT
    Add this line to ~/.bashrc, then open a new terminal:

        source $REPO_ROOT/scripts/native/env.sh

    Then verify the stack:

        ./scripts/native/smoke-test.sh

NEXT
