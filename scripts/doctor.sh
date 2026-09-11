#!/usr/bin/env bash
# =============================================================================
# Drone VIP environment doctor  --  Linux and macOS
#
#   ./scripts/doctor.sh          inspect and recommend
#   ./scripts/doctor.sh --write  also write .devcontainer/.env
#
# Looks at what your machine can actually do and tells you WHICH dev container
# configuration to open.
# =============================================================================
set -o pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ENV_FILE="$REPO_ROOT/.devcontainer/.env"
WRITE=0
[ "${1:-}" = "--write" ] && WRITE=1

bold()  { printf '\033[1m%s\033[0m\n' "$*"; }
good()  { printf '  \033[32myes\033[0m   %s\n' "$*"; }
bad()   { printf '  \033[31mno\033[0m    %s\n' "$*"; }
warn()  { printf '  \033[33m??\033[0m    %s\n' "$*"; }
info()  { printf '        %s\n' "$*"; }

echo
bold "=== Drone VIP environment check ==="
echo

# --- host --------------------------------------------------------------------
OS=$(uname -s)
ARCH=$(uname -m)
bold "Host"
info "OS           : $OS"
info "Architecture : $ARCH"
case "$OS/$ARCH" in
    Darwin/arm64)
        info "Apple Silicon detected. The image builds natively for arm64,"
        info "so you are NOT running under emulation. Good."
        ;;
    Darwin/x86_64)
        info "Intel Mac detected."
        ;;
esac
echo

# --- docker ------------------------------------------------------------------
bold "Docker"
DOCKER_OK=0
if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
        good "docker is installed and the daemon is running"
        DOCKER_OK=1
    else
        bad "docker is installed but the daemon is not responding"
        info "Start Docker Desktop (macOS) or: sudo systemctl start docker (Linux)"
    fi
else
    bad "docker not found"
    info "macOS:  https://docs.docker.com/desktop/install/mac-install/"
    info "Linux:  https://docs.docker.com/engine/install/"
fi

# Memory available to Docker. PX4 + Gazebo need headroom; Docker Desktop
# defaults are often too small and the failure mode is a build that looks hung.
if [ "$DOCKER_OK" = "1" ]; then
    DOCKER_MEM=$(docker info --format '{{.MemTotal}}' 2>/dev/null)
    if [ -n "$DOCKER_MEM" ] && [ "$DOCKER_MEM" -gt 0 ] 2>/dev/null; then
        DOCKER_MEM_GB=$(( DOCKER_MEM / 1024 / 1024 / 1024 ))
        if [ "$DOCKER_MEM_GB" -ge 8 ]; then
            good "Docker has ${DOCKER_MEM_GB} GB of RAM"
        else
            warn "Docker has only ${DOCKER_MEM_GB} GB of RAM"
            info "8 GB or more is strongly recommended."
            if [ "$OS" = "Darwin" ]; then
                info "Raise it in Docker Desktop > Settings > Resources."
            fi
        fi
    fi
fi
echo

# --- gpu ---------------------------------------------------------------------
bold "GPU"
HAS_NVIDIA=0
if [ "$OS" = "Darwin" ]; then
    bad "no GPU passthrough is possible on macOS"
    info "Docker on macOS runs in a VM with no GPU access. This is a hard"
    info "platform limit, not a configuration problem. Software rendering it is."
    info "Physics still runs at full speed; only GUI framerate is affected."
elif [ "$DOCKER_OK" = "1" ] && docker info 2>/dev/null | grep -q 'nvidia'; then
    good "NVIDIA Container Toolkit is installed and registered with Docker"
    HAS_NVIDIA=1
    command -v nvidia-smi >/dev/null 2>&1 && \
        info "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
elif command -v nvidia-smi >/dev/null 2>&1; then
    warn "an NVIDIA GPU is present but the Container Toolkit is NOT installed"
    info "Without it, Docker cannot see the GPU. Install:"
    info "https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html"
else
    warn "no NVIDIA GPU detected -- software rendering will be used"
    info "That is fine. Physics is unaffected; only GUI framerate suffers."
fi
echo

# --- display -----------------------------------------------------------------
bold "Display"
NATIVE_DISPLAY=0
XAUTH_OUT=""
if [ "$OS" = "Darwin" ]; then
    bad "no native X11 path on macOS"
    info "XQuartz exists but is slow and unreliable for Gazebo. Use the"
    info "browser GUI instead -- it is genuinely the better option here."
elif [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    good "X server reachable at DISPLAY=$DISPLAY"
    [ "${XDG_SESSION_TYPE:-}" = "wayland" ] && \
        info "Wayland session; apps will go through XWayland. That works."
    NATIVE_DISPLAY=1

    # X access control comes in two flavours and the container needs to get
    # past whichever one you have. Detect, do not guess.
    XHOST_OUT=$(xhost 2>/dev/null)
    if grep -q 'SI:localuser' <<< "$XHOST_OUT"; then
        good "access control is SI:localuser (uid-based) -- no cookie needed"
        info "The container is authorized by matching your uid ($(id -u))."
        info "VS Code remaps the container user to your uid automatically."
        HOST_UID=$(id -u)
        if [ "$HOST_UID" != "1000" ]; then
            warn "your uid is $HOST_UID, but the image's user is uid 1000"
            info "VS Code's updateRemoteUserUID normally fixes this. If GUI apps"
            info "get 'cannot open display', that is the thing that failed."
        fi
    elif command -v xauth >/dev/null 2>&1; then
        # Traditional MIT-MAGIC-COOKIE. Extract one the container can use.
        XAUTH_OUT=/tmp/.docker.xauth
        rm -f "$XAUTH_OUT"; touch "$XAUTH_OUT"
        # 'ffff' rewrites the address family to FamilyWild, so the cookie is
        # accepted from inside the container's network namespace too.
        xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' \
            | xauth -f "$XAUTH_OUT" nmerge - 2>/dev/null
        chmod 644 "$XAUTH_OUT"
        if [ -s "$XAUTH_OUT" ]; then
            good "MIT-MAGIC-COOKIE extracted to $XAUTH_OUT"
        else
            warn "no cookie found and no SI:localuser rule"
            info "If GUI apps cannot open the display, the native config will"
            info "fall back to the browser GUI automatically."
            XAUTH_OUT=""
        fi
    fi
else
    warn "no X display detected (headless or SSH session)"
    info "The browser GUI works here and needs nothing extra."
fi
echo

# --- recommendation ----------------------------------------------------------
bold "=== Recommendation ==="
echo
if [ "$NATIVE_DISPLAY" = "1" ]; then
    printf '  Open the \033[1mDrone VIP - Linux (native X11 + GPU)\033[0m configuration.\n'
    [ "$HAS_NVIDIA" = "1" ] && info "Your NVIDIA GPU will be used. This is the best experience available." \
                            || info "Software rendering; still the fastest option on this machine."
else
    printf '  Open the \033[1mDrone VIP - Universal (browser GUI)\033[0m configuration.\n'
    info "Then browse to http://localhost:6080 once it starts."
fi
echo
info "In VS Code: F1 > 'Dev Containers: Reopen in Container' > pick from the list."
echo

# --- write .env ---------------------------------------------------------------
if [ "$WRITE" = "1" ]; then
    {
        echo "# Generated by scripts/doctor.sh on $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "# Machine-specific. Gitignored. Re-run the doctor to regenerate."
        echo
        if [ "$HAS_NVIDIA" = "1" ]; then
            echo "DOCKER_RUNTIME=nvidia"
            echo "NVIDIA_VISIBLE_DEVICES=all"
            echo "NVIDIA_DRIVER_CAPABILITIES=all"
        else
            echo "# No NVIDIA Container Toolkit found; leaving runtime at the default."
        fi
        [ -n "$XAUTH_OUT" ] && echo "XAUTH_FILE=$XAUTH_OUT"
        if [ "$NATIVE_DISPLAY" = "1" ]; then
            echo "DRONE_DISPLAY_MODE=auto"
        else
            echo "DRONE_DISPLAY_MODE=vnc"
        fi
    } > "$ENV_FILE"
    bold "Wrote $ENV_FILE"
    sed 's/^/  /' "$ENV_FILE"
    echo
fi

[ "$DOCKER_OK" = "1" ] || exit 1
exit 0
