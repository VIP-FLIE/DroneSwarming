#!/usr/bin/env bash
# =============================================================================
# Drone VIP environment check. Linux and MacOS.
#
#   ./scripts/doctor.sh          look, report, change nothing
#   ./scripts/doctor.sh --write  also write .devcontainer/.env
#
# This is optional. The container works without ever running it. All it does is
# check Docker is healthy and turn on GPU support if your machine has a GPU.
# =============================================================================
set -o pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ENV_FILE="$REPO_ROOT/.devcontainer/.env"
TMP="${TMPDIR:-/tmp}/drone-doctor.$$"
WRITE=0
[ "${1:-}" = "--write" ] && WRITE=1
trap 'rm -f "$TMP" "$TMP".done' EXIT

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
good() { printf '  \033[32myes\033[0m   %s\n' "$*"; }
bad()  { printf '  \033[31mno\033[0m    %s\n' "$*"; }
warn() { printf '  \033[33m??\033[0m    %s\n' "$*"; }
info() { printf '        %s\n' "$*"; }

# -----------------------------------------------------------------------------
# Portable timeout.
#
# MacOS does not ship GNU coreutils, so there is no `timeout` command. And
# `docker info` against Docker Desktop talks to a VM, which can take several
# seconds on a good day and block indefinitely while the VM is still coming up.
# An unguarded `docker info` is what makes this script hang on a Mac.
#
# The completion sentinel is a file rather than `kill -0 $pid`, because a
# finished background job stays a zombie until it is reaped, so `kill -0` keeps
# reporting it alive and we would wait out the full timeout every single time.
# -----------------------------------------------------------------------------
run_with_timeout() {
    secs="$1"; shift
    outfile="$1"; shift
    rm -f "$outfile.done"
    { "$@" >"$outfile" 2>&1; echo $? >"$outfile.done"; } &
    _pid=$!
    _i=0
    while [ ! -f "$outfile.done" ]; do
        if [ "$_i" -ge "$secs" ]; then
            kill -TERM "$_pid" 2>/dev/null
            sleep 1
            kill -KILL "$_pid" 2>/dev/null
            return 124
        fi
        sleep 1
        _i=$((_i + 1))
    done
    _rc=$(cat "$outfile.done" 2>/dev/null || echo 1)
    rm -f "$outfile.done"
    return "$_rc"
}

echo
bold "=== Drone VIP environment check ==="
echo

OS=$(uname -s)
ARCH=$(uname -m)
bold "Host"
info "OS           : $OS"
info "Architecture : $ARCH"
[ "$OS/$ARCH" = "Darwin/arm64" ] && info "Apple Silicon. The image is built native arm64, so no emulation."
echo

# -----------------------------------------------------------------------------
bold "Docker"
DOCKER_OK=0
DOCKER_INFO=""
if ! command -v docker >/dev/null 2>&1; then
    bad "docker not found"
    [ "$OS" = "Darwin" ] && info "https://docs.docker.com/desktop/install/mac-install/" \
                         || info "https://docs.docker.com/engine/install/"
else
    # ONE docker call for the whole script. Every extra call is another chance
    # to hit a slow or wedged daemon, so the result is cached and reused.
    printf '  ...   asking Docker how it is doing (up to 25s)\r'
    if run_with_timeout 25 "$TMP" docker info --format '{{json .}}'; then
        DOCKER_INFO=$(cat "$TMP" 2>/dev/null)
        printf '                                             \r'
        good "docker is installed and the daemon is responding"
        DOCKER_OK=1
    else
        RC=$?
        printf '                                             \r'
        if [ "$RC" = "124" ]; then
            bad "Docker did not answer within 25 seconds"
            info "The daemon is usually still starting up. Open Docker Desktop,"
            info "wait for the whale icon to stop animating, then run this again."
            info "If it never settles, quit Docker Desktop fully and reopen it."
        else
            bad "docker is installed but the daemon is not responding"
            [ "$OS" = "Darwin" ] && info "Start Docker Desktop and wait for it to finish starting." \
                                 || info "Try: sudo systemctl start docker"
        fi
    fi
fi

if [ "$DOCKER_OK" = "1" ]; then
    # Parsed out of the single JSON blob above. No extra docker calls.
    MEM=$(printf '%s' "$DOCKER_INFO" | sed -n 's/.*"MemTotal":\([0-9]*\).*/\1/p')
    if [ -n "$MEM" ] && [ "$MEM" -gt 0 ] 2>/dev/null; then
        MEM_GB=$(( MEM / 1024 / 1024 / 1024 ))
        if [ "$MEM_GB" -ge 8 ]; then
            good "Docker has ${MEM_GB} GB of RAM"
        else
            warn "Docker only has ${MEM_GB} GB of RAM, 8 or more is much better"
            [ "$OS" = "Darwin" ] && info "Docker Desktop > Settings > Resources > Memory"
        fi
    fi
fi
echo

# -----------------------------------------------------------------------------
bold "GPU"
HAS_NVIDIA=0
if [ "$OS" = "Darwin" ]; then
    warn "MacOS cannot pass a GPU to Docker, so you get software rendering"
    info "That is a platform limit, not something to fix. Flight behavior is"
    info "unaffected. Only camera-heavy work really notices. See"
    info "docs/simulation/SIMULATION-FIDELITY.md."
elif [ "$DOCKER_OK" = "1" ] && printf '%s' "$DOCKER_INFO" | grep -q '"nvidia"'; then
    good "Docker can see an NVIDIA GPU"
    HAS_NVIDIA=1
elif command -v nvidia-smi >/dev/null 2>&1; then
    warn "you have an NVIDIA GPU but Docker cannot see it"
    info "Install the NVIDIA Container Toolkit, then run this again:"
    info "https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html"
else
    warn "no NVIDIA GPU found, you will get software rendering"
    info "Fine for everything except camera-heavy work."
fi
echo

# -----------------------------------------------------------------------------
bold "=== What to do next ==="
echo
info "Open the repo in VS Code, hit F1, run 'Dev Containers: Reopen in Container'."
info "There is only one configuration, so there is nothing to choose."
info "When it is up, open http://localhost:6080 to see the simulation."
echo

if [ "$WRITE" = "1" ]; then
    # The header is written to match what this machine actually got. A blanket
    # "safe to delete" is true on a GPU-less machine and actively misleading on
    # a machine with a GPU, where deleting the file costs you the GPU with
    # nothing anywhere reporting an error.
    {
        echo "# Written by scripts/doctor.sh. Machine-specific, gitignored."
        if [ "$HAS_NVIDIA" = "1" ]; then
            echo "#"
            echo "# Docker on this machine can see an NVIDIA GPU, and the lines below are"
            echo "# what hand it to the container. DO NOT delete this file expecting"
            echo "# nothing to change. Without it compose falls back to the runc runtime,"
            echo "# the GPU never reaches the container, and VirtualGL quietly degrades to"
            echo "# llvmpipe software rendering. Everything still starts and the GUI still"
            echo "# works, so the only symptom is that the simulation runs slower:"
            echo "# measured on an RTX 3070, gz sim real-time factor 1.00 -> 0.81."
            echo "#"
            echo "# Regenerate with: ./scripts/doctor.sh --write"
            echo
            echo "DOCKER_RUNTIME=nvidia"
            echo "NVIDIA_VISIBLE_DEVICES=all"
            echo "NVIDIA_DRIVER_CAPABILITIES=all"
        else
            echo "#"
            echo "# No GPU visible to Docker on this machine, so there is nothing to"
            echo "# configure and this file sets nothing. It IS safe to delete: the"
            echo "# container behaves identically with or without it, and you get"
            echo "# llvmpipe software rendering either way."
            echo "#"
            echo "# Regenerate with: ./scripts/doctor.sh --write"
        fi
    } > "$ENV_FILE"
    bold "Wrote $ENV_FILE"
    sed 's/^/  /' "$ENV_FILE"
    echo
fi

[ "$DOCKER_OK" = "1" ] || exit 1
exit 0
