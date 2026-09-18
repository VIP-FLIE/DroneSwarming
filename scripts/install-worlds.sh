#!/usr/bin/env bash
# =============================================================================
# Puts this repo's worlds where PX4 will find them.
#
#   ./scripts/install-worlds.sh            link worlds/ into PX4
#   ./scripts/install-worlds.sh --list     show what PX4 can load right now
#   ./scripts/install-worlds.sh --remove   take this repo's links back out
#
# Links two things: every worlds/*.sdf into PX4's worlds directory, and every
# worlds/models/<name>/ into PX4's models directory, which is what makes
# <uri>model://<name></uri> resolve inside your world.
#
# Safe to re-run, and you have to re-run it after rebuilding the dev container,
# because PX4 lives in the container image and is recreated with it.
#
# WHY THIS EXISTS
# ---------------
# PX4 does not take a path to a world. Its startup script does the equivalent
# of
#
#     gz sim -r -s "$PX4_GZ_WORLDS/$PX4_GZ_WORLD.sdf"
#
# and $PX4_GZ_WORLDS is hardcoded to PX4's own worlds directory by the
# generated build/px4_sitl_default/rootfs/gz_env.sh, which overwrites whatever
# you set. See ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim line 52 in the
# pinned v1.16.1. So the only way in is to have a file sitting in that
# directory. This symlinks ours there rather than copying, so editing a world
# in this repo takes effect on the next launch with no reinstall.
#
# Models go the same way for the same reason. Both of PX4's directories are
# already on GZ_SIM_RESOURCE_PATH in the dev container and in a native install,
# so linking into them needs no environment change and no image rebuild.
#
# Full guide: docs/WORLDS.md
# =============================================================================
set -o pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SRC_DIR="$REPO_ROOT/worlds"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
DEST_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
MODEL_SRC_DIR="$SRC_DIR/models"
MODEL_DEST_DIR="$PX4_DIR/Tools/simulation/gz/models"

MODE=install
case "${1:-}" in
    --list)   MODE=list ;;
    --remove) MODE=remove ;;
    -h|--help)
        sed -n '3,9p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
        exit 0 ;;
    "") ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
esac

if [ ! -d "$DEST_DIR" ]; then
    echo "No PX4 worlds directory at $DEST_DIR" >&2
    echo "PX4 is not installed yet. Dev container: rebuild it. Native: run" >&2
    echo "./scripts/native/install.sh" >&2
    exit 1
fi

# Ours are symlinks, PX4's own are regular files. That is the whole test.
if [ "$MODE" = list ]; then
    printf '\nWorlds PX4 can load, from %s\n\n' "$DEST_DIR"
    for f in "$DEST_DIR"/*.sdf; do
        [ -e "$f" ] || continue
        name=$(basename "$f" .sdf)
        if [ -L "$f" ]; then
            printf '  %-20s ours    -> %s\n' "$name" "$(readlink "$f")"
        else
            printf '  %-20s PX4\n' "$name"
        fi
    done
    printf '\nModels %s can resolve as model://<name>, ours only\n\n' "$MODEL_DEST_DIR"
    found=0
    for d in "$MODEL_DEST_DIR"/*; do
        [ -L "$d" ] || continue
        printf '  %-20s -> %s\n' "$(basename "$d")" "$(readlink "$d")"
        found=1
    done
    [ $found -eq 0 ] && printf '  none\n'
    printf '\nFly one with:  PX4_GZ_WORLD=<name> drone-sim\n\n'
    exit 0
fi

if [ "$MODE" = remove ]; then
    removed=0
    for f in "$DEST_DIR"/*.sdf; do
        [ -L "$f" ] || continue
        case "$(readlink "$f")" in
            "$SRC_DIR"/*) rm "$f"; echo "removed $(basename "$f")"; removed=$((removed+1)) ;;
        esac
    done
    for d in "$MODEL_DEST_DIR"/*; do
        [ -L "$d" ] || continue
        case "$(readlink "$d")" in
            "$MODEL_SRC_DIR"/*) rm "$d"; echo "removed model $(basename "$d")"; removed=$((removed+1)) ;;
        esac
    done
    echo "$removed link(s) removed."
    exit 0
fi

# --- install ----------------------------------------------------------------
if [ ! -d "$SRC_DIR" ]; then
    echo "No worlds directory at $SRC_DIR" >&2
    exit 1
fi

shopt -s nullglob
worlds=("$SRC_DIR"/*.sdf)
if [ ${#worlds[@]} -eq 0 ]; then
    echo "No .sdf files in $SRC_DIR, nothing to do."
    exit 0
fi

status=0
for src in "${worlds[@]}"; do
    name=$(basename "$src" .sdf)
    dest="$DEST_DIR/$name.sdf"

    # PX4 ships ten worlds. Shadowing one of them with ours would be a
    # confusing thing to do silently, so refuse and make them rename.
    if [ -e "$dest" ] && [ ! -L "$dest" ]; then
        echo "SKIP  $name.sdf is one of PX4's own worlds. Rename yours." >&2
        status=1
        continue
    fi

    # The world's name attribute has to match the file name, or PX4 starts
    # Gazebo, then waits 30 seconds for a world by the wrong name and exits.
    # grep -v '' drops empty matches, so a <world name=""> written inside a
    # comment in the file's own header does not get read as the real name.
    declared=$(sed -n 's/.*<world[[:space:]][^>]*name=["'"'"']\([^"'"'"']*\)["'"'"'].*/\1/p' "$src" \
        | grep -v '^$' | head -1)
    if [ -n "$declared" ] && [ "$declared" != "$name" ]; then
        echo "SKIP  $name.sdf declares <world name=\"$declared\">." >&2
        echo "      PX4 looks the world up by file name, so these have to match." >&2
        status=1
        continue
    fi

    ln -sfn "$src" "$dest"
    echo "linked $name.sdf"
done

# Models are directories, and the one that identifies them is model.config.
# Anything else in worlds/models is not a model, so it is left alone.
for src in "$MODEL_SRC_DIR"/*/; do
    [ -f "$src/model.config" ] || continue
    src=${src%/}
    name=$(basename "$src")
    dest="$MODEL_DEST_DIR/$name"

    if [ -e "$dest" ] && [ ! -L "$dest" ]; then
        echo "SKIP  model $name is one of PX4's own. Rename yours." >&2
        status=1
        continue
    fi

    ln -sfn "$src" "$dest"
    echo "linked model $name"
done

if [ $status -eq 0 ]; then
    printf '\nFly one with:  PX4_GZ_WORLD=<name> drone-sim\n'
    printf 'See them all:  ./scripts/install-worlds.sh --list\n\n'
fi
exit $status
