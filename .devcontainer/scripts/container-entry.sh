#!/usr/bin/env bash
# =============================================================================
# The container's main command. Brings the GUI up, then stays alive.
#
# WHY THIS EXISTS
# ---------------
# The desktop is started here rather than from `postStartCommand` in
# devcontainer.json. That is a dev-container lifecycle hook, and a hook can
# fail to run without reporting anything: when you attach to an
# already-running container, when an earlier hook in the chain errored, or
# when the tool opening the container does not implement hooks at all. The
# result is 6080 and 5901 published with nothing listening behind them.
#
# Starting the desktop from the container's own command removes that class of
# failure. If the container is up, the desktop has been started, because
# starting it is what the container runs.
# =============================================================================
set -o pipefail

# Same preference order: the repo copy wins, so edits do not need a rebuild.
for sd in /workspaces/DroneSwarming/.devcontainer/scripts/start-display.sh \
          /opt/drone-vip/scripts/start-display.sh; do
    if [ -r "$sd" ]; then
        bash "$sd" || true
        break
    fi
done

# Keep PID 1 alive for the editor to attach to.
exec sleep infinity
