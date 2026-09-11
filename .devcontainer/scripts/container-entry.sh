#!/usr/bin/env bash
# =============================================================================
# The container's main command. Brings the GUI up, then stays alive.
#
# WHY THIS EXISTS
# ---------------
# The browser desktop used to be started by `postStartCommand` in
# devcontainer.json. That is a dev-container lifecycle hook, and when a hook
# does not run (attaching to an already-running container, a hook that errored
# earlier in the chain, a tool that does not implement the hook at all) you get
# the exact failure this was reported with: VS Code shows 6080 and 5901
# forwarded, with nothing listening behind them and no error anywhere.
#
# Starting the desktop from the container's own command removes that class of
# failure. If the container is up, the desktop has been started, because
# starting it is literally what the container runs.
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
