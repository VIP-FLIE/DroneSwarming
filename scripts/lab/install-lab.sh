#!/usr/bin/env bash
# =============================================================================
# Turns an Ubuntu 24.04 machine with an NVIDIA card into the shared lab host.
#
#   sudo ./scripts/lab/install-lab.sh                     the system setup
#   sudo ./scripts/lab/install-lab.sh --add-user alice --slot 1
#   sudo ./scripts/lab/install-lab.sh --harden-ssh
#        ./scripts/lab/install-lab.sh --check             changes nothing
#
# This is Part 1.4, 1.6 and 1.7 of docs/LAB-SERVER.md, which is where the
# reasons live. Run it once, then --add-user once per person.
#
# What it does NOT do, on purpose:
#
#   the stack      ./scripts/native/install.sh already installs ROS 2, Gazebo
#                  and the XRCE-DDS Agent system-wide. Run that first
#   GRUB, sleep    one-line changes with consequences if they are wrong on
#                  your machine. docs/LAB-SERVER.md 1.1 has them to copy
#   Tailscale      two commands, and whether to run them at all is a
#                  conversation with whoever runs your network. 1.3
#   SSH keys       each person generates their own and sends you the public
#                  half. A script that handled keys would be a script that
#                  encouraged mailing private ones around
#
# Safe to re-run. Every step checks whether it is already done.
#
# NOTE: no `set -u`. Kept consistent with the other scripts in this repo,
# which cannot use it because ROS 2 setup files expand unset variables.
# =============================================================================
set -o pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
DOCKERFILE="$REPO_ROOT/.devcontainer/Dockerfile"
TEAM_GROUP=flie
SSH_GROUP=flie-ssh
SHARED_DIR=/srv/flie/shared
SSHD_DROPIN=/etc/ssh/sshd_config.d/10-flie.conf
START=$(date +%s)

stage() { printf '\n\033[1;36m==> %s\033[0m  (%ds elapsed)\n' "$1" "$(( $(date +%s) - START ))"; }
info()  { printf '    %s\n' "$*"; }
good()  { printf '    \033[32mok\033[0m      %s\n' "$*"; }
warn()  { printf '    \033[33mWARNING\033[0m %s\n' "$*"; }
bad()   { printf '    \033[31mMISSING\033[0m %s\n' "$*"; }
die()   { printf '\n\033[31mSTOPPED\033[0m %s\n\n' "$*" >&2; exit 1; }

pin() { sed -n "s/^ARG $1=\(.*\)$/\1/p" "$DOCKERFILE" | head -1; }

MODE=install
NEW_USER=""
NEW_SLOT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --check)      MODE=check ;;
        --harden-ssh) MODE=ssh ;;
        --add-user)   MODE=adduser; NEW_USER="$2"; shift ;;
        --slot)       NEW_SLOT="$2"; shift ;;
        -h|--help)
            # Line 3 onward, so the range does not terminate on the banner's
            # own opening ===== line.
            sed -n '3,/^# ==========/p' "$0" | sed 's/^# \?//;$d'
            exit 0 ;;
        *) die "unknown option: $1" ;;
    esac
    shift
done

need_root() {
    [ "$(id -u)" -eq 0 ] || die "This needs root. Run it with sudo."
}

# ---------------------------------------------------------------------------
# --check. Reads only, so it needs no root and is the thing to run when
# something has stopped working.
# ---------------------------------------------------------------------------
run_check() {
    local fails=0
    stage "Machine"
    . /etc/os-release
    if [ "${VERSION_CODENAME:-}" = "noble" ]; then
        good "$PRETTY_NAME"
    else
        bad "$PRETTY_NAME is not noble. ROS 2 Jazzy and Gazebo Harmonic need 24.04."
        fails=$((fails+1))
    fi
    info "$(nproc) cores, $(free -g | awk '/^Mem:/{print $2}') GB RAM"
    info "$(df -h /home --output=avail 2>/dev/null | tail -1 | tr -d ' ') free on /home, about 7 GB per user"

    stage "GPU"
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        good "$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader)"
    else
        bad "no working nvidia-smi. Everything falls back to software rendering."
        fails=$((fails+1))
    fi
    systemctl is-active --quiet nvidia-persistenced \
      && good "nvidia-persistenced running" \
      || { bad "nvidia-persistenced not running. The card may deinitialise when idle."; fails=$((fails+1)); }
    if [ -e /dev/dri/renderD128 ]; then
        good "/dev/dri/renderD128 is $(stat -c '%U:%G %a' /dev/dri/renderD128)"
    else
        bad "no /dev/dri/renderD128, so VirtualGL's EGL back end has no device"
        fails=$((fails+1))
    fi

    stage "Rendering"
    command -v vglrun >/dev/null 2>&1 \
      && good "VirtualGL $(vglrun -v 2>&1 | sed -n 's/.*VirtualGL v\([0-9.]*\).*/\1/p' | head -1)" \
      || { bad "vglrun not installed"; fails=$((fails+1)); }
    for wrapper in /usr/local/bin/gz /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz; do
        if [ -f "$wrapper" ] && grep -q 'vglrun -d egl' "$wrapper" 2>/dev/null; then
            good "$wrapper goes through VirtualGL"
        elif [ -e "$wrapper" ]; then
            bad "$wrapper exists but does not call vglrun. GPU silently lost."
            fails=$((fails+1))
        else
            info "$wrapper not present"
        fi
    done

    stage "Display"
    [ -x /opt/TurboVNC/bin/vncserver ] \
      && good "TurboVNC" || { bad "TurboVNC not installed"; fails=$((fails+1)); }
    for prog in websockify openbox xterm glxinfo; do
        command -v "$prog" >/dev/null 2>&1 \
          && good "$prog" || { bad "$prog not installed"; fails=$((fails+1)); }
    done
    [ -d /usr/share/novnc ] \
      && good "noVNC web assets" || { bad "/usr/share/novnc missing"; fails=$((fails+1)); }

    stage "Accounts"
    getent group "$TEAM_GROUP" >/dev/null && good "group $TEAM_GROUP" \
      || { bad "group $TEAM_GROUP does not exist"; fails=$((fails+1)); }
    getent group "$SSH_GROUP" >/dev/null && good "group $SSH_GROUP" \
      || { bad "group $SSH_GROUP does not exist"; fails=$((fails+1)); }
    local members
    members=$(getent group "$SSH_GROUP" | cut -d: -f4)
    info "may SSH in: ${members:-nobody yet}"
    [ -d "$SHARED_DIR" ] && good "$SHARED_DIR is $(stat -c '%U:%G %a' "$SHARED_DIR")" \
      || info "$SHARED_DIR does not exist yet"

    stage "The stack"
    [ -f /opt/ros/jazzy/setup.bash ] && good "ROS 2 Jazzy" \
      || { bad "ROS 2 Jazzy not installed. Run ./scripts/native/install.sh"; fails=$((fails+1)); }
    command -v MicroXRCEAgent >/dev/null 2>&1 && good "XRCE-DDS Agent" \
      || { bad "XRCE-DDS Agent not installed"; fails=$((fails+1)); }

    echo
    if [ "$fails" -eq 0 ]; then
        printf '\033[32mLab host looks ready.\033[0m Next: --add-user for each person.\n\n'
    else
        printf '\033[31m%d thing(s) missing.\033[0m Run: sudo %s\n\n' \
            "$fails" "$(realpath --relative-to="$PWD" "$0" 2>/dev/null || echo "$0")"
    fi
    return "$fails"
}

# ---------------------------------------------------------------------------
# --add-user
# ---------------------------------------------------------------------------
run_adduser() {
    need_root
    [ -n "$NEW_USER" ] || die "--add-user needs a name."
    [ -n "$NEW_SLOT" ] || die "--add-user needs --slot N. See the table in docs/LAB-SERVER.md."
    case "$NEW_SLOT" in
        [0-9]) ;;
        *) die "--slot must be a single digit 0-9, got '$NEW_SLOT'." ;;
    esac

    # Two people on one slot is the failure this whole scheme exists to
    # prevent, so it is caught here rather than in the simulator.
    local clash
    clash=$(grep -rlsE "^[[:space:]]*export[[:space:]]+DRONE_SLOT=$NEW_SLOT\b" \
              /home/*/.bashrc 2>/dev/null \
            | xargs -r -n1 dirname | xargs -r -n1 basename \
            | grep -v "^$NEW_USER$")
    [ -n "$clash" ] && die "slot $NEW_SLOT is already used by: $clash
Pick a free one from the table in docs/LAB-SERVER.md, or change theirs first."

    stage "Account for $NEW_USER, slot $NEW_SLOT"
    getent group "$TEAM_GROUP" >/dev/null || groupadd "$TEAM_GROUP"
    getent group "$SSH_GROUP"  >/dev/null || groupadd --system "$SSH_GROUP"

    if id "$NEW_USER" >/dev/null 2>&1; then
        info "user $NEW_USER already exists"
    else
        # --disabled-password: there is no password to guess, and no password
        # to share. Keys only.
        adduser --disabled-password --gecos "" "$NEW_USER" || die "adduser failed"
        good "created $NEW_USER with no password"
    fi

    # render and video are what let them reach the GPU with no X session of
    # their own, which is how VirtualGL's EGL back end works. Deliberately no
    # sudo group: nothing in the normal workflow needs root.
    usermod -aG "$TEAM_GROUP,$SSH_GROUP,render,video" "$NEW_USER"
    good "groups: $(id -nG "$NEW_USER")"

    local home; home=$(getent passwd "$NEW_USER" | cut -d: -f6)
    install -d -o "$NEW_USER" -g "$NEW_USER" -m 700 "$home/.ssh"
    touch "$home/.ssh/authorized_keys"
    chown "$NEW_USER:$NEW_USER" "$home/.ssh/authorized_keys"
    chmod 600 "$home/.ssh/authorized_keys"
    good "$home/.ssh ready"

    # Written now rather than left to them, because a slot they forget to set
    # is a slot that silently collides with slot 0.
    if grep -qsE '^[[:space:]]*export[[:space:]]+DRONE_SLOT=' "$home/.bashrc"; then
        sed -i "s/^[[:space:]]*export[[:space:]]\+DRONE_SLOT=.*/export DRONE_SLOT=$NEW_SLOT/" "$home/.bashrc"
        good "updated DRONE_SLOT=$NEW_SLOT in their ~/.bashrc"
    else
        cat >> "$home/.bashrc" <<BEOF

# Your slot on this machine. Everything that keeps your simulation separate
# from everyone else's derives from it. See docs/LAB-SERVER.md.
export DRONE_SLOT=$NEW_SLOT
BEOF
        good "set DRONE_SLOT=$NEW_SLOT in their ~/.bashrc"
    fi

    install -d -o root -g "$TEAM_GROUP" -m 2775 "$SHARED_DIR"
    command -v setfacl >/dev/null 2>&1 && setfacl -d -m "g:$TEAM_GROUP:rwx" "$SHARED_DIR" 2>/dev/null

    cat <<NEXTEOF

Now, in this order:

  1. $NEW_USER generates a key ON THEIR OWN MACHINE and sends you the
     PUBLIC half only. Never the private half, and never one you made:

         ssh-keygen -t ed25519 -C "$NEW_USER@udel.edu"
         cat ~/.ssh/id_ed25519.pub

  2. You paste it in, then Ctrl-D:

         sudo -u $NEW_USER tee -a $home/.ssh/authorized_keys

  3. They log in and install their half of the stack. It needs no sudo,
     because you already installed the system-wide half:

         git clone https://github.com/VIP-FLIE/DroneSwarming.git
         cd DroneSwarming
         ./scripts/native/install.sh --skip-apt
         echo "source \$PWD/scripts/native/env.sh" >> ~/.bashrc

  4. They set a VNC password and start their desktop:

         /opt/TurboVNC/bin/vncpasswd
         ./scripts/lab/lab-desktop

  5. Write "$NEW_USER" into the slot $NEW_SLOT row of the table in
     docs/LAB-SERVER.md and commit it. That table is the only place the
     mapping is recorded.

Their ports, for the ~/.ssh/config on their laptop:

     LocalForward $((5910 + NEW_SLOT)) localhost:$((5910 + NEW_SLOT))
     LocalForward $((6080 + NEW_SLOT)) localhost:$((6080 + NEW_SLOT))

NEXTEOF
}

# ---------------------------------------------------------------------------
# --harden-ssh. Separate and opt-in, because getting it wrong locks you out.
# ---------------------------------------------------------------------------
run_ssh() {
    need_root
    stage "SSH"
    getent group "$SSH_GROUP" >/dev/null || groupadd --system "$SSH_GROUP"

    if [ -f "$SSHD_DROPIN" ]; then
        info "$SSHD_DROPIN already exists, leaving it alone:"
        sed 's/^/        /' "$SSHD_DROPIN"
        return 0
    fi

    # A key on the admin account first. Writing PasswordAuthentication no on a
    # machine with no key installed anywhere locks everybody out of a machine
    # that is, by definition, not in front of you.
    local keyed=0
    for home in /home/*/ /root/; do
        [ -s "${home}.ssh/authorized_keys" ] && keyed=1
    done
    [ "$keyed" -eq 1 ] || die "No authorized_keys anywhere on this machine has a key in it.
Turning off password authentication now would lock you out. Install your own
key first, then run this again."

    cat > "$SSHD_DROPIN" <<SEOF
# Written by scripts/lab/install-lab.sh --harden-ssh. docs/LAB-SERVER.md 1.2.
#
# NOTE: Ubuntu 24.04 socket-activates sshd, so Port and ListenAddress here are
# ignored. Those live in the ssh.socket unit now. Authentication still reads
# from here.
PermitRootLogin no
PasswordAuthentication no
KbdInteractiveAuthentication no
AllowGroups $SSH_GROUP

# A long headless swarm run with an idle terminal is exactly the shape of
# connection a NAT or a campus firewall times out, and losing the session kills
# what is attached to it. Use tmux as well.
ClientAliveInterval 60
ClientAliveCountMax 10
SEOF
    good "wrote $SSHD_DROPIN"

    if sshd -t; then
        good "sshd config parses"
        systemctl restart ssh && good "sshd restarted"
        cat <<'WEOF'

    KEEP THIS SESSION OPEN until you have proved a new one works:

        ssh -o BatchMode=yes you@this-machine true && echo "still in"

    If it fails, you still have this shell to undo it with.

WEOF
    else
        rm -f "$SSHD_DROPIN"
        die "sshd rejected that config, so it was removed and nothing changed."
    fi
}

# ---------------------------------------------------------------------------
# The system setup.
# ---------------------------------------------------------------------------
run_install() {
    need_root
    . /etc/os-release
    [ "${VERSION_CODENAME:-}" = "noble" ] \
      || die "This is $PRETTY_NAME. The lab host has to be Ubuntu 24.04 (noble)
or a derivative on the same base, because that is what ROS 2 Jazzy and Gazebo
Harmonic publish packages for."

    stage "Groups"
    getent group "$TEAM_GROUP" >/dev/null || { groupadd "$TEAM_GROUP"; good "created $TEAM_GROUP"; }
    getent group "$SSH_GROUP"  >/dev/null || { groupadd --system "$SSH_GROUP"; good "created $SSH_GROUP"; }
    install -d -o root -g "$TEAM_GROUP" -m 2775 "$SHARED_DIR"
    good "$SHARED_DIR, group $TEAM_GROUP, setgid so new files inherit it"

    stage "GPU"
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        good "$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader)"
        # Without this the driver can deinitialise the card when no client has
        # it open, which on a machine with nobody at the console is most of the
        # time. The first vglrun after that pays to bring it back, and
        # sometimes fails instead.
        systemctl enable --now nvidia-persistenced >/dev/null 2>&1 \
          && good "nvidia-persistenced enabled" \
          || warn "could not enable nvidia-persistenced, check: systemctl status nvidia-persistenced"
    else
        warn "no working NVIDIA driver found."
        warn "Everything below still installs, but rendering will be llvmpipe."
    fi

    stage "Display and rendering packages"
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    # Same set the container installs, for the same reasons. openbox rather
    # than a desktop environment: this exists to hold a Gazebo window.
    apt-get install -y --no-install-recommends \
        novnc websockify openbox xterm \
        x11-utils x11-xserver-utils xauth dbus-x11 \
        libgl1-mesa-dri mesa-utils \
        libxcb-cursor0 libxkbcommon-x11-0 \
        acl curl ca-certificates gnupg wget \
      || die "apt failed on the display packages"
    ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html
    good "noVNC, websockify, openbox"

    stage "VirtualGL"
    # Pinned from the Dockerfile, the one place this repo sets versions, so the
    # lab machine renders through the same code the container does.
    VGL_VERSION=$(pin VIRTUALGL_VERSION)
    [ -n "$VGL_VERSION" ] || die "could not read VIRTUALGL_VERSION from $DOCKERFILE"
    HAVE_VGL=$(dpkg-query -W -f='${Version}' virtualgl 2>/dev/null | cut -d- -f1)
    if [ "$HAVE_VGL" = "$VGL_VERSION" ]; then
        good "VirtualGL $VGL_VERSION already installed"
    else
        ARCH=$(dpkg --print-architecture)
        info "installing VirtualGL $VGL_VERSION (${HAVE_VGL:-none} present)"
        curl -sfL -o /tmp/virtualgl.deb \
          "https://github.com/VirtualGL/virtualgl/releases/download/${VGL_VERSION}/virtualgl_${VGL_VERSION}_${ARCH}.deb" \
          || die "could not download VirtualGL $VGL_VERSION for $ARCH"
        apt-get install -y --no-install-recommends /tmp/virtualgl.deb || die "VirtualGL install failed"
        rm -f /tmp/virtualgl.deb
        good "VirtualGL $VGL_VERSION"
    fi

    stage "TurboVNC"
    if [ -x /opt/TurboVNC/bin/vncserver ]; then
        good "already installed"
    else
        wget -qO- https://packagecloud.io/dcommander/turbovnc/gpgkey \
          | gpg --dearmor > /etc/apt/trusted.gpg.d/TurboVNC.gpg \
          || die "could not fetch the TurboVNC signing key"
        wget -qO /etc/apt/sources.list.d/TurboVNC.list \
          https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list \
          || die "could not fetch the TurboVNC apt list"
        apt-get update -qq
        apt-get install -y turbovnc || die "TurboVNC install failed"
        good "TurboVNC $(dpkg-query -W -f='${Version}' turbovnc 2>/dev/null)"
    fi

    stage "VirtualGL wrappers"
    # PATH ordering alone is not enough, and this bit the project for real.
    # ROS 2 ships its own gz-tools and `source /opt/ros/jazzy/setup.bash`
    # prepends that directory, so every script that sources ROS puts ROS's gz
    # in front of /usr/local/bin and silently drops VirtualGL. You lose the GPU
    # with nothing reporting an error. Wrapping both entry points makes
    # acceleration independent of PATH order.
    write_wrapper() {
        local real="$1" path="$2"
        cat > "$path" <<WEOF
#!/usr/bin/env bash
# Written by scripts/lab/install-lab.sh. Renders on the GPU through VirtualGL's
# EGL back end, which needs no X server of its own. DRONE_VGL=0 bypasses it.
REAL=$real
if [ "\${DRONE_VGL:-1}" = "1" ] && command -v vglrun >/dev/null 2>&1; then
    exec vglrun -d egl "\$REAL" "\$@"
fi
exec "\$REAL" "\$@"
WEOF
        chmod +x "$path"
    }

    if [ -x /usr/bin/gz ]; then
        write_wrapper /usr/bin/gz /usr/local/bin/gz
        good "/usr/local/bin/gz wraps the apt gz"
    else
        warn "no /usr/bin/gz. Run ./scripts/native/install.sh first."
    fi

    ROS_GZ=/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
    if [ -f "$ROS_GZ" ]; then
        if grep -q 'vglrun -d egl' "$ROS_GZ" 2>/dev/null; then
            good "ROS's vendored gz already wrapped"
        else
            # Move the real one aside once. Re-running must not wrap a wrapper.
            [ -f "${ROS_GZ}.real" ] || mv "$ROS_GZ" "${ROS_GZ}.real"
            write_wrapper "${ROS_GZ}.real" "$ROS_GZ"
            good "wrapped ROS's vendored gz"
        fi
    else
        warn "no ROS vendored gz at $ROS_GZ. Run ./scripts/native/install.sh first."
    fi

    stage "Check"
    run_check
    RC=$?

    cat <<'DONEEOF'

    Still to do by hand, because they are judgement calls rather than steps.
    docs/LAB-SERVER.md has each one:

      1.1  default to Ubuntu in GRUB, mask sleep, fix the dual-boot clock
      1.3  Tailscale, after talking to whoever runs your network
      1.2  sudo ./scripts/lab/install-lab.sh --harden-ssh
      1.4  sudo ./scripts/lab/install-lab.sh --add-user NAME --slot N

    Then prove the GPU is actually in the path, as a normal user, not root:

      vglrun -d egl glxinfo | grep 'OpenGL renderer'

    You want the card named. llvmpipe means software rendering, and the usual
    cause is that the user is not in the render and video groups yet, or has
    not logged out and back in since they were added.

DONEEOF
    return "$RC"
}

case "$MODE" in
    check)   run_check ;;
    adduser) run_adduser ;;
    ssh)     run_ssh ;;
    install) run_install ;;
esac
