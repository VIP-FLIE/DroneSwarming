# The lab machine

> **Convention:** `your_username` is a placeholder. Replace it with the actual
> username of the person you are onboarding.

How the Evans 153 machine is set up so the whole team can SSH in, fly the
simulator, and see it, from macOS, Windows or Linux.

This is the third way to run the stack. [SETUP.md](SETUP.md) is the dev
container, [NATIVE-SETUP.md](NATIVE-SETUP.md) is a native install on your own
Ubuntu box, and this is a native install on one shared machine that everybody
logs into. The stack itself is the native one, installed exactly as
NATIVE-SETUP.md describes. Everything extra here exists for one of two reasons:
several people use the machine at once, and nobody is sitting in front of it.

Two audiences. Part 1 is the administrator, once. Part 2 is a team member
getting an account. Part 3 is a team member on a normal day.

## The problem this solves

Gazebo needs a GPU and ROS 2 Jazzy needs Ubuntu 24.04. The lab machine has
both. Most of the team is on macOS, which has neither.

X11 forwarding over SSH does not work here. It ships every OpenGL call across
the network to be rendered on the client, so it needs a working X server on the
Mac (XQuartz), it needs indirect GLX, and Gazebo's ogre2 render engine does not
survive the trip. Instead: render on the lab machine's NVIDIA card, compress the
finished frames, and send pixels.

| | Where GL runs | Client needs | Gazebo |
|---|---|---|---|
| `ssh -X` | Your laptop | XQuartz or VcXsrv, indirect GLX | Does not work |
| VNC of a virtual desktop | Lab machine, software rendering | Browser or VNC client | Works, slow |
| VNC of a virtual desktop, VirtualGL | Lab machine, NVIDIA card | Browser or VNC client | Works |

The last row is what the dev container already does inside Docker, with Xvfb
and x11vnc. The lab machine does the same thing with TurboVNC in place of Xvfb
plus x11vnc, because TurboVNC is built for exactly this job and is faster on 3D
content. See [What differs from the container](#what-differs-from-the-container).

## Decisions

**Separate accounts, not a shared root login.** Nine reasons, and the first
four are specific to this stack rather than general hygiene.

| | |
|---|---|
| Home directories | Every user needs their own `~/PX4-Autopilot`, `~/ros2_ws`, `~/.ros`, `~/.gz` and `~/.config`. PX4 SITL writes its rootfs under `build/px4_sitl_default/rootfs/<instance>` at runtime, so two people sharing one tree corrupt each other's vehicles |
| `pkill` | [SWARM.md](../swarm/SWARM.md) tells you to run `pkill -f 'gz sim'` when a server survives a launch. As a normal user that signals only your own processes. As root it kills every simulation on the machine, mid-flight |
| Displays | Each user needs their own X display, VNC port and `~/.vnc`. One shared account means one desktop that everybody fights over |
| Slots | The whole isolation scheme below keys off who you are logged in as |
| Attribution | When the GPU is pinned and real-time factor drops, `w`, `last` and `nvidia-smi` have to name whose run is doing it |
| Revocation | Somebody graduates, you run `userdel`. With a shared login you rotate one key and redistribute it to everyone still here |
| Blast radius | A mistyped `rm -rf` in a home directory costs one person a rebuild |
| Audit | `journalctl _COMM=sudo` is only useful if sudo is tied to a person |
| Habit | Working as root for months is a habit that follows people to machines where it matters more |

Team members get **no sudo**. They do not need it. The parts of `install.sh`
that want root install system packages, and those are installed once by the
administrator in Part 1. Team members run it with `--skip-apt`.

**Reach the machine over Tailscale.** The campus network does not accept
inbound connections from outside, so without something the team has to be on
campus Wi-Fi or on the UD VPN to reach the machine at all. Tailscale gives the
machine a stable address and a name that work from anywhere, through NAT, with
no firewall changes and nothing listening on the public internet. The free
Personal plan covers 6 users and unlimited devices, which fits a team of five
with one spare.

On a university network, two constraints apply:

- Bring it up with whoever administers the lab and the network first. An
  overlay network on a campus machine is the kind of thing network security
  wants to know about, and it is a short conversation if you have it before
  rather than after.
- Do not advertise subnet routes and do not make it an exit node. Without
  those, Tailscale exposes this one host and nothing else on the campus
  network. With them it becomes a way around the campus firewall for the whole
  subnet, which is a different proposition entirely.

The fallback, if Tailscale is not allowed, is the UD VPN plus the machine's
campus hostname. It costs nothing to set up and everything else in this
document is unchanged. Part 1.3 covers both.

## The isolation problem

Several fixed resources in the stack are hardcoded to one value. On your own
laptop that is fine. On a shared machine the second person to start hits them.

Every row below was read out of the pinned PX4 v1.16.1 tree or this repo, not
recalled.

| Resource | Fixed at | Set in | Second user gets | Fix |
|---|---|---|---|---|
| ROS 2 domain | `ROS_DOMAIN_ID=0` | `scripts/native/env.sh` | Both people's nodes on one domain. A node subscribes to whichever vehicle answers first | `ROS_DOMAIN_ID` = slot |
| ROS 2 discovery | `LOCALHOST` | `scripts/native/env.sh` | No help at all. It stops discovery crossing the subnet, and on one machine everybody already is localhost | as above |
| Gazebo world | any running world | `px4-rc.gzsim`, `get_gz_world()` greps `gz topic -l` for `/world/*/clock` | Their PX4 finds your world, joins it, and spawns their drone into your flight. `allow_renaming: false`, so a name clash fails silently | `GZ_PARTITION` = username |
| PX4 GCS MAVLink port | `18570 + px4_instance` | `px4-rc.mavlink` | `mavlink start -u 18571` cannot bind. No GCS link, so `NAV_DLL_ACT 2` refuses to arm and the vehicle reports `Arming denied: Resolve system health failures first` | per-user instance block |
| XRCE-DDS agent | `8888` | `MicroXRCEAgent udp4 -p 8888` | Cannot bind. No `/fmu/` topics | `8888 + slot`, and `PX4_UXRCE_DDS_PORT` to match |
| `drone-gcs` listener | `udpin:0.0.0.0:14550` | `.devcontainer/scripts/drone-gcs` | Cannot bind | `14550 + slot` |
| `drone-gcs` heartbeats | `18570` through `18579` | same file | Heartbeats the other user's vehicles as well as its own | derive from the instance block |
| Raw SITL command, one vehicle | instance `0` | `make px4_sitl`, which has no way to pass `-i` | Everybody's single-drone run is instance 0, so everybody wants port 18570 | run the binary with `-i` above slot 0 |
| Smoke test scratch files | `/tmp/sitl.log` | `scripts/native/smoke-test.sh` | `/tmp` is sticky, so the second person cannot write it and reads the first person's run as their own | per-user scratch directory |
| X display | `:1` | container only | Nothing to collide with natively, but each user still needs their own | `:10 + slot` |

The `Arming denied` message is the same one that appears when no `drone-gcs`
runs at all. Check `drone-gcs` first: if it is up and healthy, the cause is
somebody else's PX4 holding the port.

### Slots

Each person gets a slot number, 0 through 9, assigned once. Everything above
derives from it. `scripts/native/env.sh` does the deriving, so the only thing
anyone sets by hand is the slot:

```bash
export DRONE_SLOT=1            # in ~/.bashrc, above the line that sources env.sh
```

Ten swarm instances per slot, which is one more drone than this repo has ever
flown, plus instance 0 for slot 0.

| Slot | User | `ROS_DOMAIN_ID` | `GZ_PARTITION` | PX4 instances | PX4 GCS ports | XRCE agent | `drone-gcs` | Display | VNC | noVNC |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | dan | 0 | dan | 0-10 | 18570-18580 | 8888 | 14550 | `:10` | 5910 | 6080 |
| 1 | | 1 | | 11-20 | 18581-18590 | 8889 | 14551 | `:11` | 5911 | 6081 |
| 2 | | 2 | | 21-30 | 18591-18600 | 8890 | 14552 | `:12` | 5912 | 6082 |
| 3 | | 3 | | 31-40 | 18601-18610 | 8891 | 14553 | `:13` | 5913 | 6083 |
| 4 | | 4 | | 41-50 | 18611-18620 | 8892 | 14554 | `:14` | 5914 | 6084 |

Fill in the names as accounts are created. `install-lab.sh --add-user` refuses
a slot another account already has, but this table is the only place the
mapping is written down for a human, so keep it current and commit it.

Slot 0 is the single-user setup exactly as it has always been. Nobody on their
own laptop sets `DRONE_SLOT`, and nothing about their simulation changes.

Three notes on the numbering.

PX4 derives the topic namespace from the instance, so slot 1's first drone is
`/px4_11/fmu/...`, not `/px4_1/`. Nothing in this repo hardcodes that, because
the launch files pass `px4_ns` as a parameter. Anything you write by hand
should read it rather than assume it.

Slot 0 also owns instance 0, which is what the raw SITL command without `-i`
runs: one vehicle on bare `/fmu/` with no namespace, the way a real aircraft
publishes. No other slot can produce instance 0, so it costs nobody anything.

Ten instances starting at base+1 rather than base also fixed a bug that was
already there. `drone-gcs` covered ports 18570 through 18579, which is
instances 0 through 9, while the launch files number vehicles from 1. So
`count:=10` produced instance 10 on port 18580, which nothing heartbeated: the
tenth drone of a ten-drone swarm never got a link, never satisfied its datalink
failsafe, and sat on the pad reporting `Arming denied` while the other nine
flew. `scripts/test-drone-gcs.py` pins it now.

One thing the blocks do not separate. PX4's mavlink module defaults its
*remote* port to 14550 (`DEFAULT_REMOTE_PORT_UDP`, `mavlink_main.h`), and
`px4-rc.mavlink` does not pass `-o` to change it. With `MAV_BROADCAST` on, a
vehicle nobody has contacted yet broadcasts toward that port, which is slot 0's
listen port. Slot 0's station may therefore count a vehicle that is not its own
for a few seconds. It resolves itself once that vehicle's real station
heartbeats its instance directly, and nothing mis-flies. Slot 0 seeing a
transient extra drone in its tally is expected rather than a fault.

`px4-rc.mavlink` collapses the offboard *remote* port to 14549 for every
instance above 9, so all slots above 0 share it. Nothing in this repo binds
that port. It matters only if somebody starts using MAVSDK or QGroundControl's
offboard link, at which point those slots need a scheme of their own.

---

# Part 1. Administrator setup

Once, by whoever has root. Budget an afternoon.

## 1.0 Inventory the machine first

Write these down. Everything after this is sized off them.

```bash
nvidia-smi                       # driver version, GPU model, VRAM
nproc                            # cores, which is what caps concurrent swarms
free -g                          # RAM
df -h /home /                    # free space, see the budget below
lsb_release -a                   # must be 24.04 or a noble derivative
ip -brief addr                   # the campus address, for the VPN fallback
```

Disk budget, per user: roughly 1.7 GB for the PX4 clone, 2 GB for its build,
1.5 GB for `px4_msgs`, and 1 GB for this repo's build. Call it 7 GB each, so
about 35 GB for five people, on top of the system. If `/home` is tight, a
shared reference clone saves most of the clone cost:

```bash
sudo git clone --bare https://github.com/PX4/PX4-Autopilot.git /opt/px4-reference.git
# then each user clones with:  git clone --reference /opt/px4-reference.git ...
```

Build directories have to stay per user either way, for the reason in the
Decisions table.

## 1.1 Keep it on, and keep it on Linux

The machine is useless to a remote user if it is sitting in Windows or asleep.

**Default to Ubuntu.** Find the Ubuntu entry and make it the default:

```bash
sudo grep -E '^menuentry|^submenu' /boot/grub/grub.cfg | cut -d"'" -f2
sudoedit /etc/default/grub
#   GRUB_DEFAULT="Ubuntu"          (or the exact menuentry title)
#   GRUB_TIMEOUT=5
sudo update-grub
```

To boot Windows once without changing the default, from a Linux session:

```bash
sudo grub-reboot "Windows Boot Manager"   # exact title from the list above
sudo reboot
```

Say out loud to the team: **do not reboot into Windows unless you are
physically in the room.** Nobody can get it back remotely.

**Stop it sleeping.**

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

The login screen suspends on its own too, independently of the above, if the
machine sits at GDM with nobody logged in:

```bash
sudo -u gdm dbus-run-session -- gsettings set \
  org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
```

**Fix the clock.** Windows keeps the hardware clock in local time and Ubuntu
keeps it in UTC, so a dual-boot machine's clock jumps by the UTC offset every
time you switch. Log timestamps and anything time-sensitive in DDS then
disagree. Set UTC on both sides:

```bash
timedatectl set-local-rtc 0
sudo timedatectl set-ntp true
```

and on the Windows side set
`HKLM\SYSTEM\CurrentControlSet\Control\TimeZoneInformation\RealTimeIsUniversal`
to DWORD 1. Also turn off Windows Fast Startup, which hibernates rather than
shutting down and leaves shared partitions dirty.

**Recovery.** Enable Wake-on-LAN so a power cut is not a trip to the lab:

```bash
sudo apt install ethtool
ip -brief link                                   # find the wired interface
sudo ethtool -s <iface> wol g                    # and make it persist in netplan
```

## 1.2 SSH

```bash
sudo ./scripts/lab/install-lab.sh --harden-ssh
```

Separate and opt-in, because getting this wrong locks you out of a machine
that is not in front of you. The script refuses to run at all until some
account on the machine has a key in its `authorized_keys`, validates with
`sshd -t` before restarting, and removes its own file if sshd rejects it.

What it writes to `/etc/ssh/sshd_config.d/10-flie.conf`:

```
PermitRootLogin no
PasswordAuthentication no
KbdInteractiveAuthentication no
AllowGroups flie-ssh
ClientAliveInterval 60
ClientAliveCountMax 10
```

**Keep your current session open until you have proved a new one works.**

Two things about this on 24.04. Ubuntu socket-activates sshd, so `Port` and
`ListenAddress` in `sshd_config` are ignored and live in the `ssh.socket` unit
instead. Authentication still reads from `sshd_config`, which is all of the
above.

`ClientAlive*` exists because a long headless swarm run with an idle terminal
is the connection shape a NAT or a campus firewall times out, and losing the
SSH session kills whatever is attached to it. Part 3 covers `tmux` for the same
reason.

## 1.3 Network reach

### Tailscale, the primary path

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --hostname=flie-lab
```

That script adds Tailscale's apt repository and installs from it. The
repository instructions at <https://tailscale.com/download/linux> do the same
thing by hand.

Do not pass `--advertise-routes` or `--advertise-exit-node`, for the reason in
Decisions.

Leave Tailscale SSH off (`--ssh` not set). Regular OpenSSH with the keys from
1.4 keeps one authentication story that also works over the VPN fallback and
from the console.

Then:

```bash
sudo tailscale set --auto-update
tailscale status            # note the 100.x address and the MagicDNS name
```

In the Tailscale admin console, disable key expiry for this node. Otherwise it
drops off the tailnet every 180 days and somebody has to walk to Evans to
re-authenticate it.

Each team member installs Tailscale on their laptop and is invited to the
tailnet. After that `ssh your_username@flie-lab` works from anywhere.

### UD VPN, the fallback

Nothing to install on the lab machine. Confirm the campus address from 1.0 is
static, or get a DNS name for it from IT, and have the team connect to the UD
VPN before SSHing. Everything else in this document is identical.

Either way: **do not port-forward 22 to the public internet.**

## 1.4 Accounts

One command per person. It creates the account with no password, puts them in
the groups that matter, prepares `~/.ssh`, writes their slot into their
`~/.bashrc`, and refuses a slot somebody else already has:

```bash
sudo ./scripts/lab/install-lab.sh --add-user your_username --slot 1
```

It then prints the remaining steps, which are the ones a script should not do
for you. The important one: **each person generates their own key on their own
laptop and sends you the public half.** Never the private half, and never a key
you generated for them.

```bash
# on their laptop
ssh-keygen -t ed25519 -C "your_username@udel.edu"
cat ~/.ssh/id_ed25519.pub          # this is what they send you
```

```bash
# on the lab machine, paste it, then Ctrl-D
sudo -u your_username tee -a /home/your_username/.ssh/authorized_keys
```

What the script sets up, and why:

| | |
|---|---|
| `--disabled-password` | There is no password to guess and none to share. Keys only |
| groups `render`, `video` | What lets them reach the GPU with no X session of their own, which is how VirtualGL's EGL back end works |
| group `flie-ssh` | `AllowGroups` in 1.2 means membership here is what grants SSH at all, so removing somebody is one `gpasswd -d` |
| group `flie` | Shared files under `/srv/flie/shared`, setgid so new files inherit the group |
| no `sudo` | Deliberate. Nothing in the normal workflow needs root, because 1.5 installed the system-wide half already |

## 1.5 The stack, system-wide

Run the normal installer as yourself, once. Everything it puts in `/opt`,
`/usr` and `/usr/local` is then shared, and every later user skips it.

```bash
git clone https://github.com/VIP-FLIE/DroneSwarming.git
cd DroneSwarming
./scripts/native/install.sh
./scripts/native/smoke-test.sh          # want: Stack is good.
```

## 1.6 Rendering and the display stack

```bash
sudo ./scripts/lab/install-lab.sh
```

That is 1.6 and 1.7 in one command, and it is safe to re-run. It:

- enables `nvidia-persistenced`, so the card stays initialised when nobody has
  it open, which on a machine with nobody at the console is most of the time
- installs VirtualGL at the version `.devcontainer/Dockerfile` pins, so the lab
  machine renders through the same code the container does
- installs TurboVNC, noVNC, websockify, openbox and the X utilities
- wraps **both** `gz` entry points so they run under `vglrun -d egl`
- creates the `flie` and `flie-ssh` groups and `/srv/flie/shared`
- finishes by running `--check` and telling you what is left

Two mechanisms to understand before running it:

**EGL.** Nobody is logged in at the console, so there is no X server on `:0`
and no desktop session owning the card. VirtualGL's EGL back end renders
through an EGL device directly, with no 3D X server involved. No dummy plug, no
`xorg.conf`. The dev container uses the same mechanism.

If `vglrun` cannot reach the card, the usual cause is group membership, and
group changes need a fresh login to take effect. `/dev/dri/renderD128` is
`root:render` mode 0660 on Ubuntu, so the `render` group is what grants it. The
documented alternative is `sudo /opt/VirtualGL/bin/vglserver_config`, option 2,
"Configure server for use with VirtualGL (EGL back end only)". Prove which one
is needed rather than assuming:

```bash
sudo -u your_username vglrun -d egl glxinfo | grep 'OpenGL renderer'
```

You want the card named. `llvmpipe` means software rendering.

**Both `gz` wrappers.** PATH ordering alone is not enough. ROS 2 ships its own
gz-tools at `/opt/ros/jazzy/opt/gz_tools_vendor/bin`, and
`source /opt/ros/jazzy/setup.bash` prepends that directory. Every script that
sources ROS therefore puts ROS's `gz` in front of `/usr/local/bin` and silently
drops VirtualGL. The GPU is lost with nothing reporting an error.

| Binary | Wrapped because |
|---|---|
| `/usr/bin/gz` | The apt one, reached when ROS is not sourced |
| `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` | ROS prepends it on every `source`, jumping ahead of `/usr/local/bin` |

## 1.7 Clients

Nothing to install on the lab machine for this. Two ways in, both showing the
same desktop:

| | Install on the laptop | Use it for |
|---|---|---|
| TurboVNC Viewer | A `.dmg`, `.exe` or `.deb` from <https://turbovnc.org> | Flying. Faster on 3D |
| Browser, noVNC | Nothing at all | A quick look, or a machine you cannot install on |

## 1.8 Check before handing it out

```bash
./scripts/lab/install-lab.sh --check     # reads only, needs no root
```

It reports the machine, the GPU, `nvidia-persistenced`, the render device
permissions, VirtualGL, both `gz` wrappers, TurboVNC, noVNC, the groups, who
may SSH in, and whether the stack itself is installed. Exit code is the number
of things missing.

Then the two that need a person:

```bash
./scripts/native/smoke-test.sh                    # Stack is good.
vglrun -d egl glxinfo | grep 'OpenGL renderer'    # names the card, not llvmpipe
```

---

# Part 2. Onboarding one person

Per person, about 30 minutes, mostly builds.

**1. Account and slot.** You run one command:

```bash
sudo ./scripts/lab/install-lab.sh --add-user your_username --slot 1
```

Then install the public key they sent you, per 1.4.

**2. They connect.** On their laptop, in `~/.ssh/config`:

```
Host flie-lab
    HostName flie-lab                 # the MagicDNS name, or the campus one
    User your_username
    LocalForward 5911 localhost:5911  # their VNC port, from the slot table
    LocalForward 6081 localhost:6081  # their noVNC port
    ServerAliveInterval 60
```

```bash
ssh flie-lab
```

**3. They install their half of the stack.**

```bash
git clone https://github.com/VIP-FLIE/DroneSwarming.git
cd DroneSwarming
./scripts/native/install.sh --skip-apt
echo "source $PWD/scripts/native/env.sh" >> ~/.bashrc
```

`--skip-apt` is the whole point of 1.5. It builds PX4, `px4_msgs` and this repo
into their home directory and touches nothing system-wide, so it needs no sudo.

`--add-user` already put `export DRONE_SLOT=1` in their `~/.bashrc`, above
where that `source` line lands. Order matters: `env.sh` reads the slot.

**4. Verify.**

```bash
exec bash
./scripts/native/smoke-test.sh
```

Section 5 is the one to read. It prints their slot, partition, instance range
and ports, and fails if the derived values disagree with the slot or if other
people are logged in and they have no slot at all.

**5. Their desktop.**

```bash
/opt/TurboVNC/bin/vncpasswd     # once. Not their account password
./scripts/lab/lab-desktop
```

`lab-desktop` prints the exact `ssh -L` line and both URLs. Then, on the
laptop, connect the TurboVNC Viewer to `localhost:5911` or open
<http://localhost:6081>. Both go through the SSH tunnel from step 2, so nothing
is exposed on the network.

The VNC password is not optional. `-localhost` limits the port to this machine,
and on a machine five people are logged into that is not much of a limit.

---

# Part 3. A normal day

```bash
ssh flie-lab
tmux new -s fly              # or: tmux attach -t fly
./scripts/lab/lab-status     # who else is on, and what they are running
./scripts/lab/lab-desktop    # desktop up, prints your ports
export DISPLAY=:11           # your slot's display. env.sh does not set this
```

`tmux` is not optional for anything long. An SSH session that drops takes every
process attached to it, and a 20 minute swarm run is a long time to hold a
connection over campus Wi-Fi. Detach with `Ctrl-b d` and everything keeps
flying.

Four panes, as [NATIVE-SETUP.md](NATIVE-SETUP.md) describes, with the one
difference that the agent port is yours:

```bash
MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT                    # 1. PX4 to ROS 2 bridge
drone-gcs                                                     # 2. ground control station link
cd ~/PX4-Autopilot                                            # 3. simulator
PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i $((PX4_INSTANCE_BASE + 1))   # gives you a pxh> prompt
ros2 topic list | grep fmu                                    # 4. your side
```

`drone-gcs` reads the rest of it from the environment, so there is nothing else
to remember. Swarms are unchanged:

```bash
ros2 launch swarm_naive swarm.launch.py
ros2 launch swarm_comms swarm.launch.py
HEADLESS=1 ros2 launch swarm_naive swarm.launch.py count:=6
```

Your topics are under `/px4_<your first instance>/fmu/`, which `lab-status` and
the launch file's own startup line both print.

**Write code over Remote-SSH, not over VNC.** VS Code's Remote-SSH extension
puts the editor on your laptop and the files on the lab machine, with no
graphics involved. The VNC desktop is for looking at Gazebo.

## Etiquette

The machine has one GPU and a fixed number of cores, and PX4 runs in lockstep,
so an oversubscribed machine does not drop frames, it drops real-time factor.
Two people each flying eight drones will both see their swarm go slow and both
assume it is their own code.

| | |
|---|---|
| Run headless unless you are looking | `HEADLESS=1`. Saves the GPU and the CPU for everyone else, and swarm work needs it anyway |
| Check before you launch | `lab-status` names who is flying and what the GPU is doing |
| Check RTF before you trust a run | Real-time factor is a shared resource. A run at 0.6 does not show what your controller does |
| Say so for anything long | If you are taking the machine for an hour, put it in GroupMe |
| Kill what you finished | `pkill -f 'gz sim'` reaches only your own processes. Leaving a server running means your next launch joins it |
| Do not reboot into Windows remotely | Nobody can get it back |

## What differs from the container

The stack is identical. ROS 2, Gazebo and PX4 are the pinned versions from
`.devcontainer/Dockerfile`, installed by the same `install.sh`, and
`smoke-test.sh` still checks them against that file. Flight behaviour does not
change. What differs is how the pixels reach you and how you are kept apart
from the other four people.

| | Container | Native, your laptop | Lab machine |
|---|---|---|---|
| Display server | Xvfb | Yours | TurboVNC `Xvnc` |
| Publisher | x11vnc plus noVNC | n/a | TurboVNC plus noVNC |
| Rendering | `vglrun -d egl`, NVIDIA only | Your GL driver | `vglrun -d egl`, the lab NVIDIA card |
| Reached over | Published port on localhost | n/a | SSH tunnel |
| Isolation | Docker's network | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` | Per-slot domain, partition and ports |
| `ROS_DOMAIN_ID` | 0 | 0 | Your slot |

TurboVNC in place of Xvfb plus x11vnc is the one deliberate divergence.
The Dockerfile's reason for Xvfb is that it exposes Mesa llvmpipe with OpenGL
4.5, which matters on the many team laptops with no usable GPU. The lab machine
has a GPU and VirtualGL handles GL either way, so that reason does not apply
here, and TurboVNC is faster on 3D content.

---

# Part 4. Where the slot scheme lives

All of it defaults to slot 0, which is the single-user setup exactly as it was.
`DRONE_SLOT` stays unset on a laptop.

| File | Change |
|---|---|
| [scripts/native/env.sh](../../scripts/native/env.sh) | Derives `ROS_DOMAIN_ID`, `GZ_PARTITION`, `PX4_INSTANCE_BASE`, `PX4_UXRCE_DDS_PORT` and `DRONE_GCS_PORT` from one `DRONE_SLOT`. Rejects a slot that is not 0-9 and falls back to 0, loudly |
| [.devcontainer/scripts/drone-gcs](../../.devcontainer/scripts/drone-gcs) | `--listen-port` and `--instance-base`, defaulting from the environment. Serves a block of ten instances starting at base+1, which also fixed the missing tenth drone |
| [src/swarm_naive/launch/swarm.launch.py](../../src/swarm_naive/launch/swarm.launch.py) | `instance_base` launch argument, `instance = instance_base + slot + 1`, and a validation that refuses a base that would overlap the next slot |
| [src/swarm_comms/launch/swarm.launch.py](../../src/swarm_comms/launch/swarm.launch.py) | The same |
| [scripts/native/smoke-test.sh](../../scripts/native/smoke-test.sh) | New section 5, the slot check. Also moved its scratch files out of `/tmp`, which is shared and sticky, and made its own SITL run use your instance rather than 0 |
| [scripts/test-drone-gcs.py](../../scripts/test-drone-gcs.py) | Tests: the block reaches instance 10, blocks do not overlap, the arguments validate, and the environment defaults work and survive a typo |
| `scripts/lab/install-lab.sh` | The administrator's tool. `--check`, `--add-user`, `--harden-ssh`, and the default system setup |
| `scripts/lab/lab-desktop` | Your TurboVNC desktop and noVNC, on your slot's display and ports |
| `scripts/lab/lab-status` | Who is on, what is flying, which slots hold ports, the GPU, and your own environment |

Nothing here changes what the container does. The container is always slot 0,
`DRONE_SLOT` is never set in it, and every default is what it was.

## Checking it yourself

```bash
./scripts/test-drone-gcs.py                       # checks the drone-gcs bookkeeping
./scripts/native/smoke-test.sh                    # section 5 is the slot check
DRONE_SLOT=3 bash -c 'source scripts/native/env.sh; echo $PX4_INSTANCE_BASE'
```

The last one should print 30. Then confirm the launch files followed:

```bash
ros2 launch swarm_naive swarm.launch.py --show-args | grep -A3 instance_base
```

---

# Problems

| Symptom | Cause | Fix |
|---|---|---|
| `Arming denied: Resolve system health failures first`, with `drone-gcs` running and healthy | Another user's PX4 holds your GCS port, so `mavlink start` could not bind | Check `DRONE_SLOT` is set and `PX4_INSTANCE_BASE` matches the table. `ss -ulnp \| grep 185` shows who has what |
| A drone you did not launch appears in your Gazebo | `GZ_PARTITION` unset, so `get_gz_world()` found somebody else's world | `echo $GZ_PARTITION`. Then `pkill -f 'gz sim'` and relaunch |
| `/fmu/` topics from a vehicle you did not start | Shared `ROS_DOMAIN_ID` | `echo $ROS_DOMAIN_ID` against the slot table |
| No `/fmu/` topics at all | The agent is on 8888 but your PX4 is looking at `8888 + slot`, or the other way round | `MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT` |
| The tenth drone of a ten-drone swarm never arms | Fixed. `drone-gcs` covered instances 0-9 while the launch files number from 1 | Update this repo. `./scripts/test-drone-gcs.py` pins it |
| `instance_base must be 0, 10, 20 ... 90` at launch | `PX4_INSTANCE_BASE` was set by hand to something that overlaps another slot | `unset PX4_INSTANCE_BASE` and re-source `env.sh` |
| Smoke test section 5 says the base does not match the slot | Something set `PX4_INSTANCE_BASE` after `env.sh` ran | `unset PX4_INSTANCE_BASE; source scripts/native/env.sh` |
| `lab-desktop` says you have no VNC password | You have not set one yet | `/opt/TurboVNC/bin/vncpasswd`. Not your account password |
| `MicroXRCEAgent` exits immediately | Somebody else has your port, or you started two | `ss -ulnp \| grep 888` |
| Gazebo at a few frames per second, `nvidia-smi` shows nothing running | Not going through VirtualGL. `source`ing ROS puts its vendored `gz` ahead of the wrapper | Check both wrappers from 1.6 exist. `vglrun -d egl glxinfo \| grep renderer` |
| `vglrun` reports `Could not open EGL device` | Not in `render` and `video`, or `nvidia-persistenced` is not running | `id`, then `systemctl status nvidia-persistenced`. Group changes need a fresh login |
| VNC viewer connects to a grey screen with no window manager | `openbox` did not start | `~/.vnc/xstartup.turbovnc`, then `lab-desktop restart` |
| SSH works, `localhost:5911` refuses | The tunnel is not up, or you are using the wrong slot's port | `LocalForward` lines in `~/.ssh/config` against the slot table |
| Everything dropped when my laptop slept | The SSH session went with it | `tmux`. Always, for anything long |
| Machine unreachable | Booted into Windows, asleep, or off | Somebody has to go to Evans 153. This is why 1.1 exists |
| Real-time factor well below 1.0 with a small swarm | Somebody else is running a big one | `lab-status` |
| Tailscale node vanished after months | Key expiry | Disable key expiry for this node in the admin console. Re-authentication needs console access |

---

# Status

The repo changes in Part 4 are written and tested. Nothing in Part 1 has been
run on the actual lab machine yet.

| | Status |
|---|---|
| The isolation problem | Verified. Read out of PX4 v1.16.1 (`px4-rc.mavlink`, `px4-rc.gzsim`, `rcS`, `platforms/posix/CMakeLists.txt`) and this repo. The Gazebo variable names confirmed present in the `libgz-transport13` that Harmonic loads |
| Part 4, the repo changes | Verified on one machine. The native smoke test passed, including a vehicle arming and flying; section 5 is the slot check; `./scripts/test-drone-gcs.py` runs the drone-gcs checks; both launch files' `instance_base` default tracks `DRONE_SLOT` |
| Part 4, two users at once | Untested. One machine, one user is what has actually been run |
| Tailscale limits | 6 users, unlimited user devices on the free Personal plan, confirmed September 2026 |
| VirtualGL EGL back end | The mechanism is what the container already uses. `vglserver_config` option 2 versus plain `render` group membership is untested, hence the check in 1.6 |
| TurboVNC | Untested. The TurboVNC plus VirtualGL pairing is the standard remote 3D stack, but no measurement of Gazebo framerate over it exists yet |
| Everything else in Part 1 | Untested |

Measure it once it is up, and put the numbers in
[SIMULATION-FIDELITY.md](../simulation/SIMULATION-FIDELITY.md) next to the
existing ones: Gazebo GUI framerate over TurboVNC against the same scene on the
console, and real-time factor with two people flying at once. The second number
decides how many people can share the machine. It is unmeasured.