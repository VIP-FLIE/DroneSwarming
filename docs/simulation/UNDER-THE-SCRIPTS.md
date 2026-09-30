# The stack without the scripts

`drone-gcs` is a script in this repo that supplies a MAVLink ground station
link for SITL. It hides which parts of the stack are real flight software and
which are simulation scaffolding.

This document runs the same stack with nothing of this repo's in the way, then
takes `drone-gcs` apart and shows what it is standing in for.

Nothing here replaces `drone-gcs`. Use it for everyday work.

## The four processes

Every setup in this repo is these four, in this order.

| Process | What it actually is | On a real X500 |
|---|---|---|
| `MicroXRCEAgent` | eProsima Micro XRCE-DDS Agent v2.4.3. Translates between DDS, which ROS 2 speaks, and the compact variant PX4 speaks | Yes. Runs on the companion computer |
| `px4` | The PX4 flight stack, pinned v1.16.1, compiled for Linux instead of a flight controller board | Yes. The same source, cross compiled for the Pixhawk and flashed to it |
| `gz sim` | Gazebo Harmonic. Physics, and the sensor readings PX4 would otherwise get from hardware | No. Replaced by the world |
| A MAVLink ground station | One heartbeat a second, so PX4's datalink-loss failsafe is satisfied | Yes. QGroundControl on a laptop over a telemetry radio |

Gazebo is the only one of the four that does not exist on the aircraft. The
agent and PX4 fly on it, and the ground station becomes a laptop in a field.

## Run it with nothing of ours

Four terminals. The first three use nothing from this repo: they are PX4's
binary, eProsima's agent and a MAVLink heartbeat. The fourth is your own node.

```bash
MicroXRCEAgent udp4 -p 8888
```

```bash
python3 -c "
import time
from pymavlink import mavutil
link = mavutil.mavlink_connection('udpout:127.0.0.1:18570', source_system=255)
while True:
    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                            mavutil.mavlink.MAV_STATE_ACTIVE)
    time.sleep(1)
"
```

```bash
cd ~/PX4-Autopilot
PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

```bash
ros2 run cpp_single_hop hop
```

One environment variable and a binary is the whole of terminal 3. It brings up
Gazebo, loads a world, spawns an x500 in it, and gives you a `pxh>` prompt.

Terminal 2 is the whole of `drone-gcs` for a single vehicle. Eight lines.

Terminal 4 is unchanged whichever way you started the other three, and it is
unchanged on the aircraft.

## The build target and its environment

`make px4_sitl gz_x500` is PX4's own CMake target. It builds SITL if it is out
of date, then runs it. From
`src/modules/simulation/gz_bridge/CMakeLists.txt` line 101 in v1.16.1, it
expands to:

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 ../bin/px4
```

The `cd` is not needed. PX4 changes into its own rootfs on startup
(`platforms/posix/src/px4/common/main.cpp` line 269), and with `-i N` it uses
`rootfs/N`, so several instances do not share one log directory.

`GZ_SIM_RESOURCE_PATH` needs no export from you. PX4 writes
`build/px4_sitl_default/rootfs/gz_env.sh` at build time and its own startup
script sources it. Verified by unsetting the variable before launching: Gazebo
still found its models and worlds. That holds only while PX4 is the one
starting Gazebo. When you start Gazebo yourself, source `gz_env.sh` yourself.
See "Seeing where the simulator ends" below.

## How one environment variable becomes a drone

`PX4_SIM_MODEL=gz_x500` is the only input. The chain, all inside PX4:

1. `rcS` line 56 looks in `etc/init.d-posix/airframes/` for a file whose name
   ends in `_gz_x500`, and finds `4001_gz_x500`. `PX4_SYS_AUTOSTART` is optional
   here. Set it only when you want an airframe the name does not pick.
2. `4001_gz_x500` sets the airframe parameters, including
   `param set-default NAV_DLL_ACT 2`, which is the reason terminal 2 exists. It
   also defaults `PX4_GZ_WORLD` to `default`.
3. `px4-rc.gzsim` looks for a running Gazebo world. Finding none, it starts one:
   `gz sim -r -s $PX4_GZ_WORLDS/default.sdf`, plus `gz sim -g` for the GUI
   unless `HEADLESS` is set.
4. The same script spawns the vehicle into that world with a Gazebo service
   call, `gz service -s /world/default/create`, naming the model `x500_<instance>`.
5. `gz_bridge` connects PX4's actuator outputs and Gazebo's sensor topics.
6. `uxrce_dds_client` connects to the agent on udp 8888 and creates the `/fmu/`
   topics.

Steps 3, 4 and 5 are the simulator. Steps 1, 2 and 6 happen on the real
aircraft too.

## What `drone-gcs` is standing in for

PX4's x500 airframe sets `NAV_DLL_ACT 2`, return on datalink loss. PX4 refuses
to arm with a failsafe whose precondition is already broken, so it needs to see
a ground station before it will leave the ground. Any of these satisfy it:

| Option | When to use it |
|---|---|
| The eight-line heartbeat above | Understanding what the requirement actually is |
| QGroundControl, listening on udp 14550 | You want a map, a HUD, parameters and logs. This is what a real team uses |
| `drone-gcs` | Everyday work, and any swarm |
| `drone-gcs --expect 3` | A swarm, when you want it to check the count rather than you reading it |

`drone-gcs` does two things the eight-line version does not:

- Listens on udp 14550 as well, the port QGroundControl uses, so a vehicle that
  reaches out first is answered.
- Heartbeats every SITL instance's own port, `18570 + instance`, for instances 0
  to 10. Each instance runs its own GCS link on its own port
  (`px4-rc.mavlink`, `udp_gcs_port_local`), with no broadcast, so a station
  sitting on one port serves one vehicle. A swarm started against a
  single-vehicle station has drone 0 fly away while the rest report
  `Arming denied`.

### What PX4 is actually checking

`gcs_connection_lost` is one bool on `/fmu/out/vehicle_status_v1`, and it
latches:

| Event | Effect | Source, v1.16.1 |
|---|---|---|
| Boot | Starts **true**. PX4 assumes no link until shown one | `Commander.cpp` line 695 |
| Any MAVLink heartbeat with type `MAV_TYPE_GCS` arrives | Goes false | `Commander.cpp` line 2752 |
| `COM_DL_LOSS_T` seconds of silence, default 10 | Goes true again, but only if a heartbeat had ever arrived | `Commander.cpp` lines 2816-2820 |
| While true, and `NAV_DLL_ACT > 0` | Arming is refused for every mode | `rcAndDataLinkCheck.cpp` |

Two consequences. Killing the ground station mid-flight takes about ten seconds
to trigger the failsafe. A vehicle that has never seen a station reports
`gcs_connection_lost: true` with `gcs_connection_lost_counter: 0`, because the
flag starts true rather than latching from a lost link.

Measured with no ground station running at all:

```
gcs_connection_lost: true
gcs_connection_lost_counter: 0
pre_flight_checks_pass: false
```

```
WARN  [commander] Arming denied: Resolve system health failures first
```

Do not fix that with `param set NAV_DLL_ACT 0`. It deletes the failsafe instead
of supplying the link, and the simulated vehicle stops behaving like the real
one when telemetry drops. See
[SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md).

## Seeing where the simulator ends

Start Gazebo and PX4 in separate terminals to separate the simulator from the
flight stack.

```bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
gz sim -r -s "$PX4_GZ_WORLDS/default.sdf"
```

```bash
gz model --list          # ground_plane. A world with no aircraft in it
```

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

PX4 prints `Standalone PX4 launch, waiting for Gazebo`, finds the running world,
spawns itself into it, and `gz model --list` then shows `x500_0`.

`-s` is the server: physics, no window. `gz sim -g` in a third terminal attaches
a GUI to it. PX4's own startup script runs exactly that pair, which is why
`HEADLESS=1` suppresses a window without affecting the flight.

**Source `gz_env.sh` first.** Skipping it fails with the missing-sensor errors
below. PX4 generates that file at build time and sources it itself on the normal
path, which is why nothing else in this repo mentions it. When you start Gazebo
yourself, you are the one who has to source it.

Without it, `GZ_SIM_SYSTEM_PLUGIN_PATH` does not include
`build/px4_sitl_default/src/modules/simulation/gz_plugins`, so Gazebo loads a
world with none of PX4's sensor or motor plugins in it. The drone still spawns.
It still appears in `gz model --list`. It has no sensors:

```
WARN  [health_and_arming_checks] Preflight Fail: Accel Sensor 0 missing
WARN  [health_and_arming_checks] Preflight Fail: Gyro Sensor 0 missing
WARN  [health_and_arming_checks] Preflight Fail: barometer 0 missing
WARN  [health_and_arming_checks] Preflight Fail: Found 0 compass (required: 1)
WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data
```

A vehicle that boots, connects, publishes `/fmu/` topics and then refuses to arm
with a list of missing hardware is PX4 behaving exactly as it would on a real
aircraft with an unplugged sensor. The simulated sensors are Gazebo plugins, and
a Gazebo that cannot find them reports the aircraft without them.

## What is the simulator and what is the aircraft

| Piece | Here | On the X500 |
|---|---|---|
| Flight stack | `build/px4_sitl_default/bin/px4`, a Linux process | The same PX4 source cross compiled for the Pixhawk's microcontroller |
| IMU, GPS, barometer, magnetometer | Gazebo computes them from the model's true state | Real sensors on the flight controller and the GPS mast |
| Motor commands | PX4 actuator outputs into Gazebo's rotor plugins | PWM or DShot out of the Pixhawk into four ESCs |
| Physics | Gazebo, in lockstep with PX4 | The air |
| PX4 to ROS 2 link | uXRCE-DDS over udp 8888, localhost | uXRCE-DDS over a serial link to the Pixhawk's TELEM port, or ethernet |
| `MicroXRCEAgent` | Your laptop | The companion computer bolted to the airframe |
| Your ROS 2 node | Your laptop | The companion computer. Same binary |
| Ground station | `drone-gcs` on localhost | QGroundControl over a 915 MHz telemetry radio |
| `/fmu/in/` and `/fmu/out/` topic names | Identical | Identical |
| Flight modes, arming checks, failsafes | Identical | Identical |

The last three rows are identical between the two columns. Your node, the topics
it uses and the rules PX4 enforces do not change. What changes underneath is
where the sensor numbers come from and where the motor commands go.

## Check any of this yourself

```bash
ros2 topic echo --once /fmu/out/vehicle_status_v1 | grep -E "gcs_connection_lost|pre_flight"
gz model --list                      # what is actually in the world
gz topic -l | head                   # Gazebo's own topics, nothing to do with ROS
ros2 topic list | grep fmu | head    # what the agent created
ros2 topic hz /fmu/out/vehicle_local_position
```

At a `pxh>` prompt, PX4 will tell you about itself:

```
pxh> commander status
pxh> listener vehicle_status
pxh> param show NAV_DLL_ACT
pxh> mavlink status
pxh> uxrce_dds_client status
```

`-d` suppresses that prompt, which is why the swarm launch files use it: there
is no terminal to attach it to under `ros2 launch`.

## Reference

- [QUICKSTART.md](QUICKSTART.md), the everyday commands
- [HOW-IT-WORKS.md](HOW-IT-WORKS.md), the pipeline and the three rules PX4 imposes
- [../swarm/SWARM.md](../swarm/SWARM.md), several vehicles, where the launch files already run the raw binary
- [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md), what the simulator gets right
- [../setup/NATIVE-SETUP.md](../setup/NATIVE-SETUP.md), installing the four processes above
- [PX4 Gazebo simulation](https://docs.px4.io/v1.16/en/sim_gazebo_gz/), every `PX4_GZ_*` variable
- [uXRCE-DDS bridge](https://docs.px4.io/v1.16/en/middleware/uxrce_dds), including the serial setup used on hardware
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent), the agent itself
- [MAVLink HEARTBEAT](https://mavlink.io/en/messages/common.html#HEARTBEAT), the message terminal 2 sends
- [PX4 safety configuration](https://docs.px4.io/v1.16/en/config/safety.html), where `NAV_DLL_ACT` and `COM_DL_LOSS_T` live