# The stack without the scripts

`drone-gcs` supplies a MAVLink ground station link for SITL. This document runs the stack with nothing of this repo's, then shows what `drone-gcs` stands in for.

Use `drone-gcs` for everyday work.

## The four processes

Every setup is these four, in this order.

| Process | What it is | On a real X500 |
|---|---|---|
| `MicroXRCEAgent` | eProsima Micro XRCE-DDS Agent v2.4.3. Translates DDS ↔ uXRCE-DDS | Yes. Runs on companion computer |
| `px4` | PX4 flight stack v1.16.1, compiled for Linux | Yes. Same source, cross compiled for Pixhawk |
| `gz sim` | Gazebo Harmonic. Physics and sensors | No. Replaced by the world |
| MAVLink ground station | One heartbeat per second, satisfies datalink-loss failsafe | Yes. QGroundControl on a laptop over telemetry radio |

Gazebo is the only process not on the aircraft.

## Run it with nothing of ours

Four terminals. First three use nothing from this repo.

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

One environment variable and a binary is terminal 3. Terminal 2 is the whole of `drone-gcs` for a single vehicle (eight lines). Terminal 4 is unchanged on the aircraft.

## The build target

`make px4_sitl gz_x500` is PX4's CMake target. From `src/modules/simulation/gz_bridge/CMakeLists.txt` line 101 in v1.16.1:

```bash
cd ~/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 ../bin/px4
```

The `cd` is not needed. PX4 changes into its own rootfs on startup (`platforms/posix/src/px4/common/main.cpp` line 269). With `-i N` it uses `rootfs/N`.

`GZ_SIM_RESOURCE_PATH` needs no export. PX4 writes `build/px4_sitl_default/rootfs/gz_env.sh` at build time and sources it. When you start Gazebo yourself, source `gz_env.sh` yourself.

## How one environment variable becomes a drone

`PX4_SIM_MODEL=gz_x500` is the only input. Inside PX4:

1. `rcS` line 56 finds `4001_gz_x500` in `etc/init.d-posix/airframes/`. `PX4_SYS_AUTOSTART` is optional.
2. `4001_gz_x500` sets airframe parameters, including `param set-default NAV_DLL_ACT 2`. Defaults `PX4_GZ_WORLD` to `default`.
3. `px4-rc.gzsim` looks for a running Gazebo world. Finding none, starts one: `gz sim -r -s $PX4_GZ_WORLDS/default.sdf`, plus `gz sim -g` for GUI unless `HEADLESS` is set.
4. Same script spawns the vehicle: `gz service -s /world/default/create`, model `x500_<instance>`.
5. `gz_bridge` connects PX4 actuator outputs and Gazebo sensor topics.
6. `uxrce_dds_client` connects to the agent on udp 8888, creates `/fmu/` topics.

Steps 3-5 are the simulator. Steps 1, 2 and 6 happen on the aircraft.

## What `drone-gcs` stands in for

PX4's x500 airframe sets `NAV_DLL_ACT 2`. PX4 refuses to arm with a broken failsafe precondition. Any of these satisfy it:

| Option | Use |
|---|---|
| Eight-line heartbeat above | Understanding the requirement |
| QGroundControl, udp 14550 | Map, HUD, parameters, logs. Real team use |
| `drone-gcs` | Everyday work, any swarm |
| `drone-gcs --expect 3` | Swarm, count verification |

`drone-gcs` does two things the eight-line version does not:

- Listens on udp 14550 (QGroundControl port).
- Heartbeats every SITL instance's port (`18570 + instance`, instances 0-10). Each instance runs its own GCS link on its own port (`px4-rc.mavlink`, `udp_gcs_port_local`). A station on one port serves one vehicle.

### What PX4 checks

`gcs_connection_lost` is one bool on `/fmu/out/vehicle_status_v1`. It latches:

| Event | Effect | Source, v1.16.1 |
|---|---|---|
| Boot | Starts true | `Commander.cpp` line 695 |
| MAVLink heartbeat with type `MAV_TYPE_GCS` arrives | Goes false | `Commander.cpp` line 2752 |
| `COM_DL_LOSS_T` seconds of silence (default 10) | Goes true, only if a heartbeat had arrived | `Commander.cpp` lines 2816-2820 |
| While true and `NAV_DLL_ACT > 0` | Arming refused for every mode | `rcAndDataLinkCheck.cpp` |

Killing the ground station mid-flight takes ~10 s to trigger the failsafe. A vehicle that never saw a station reports `gcs_connection_lost: true` with `gcs_connection_lost_counter: 0` (flag starts true, not latched from a lost link).

Measured with no ground station:

```
gcs_connection_lost: true
gcs_connection_lost_counter: 0
pre_flight_checks_pass: false
```

```
WARN  [commander] Arming denied: Resolve system health failures first
```

Do not fix with `param set NAV_DLL_ACT 0`. See [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md).

## Separating simulator from flight stack

Start Gazebo and PX4 in separate terminals:

```bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
gz sim -r -s "$PX4_GZ_WORLDS/default.sdf"
```

```bash
gz model --list          # ground_plane. No aircraft
```

```bash
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

PX4 prints `Standalone PX4 launch, waiting for Gazebo`, finds the running world, spawns into it. `gz model --list` shows `x500_0`.

`-s` is server (physics, no window). `gz sim -g` attaches a GUI. PX4's startup script runs that pair. `HEADLESS=1` suppresses the window.

**Source `gz_env.sh` first.** Without it, `GZ_SIM_SYSTEM_PLUGIN_PATH` does not include `build/px4_sitl_default/src/modules/simulation/gz_plugins`. Gazebo loads a world with none of PX4's sensor or motor plugins. The drone spawns but has no sensors:

```
WARN  [health_and_arming_checks] Preflight Fail: Accel Sensor 0 missing
WARN  [health_and_arming_checks] Preflight Fail: Gyro Sensor 0 missing
WARN  [health_and_arming_checks] Preflight Fail: barometer 0 missing
WARN  [health_and_arming_checks] Preflight Fail: Found 0 compass (required: 1)
WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data
```

A vehicle that boots, connects, publishes `/fmu/` topics and refuses to arm with missing hardware is PX4 behaving as it would on an aircraft with an unplugged sensor.

## Simulator vs. aircraft

| Piece | Here | On the X500 |
|---|---|---|
| Flight stack | `build/px4_sitl_default/bin/px4`, Linux process | Same PX4 source, cross compiled for Pixhawk |
| IMU, GPS, barometer, magnetometer | Gazebo computes from model's true state | Real sensors on flight controller and GPS mast |
| Motor commands | PX4 actuator outputs into Gazebo rotor plugins | PWM/DShot out of Pixhawk into ESCs |
| Physics | Gazebo, lockstep with PX4 | The air |
| PX4 to ROS 2 link | uXRCE-DDS over udp 8888, localhost | uXRCE-DDS over serial to Pixhawk TELEM port, or ethernet |
| `MicroXRCEAgent` | Your laptop | Companion computer |
| Your ROS 2 node | Your laptop | Companion computer. Same binary |
| Ground station | `drone-gcs` on localhost | QGroundControl over 915 MHz telemetry radio |
| `/fmu/in/` and `/fmu/out/` topic names | Identical | Identical |
| Flight modes, arming checks, failsafes | Identical | Identical |

Your node, its topics and PX4's rules do not change. What changes is where sensor numbers come from and where motor commands go.

## Check it yourself

```bash
ros2 topic echo --once /fmu/out/vehicle_status_v1 | grep -E "gcs_connection_lost|pre_flight"
gz model --list                      # what is in the world
gz topic -l | head                   # Gazebo's topics
ros2 topic list | grep fmu | head    # what the agent created
ros2 topic hz /fmu/out/vehicle_local_position
```

At `pxh>`:

```
pxh> commander status
pxh> listener vehicle_status
pxh> param show NAV_DLL_ACT
pxh> mavlink status
pxh> uxrce_dds_client status
```

`-d` suppresses the prompt. Swarm launch files use it (no terminal under `ros2 launch`).

## Reference

- [HOW-IT-WORKS.md](HOW-IT-WORKS.md)
- [../swarm/SWARM.md](../swarm/SWARM.md)
- [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md)
- [../setup/NATIVE-SETUP.md](../setup/NATIVE-SETUP.md)
- [PX4 Gazebo simulation](https://docs.px4.io/v1.16/en/sim_gazebo_gz/)
- [uXRCE-DDS bridge](https://docs.px4.io/v1.16/en/middleware/uxrce_dds)
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- [MAVLink HEARTBEAT](https://mavlink.io/en/messages/common.html#HEARTBEAT)
- [PX4 safety configuration](https://docs.px4.io/v1.16/en/config/safety.html)