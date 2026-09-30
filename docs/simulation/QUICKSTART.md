# Running the simulator and writing your first node

Install instructions are in [../setup/NATIVE-SETUP.md](../setup/NATIVE-SETUP.md)
for Linux, or [../setup/SETUP.md](../setup/SETUP.md) for macOS and Windows. Every
terminal needs `scripts/native/env.sh` sourced, which the install puts in
`~/.bashrc`.

## Start the stack

Four terminals:

```bash
MicroXRCEAgent udp4 -p 8888     # 1. PX4 to ROS 2 bridge
drone-gcs                       # 2. ground control station link
cd ~/PX4-Autopilot              # 3. simulator, gives you a pxh> prompt
PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
ros2 topic list | grep fmu      # 4. your side
```

Terminal 4 printing nothing means the agent is not connected. Gazebo opens a
window on your desktop.

`drone-gcs` is required. PX4's x500 airframe sets `NAV_DLL_ACT 2`, return on
datalink loss, so it will not arm without a GCS, same as a real aircraft.
Skipping it gives `Arming denied: Resolve system health failures first`. Do not
fix that with `param set NAV_DLL_ACT 0`: that deletes the failsafe instead of
supplying the link, and the vehicle stops matching the real one when telemetry
drops. Run QGroundControl against udp 14550 for a real GCS. Killing `drone-gcs`
mid-flight tests the failsafe.

The terminal 3 command is PX4's own SITL binary. PX4 also ships a build target,
`make px4_sitl gz_x500`, which builds SITL if it is out of date and then runs
it. Bare `gz sim` opens Gazebo's world chooser and starts no drone. For swarms
and long runs prefix the launch with `HEADLESS=1`, which skips rendering
entirely.

`drone-gcs` is a script in this repo. [UNDER-THE-SCRIPTS.md](UNDER-THE-SCRIPTS.md)
runs the same stack with no scripts of ours in it and maps each process onto the
aircraft. The short version is one environment variable and a binary:

```bash
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

## Fly it

At the `pxh>` prompt:

```
pxh> commander takeoff
pxh> commander land
pxh> commander status                    # arming state, mode, failsafes
pxh> commander mode posctl
pxh> listener vehicle_local_position     # PX4's own view of a uORB message
```

PX4's world is untextured and the camera starts far out, so a blank grey 3D
view is normal. Scroll to zoom. To fly somewhere with scenery in it, or to build
a map of your own, see [WORLDS.md](WORLDS.md):

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=lawn PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4
```

## Build and run your code

Write code in this repo. `colcon` has to run from the repo root, the directory
that contains `src/`:

```bash
colcon build --symlink-install
source install/setup.bash
```

With `--symlink-install`, editing a Python node takes effect on the next
`ros2 run`. Rebuild after adding files or changing `setup.py`.

The repo is wherever you cloned it. New terminals source the overlay once
`scripts/native/env.sh` is in your `~/.bashrc`. `~/PX4-Autopilot` and
`~/ros2_ws` are yours and persist.

Starting a package of your own rather than editing these two:
[../development/NEW-PACKAGE.md](../development/NEW-PACKAGE.md).

`src/py_single_square` has two Python nodes:

```bash
ros2 run py_single_square listener      # position, altitude, heading, mode, 1 Hz
ros2 run py_single_square fly           # arms, goes offboard, flies a square, lands
ros2 run py_single_square fly --ros-args -p altitude:=8.0 -p side:=10.0
```

`src/cpp_single_hop` is the C++ package, with one node:

```bash
ros2 run cpp_single_hop hop         # takes off, flies 3 m forward, lands
ros2 run cpp_single_hop hop --ros-args -p altitude_m:=6.0 -p forward_m:=10.0
```

`hop` is the starting point for flight code.
[HOW-IT-WORKS.md](HOW-IT-WORKS.md) documents it and the pipeline behind it.

`src/swarm_naive` and `src/swarm_comms` are the two multi-drone examples. Each
starts its own PX4 instances, so they replace the raw PX4 command:

```bash
ros2 launch swarm_naive swarm.launch.py    # 3 drones, none aware of the others
ros2 launch swarm_comms swarm.launch.py    # the same flight, with them talking
ros2 launch swarm_naive swarm.launch.py --show-args
```

[../swarm/SWARM.md](../swarm/SWARM.md) covers both, and everything below about
running several vehicles by hand.

To start a package of your own rather than editing these two, follow
[../development/NEW-PACKAGE.md](../development/NEW-PACKAGE.md). It covers both
build types, Python and C++, from `ros2 pkg create` through to `ros2 run`.

## QoS

PX4 publishes `/fmu/out/` with BEST_EFFORT reliability and TRANSIENT_LOCAL
durability. ROS 2 subscribers default to RELIABLE and VOLATILE. These are
incompatible: the subscription is created without error, never matches the
publisher, and the callback never fires. Nothing is logged.

`ros2 topic echo` negotiates QoS on Jazzy:

```bash
ros2 topic echo /fmu/out/vehicle_local_position     # works, no flags needed
```

Measured here: default echo gets data on the topic, a plain rclpy subscription
with default QoS on the same topic gets zero callbacks.

Use the shared profile on anything touching `/fmu/`:

```python
from py_single_square.px4_qos import PX4_QOS

self.create_subscription(
    VehicleLocalPosition, '/fmu/out/vehicle_local_position',
    self.callback, PX4_QOS)
```

## Offboard mode

PX4 will not enter offboard until it is already receiving setpoints, and drops
out if the stream stops for more than about half a second. Publish
`OffboardControlMode` every timer tick, before and after the mode switch.
`takeoff_and_fly.py` does 10 Hz and sends 10 setpoints before switching.

A drone that refuses to arm or falls back to another mode has a missing or
stalled setpoint stream.

## More than one drone

Two launch files do all of this for you, and
[../swarm/SWARM.md](../swarm/SWARM.md) is the guide to them:

```bash
MicroXRCEAgent udp4 -p 8888                       # 1. same as always
drone-gcs                                         # 2. same as always
ros2 launch swarm_naive swarm.launch.py           # 3. replaces the raw PX4 command
```

The rest of this section is what those launch files automate.

Each drone is its own PX4 instance and its own copy of your node. Launch the
first, wait for Gazebo to finish loading, then the next.

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
  ./build/px4_sitl_default/bin/px4 -i 1

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_SIM_MODEL=gz_x500 \
  ./build/px4_sitl_default/bin/px4 -i 2
```

`PX4_GZ_MODEL_POSE` is in Gazebo's world frame, which is ENU: `"0,3"` is x 0,
y 3, so 3 m north. PX4's own local frame is NED, so that offset arrives at your
node as `x = 3`.

Three things become instance-dependent:

| | Instance 0 | Instance N |
|---|---|---|
| Topics | bare `/fmu/...`, like a real vehicle | `/px4_N/fmu/...` |
| MAVLink system id | 1 | N + 1 |
| GCS port | 18570 | 18570 + N |

The nodes take the first two as parameters. PX4 ignores commands addressed to a
different system, so a wrong system id produces a node that runs while nothing
arms.

```bash
ros2 run py_single_square listener --ros-args -p px4_ns:=px4_1
ros2 run py_single_square fly --ros-args -p px4_ns:=px4_2 -p sys_id:=3
ros2 run cpp_single_hop hop --ros-args -p px4_ns:=px4_2 -p system_id:=3
```

`drone-gcs` prints one line per vehicle while they boot:

```
GCS link running from /usr/local/bin/drone-gcs
Listening on udpin:0.0.0.0:14550, and heartbeating SITL instances 0-10 on ports 18570-18580.
Linked to PX4 system 2. Datalink failsafe satisfied for 1 vehicle(s); they can arm.
Linked to PX4 system 3. Datalink failsafe satisfied for 2 vehicle(s); they can arm.
```

`--expect 3` checks that count against what you launched, and reports a link it
loses or regains:

```bash
drone-gcs --expect 3
```

If that count stops short of the number you launched, the vehicles it never
names sit on the pad unable to arm, because each vehicle enforces the datalink
failsafe separately.

That first line names the file that is actually running. `drone-gcs` runs the
repo file directly, so an edit takes effect on the next run.

Two drones flying the same path independently form a fleet. A swarm holds a
formation, which needs a shared frame and a shared clock. `src/swarm_naive`
flies without either, `src/swarm_comms` implements both, and
[../swarm/SWARM.md](../swarm/SWARM.md) documents the difference.

## Problems

The smoke test names the broken piece:

```bash
./scripts/native/smoke-test.sh
```

| Symptom | Cause | Fix |
|---|---|---|
| `Arming denied: Resolve system health failures first` | No GCS link | Run `drone-gcs`. Not `NAV_DLL_ACT 0` |
| Gazebo shows demo worlds, no drone | You ran `gz sim` | Run `PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4` |
| `Waiting for Gazebo world...` then a timeout | The world's name does not match its file name | [WORLDS.md](WORLDS.md) |
| No `/fmu/` topics | Agent not running, or started after PX4 | Start agent, restart PX4 |
| Callback never fires but `ros2 topic echo` shows data | QoS mismatch | `PX4_QOS` |
| `Standalone PX4 launch, waiting for Gazebo` | `PX4_GZ_STANDALONE` set | `unset PX4_GZ_STANDALONE` |
| Gazebo at 2 fps | Software rendering | The smoke test's graphics section names the renderer |
| Offboard rejected | Setpoint stream not running | Publish setpoints before the mode change |
| Arms then disarms | Failsafe tripped | `pxh> commander status` |
| `ros2 run py_single_square` not found | Overlay not sourced | `source install/setup.bash` |
| Node runs but never arms | Wrong `system_id` for that instance | Instance N needs system id N + 1 |
| First drone flies, the rest never arm | `drone-gcs` is serving only one vehicle | It should print one linked system per drone. Check the path it prints is the copy you edited |
| New drones spawn into a field that already has drones in it | A Gazebo server survived the last run | `pkill -f 'gz sim'` |

## Reference

- [HOW-IT-WORKS.md](HOW-IT-WORKS.md), the pipeline, the three rules PX4
  imposes, and how to go from one drone to a swarm. Start here for code.
- [../swarm/SWARM.md](../swarm/SWARM.md), the two multi-drone examples, the
  shared frame, the shared clock, and what drones say to each other.
- [UNDER-THE-SCRIPTS.md](UNDER-THE-SCRIPTS.md), the same stack with no scripts
  of ours in it, and what each process becomes on real hardware.
- [WORLDS.md](WORLDS.md), loading a different map, writing your own, and the
  four things in a world the flight code can feel.
- [PX4 ROS 2 User Guide](https://docs.px4.io/v1.16/en/ros2/user_guide)
- [PX4 Offboard Control Example](https://docs.px4.io/v1.16/en/ros2/offboard_control)
- [Multi-Vehicle Simulation](https://docs.px4.io/main/en/ros2/multi_vehicle)
- [uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [px4_msgs release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg)
- [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md)