# Running the simulator and writing your first node

Every command here is the same whether you run the dev container or a native
install. Where the two genuinely differ, both are given.

Not set up yet: [docs/SETUP.md](docs/SETUP.md) for the dev container, where
`Terminal > New Terminal` in VS Code puts you inside it, or
[docs/NATIVE-SETUP.md](docs/NATIVE-SETUP.md) for a native install, where
`scripts/native/env.sh` has to be sourced in the terminal.

## Start the stack

Four terminals:

```bash
MicroXRCEAgent udp4 -p 8888     # 1. PX4 to ROS 2 bridge
drone-gcs                       # 2. ground control station link
drone-sim                       # 3. simulator, gives you a pxh> prompt
ros2 topic list | grep fmu      # 4. your side
```

Terminal 4 printing nothing means the agent is not connected.

Where Gazebo appears is the one difference:

| | |
|---|---|
| Dev container | A desktop you open in a browser at <http://localhost:6080>, or in a VNC client on `localhost:5901`. Both are covered in [docs/SETUP.md](docs/SETUP.md) |
| Native | A window on your own desktop |

`drone-gcs` is required. PX4's x500 airframe sets `NAV_DLL_ACT 2`, return on
datalink loss, so it will not arm without a GCS, same as a real aircraft.
Skipping it gives `Arming denied: Resolve system health failures first`. Do not
fix that with `param set NAV_DLL_ACT 0`: that deletes the failsafe instead of
supplying the link, and the vehicle stops matching the real one when telemetry
drops. Run QGroundControl against udp 14550 if you prefer a real GCS. Killing
`drone-gcs` mid-flight tests the failsafe.

`drone-sim` wraps `make px4_sitl gz_x500`. Bare `gz sim` does not start the
drone, it opens Gazebo's world chooser. For swarms and long runs use
`HEADLESS=1 drone-sim`, which skips rendering entirely.

`drone-sim` and `drone-gcs` are conveniences, and they hide which parts of this
stack are real flight software. To run the same thing with neither of them, and
to see which processes also exist on the aircraft, read
[docs/UNDER-THE-SCRIPTS.md](docs/UNDER-THE-SCRIPTS.md). The short version is one
environment variable and a binary:

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

Blank grey 3D view is normal: PX4's world is untextured and the camera starts
far out. Scroll to zoom. To fly somewhere with scenery in it, or to build a map
of your own, see [docs/WORLDS.md](docs/WORLDS.md):

```bash
PX4_GZ_WORLD=lawn drone-sim
```

## Build and run your code

Write code in this repo. `colcon` has to run from the repo root, the directory
that contains `src/`:

```bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means editing a Python node takes effect on the next
`ros2 run`; rebuild after adding files or changing `setup.py`.

| | |
|---|---|
| Dev container | The repo is bind mounted at `/workspaces/DroneSwarming`. New terminals source the overlay for you. `~/PX4-Autopilot` and `~/ros2_ws` belong to the image and are recreated when the container is rebuilt |
| Native | The repo is wherever you cloned it. New terminals source the overlay once `scripts/native/env.sh` is in your `~/.bashrc`. `~/PX4-Autopilot` and `~/ros2_ws` are yours and persist |

Starting a package of your own rather than editing these two:
[docs/NEW-PACKAGE.md](docs/NEW-PACKAGE.md).

`src/drone_vip` has two Python nodes:

```bash
ros2 run drone_vip listener      # position, altitude, heading, mode, 1 Hz
ros2 run drone_vip fly           # arms, goes offboard, flies a square, lands
ros2 run drone_vip fly --ros-args -p altitude:=8.0 -p side:=10.0
```

`src/drone_swarm` is the C++ package, with one node:

```bash
ros2 run drone_swarm hop         # takes off, flies 3 m forward, lands
ros2 run drone_swarm hop --ros-args -p altitude_m:=6.0 -p forward_m:=10.0
```

`hop` is the starting point for flight code, and
[docs/HOW-IT-WORKS.md](docs/HOW-IT-WORKS.md) walks through it and the whole
pipeline behind it. Read that before writing your own node.

`src/swarm_naive` and `src/swarm_comms` are the two multi-drone examples. Each
starts its own PX4 instances, so they replace `drone-sim` rather than running
alongside it:

```bash
ros2 launch swarm_naive swarm.launch.py    # 3 drones, none aware of the others
ros2 launch swarm_comms swarm.launch.py    # the same flight, with them talking
ros2 launch swarm_naive swarm.launch.py --show-args
```

[docs/SWARM.md](docs/SWARM.md) covers both, and everything below about running
several vehicles by hand.

To start a package of your own rather than editing these two, follow
[docs/NEW-PACKAGE.md](docs/NEW-PACKAGE.md). It covers both build types, Python
and C++, from `ros2 pkg create` through to `ros2 run`.

## QoS

PX4 publishes BEST_EFFORT with TRANSIENT_LOCAL durability. ROS 2 subscribers
default to RELIABLE and VOLATILE. Incompatible, so your subscription is created
without error, never matches, and the callback never fires. Nothing is logged.

`ros2 topic echo` does not have this problem on Jazzy, it negotiates QoS:

```bash
ros2 topic echo /fmu/out/vehicle_local_position     # works, no flags needed
```

So the topic looks healthy while your node gets nothing. Measured here: default
echo gets data, a plain rclpy subscription with default QoS on the same topic
gets zero callbacks.

Use the shared profile on anything touching `/fmu/`:

```python
from drone_vip.px4_qos import PX4_QOS

self.create_subscription(
    VehicleLocalPosition, '/fmu/out/vehicle_local_position',
    self.callback, PX4_QOS)
```

## Offboard mode

PX4 will not enter offboard until it is already receiving setpoints, and drops
out if the stream stops for more than about half a second. Publish
`OffboardControlMode` every timer tick, before and after the mode switch.
`takeoff_and_fly.py` does 10 Hz and sends 10 setpoints before switching.

If the drone refuses to arm or falls back to another mode, this is usually why.

## More than one drone

Two launch files do all of this for you, and
[docs/SWARM.md](docs/SWARM.md) is the guide to them:

```bash
MicroXRCEAgent udp4 -p 8888                       # 1. same as always
drone-gcs                                         # 2. same as always
ros2 launch swarm_naive swarm.launch.py           # 3. replaces drone-sim
```

The rest of this section is what those launch files automate. Read it when one
of them misbehaves.

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

The nodes take the first two as parameters. The system id matters: PX4 ignores
commands addressed to a different system, so getting it wrong looks like a node
that runs fine while nothing arms.

```bash
ros2 run drone_vip listener --ros-args -p px4_ns:=px4_1
ros2 run drone_vip fly --ros-args -p px4_ns:=px4_2 -p sys_id:=3
ros2 run drone_swarm hop --ros-args -p px4_ns:=px4_2 -p system_id:=3
```

Watch `drone-gcs` while they boot. It prints one line per vehicle, and the count
is the thing to check:

```
GCS link running from /usr/local/bin/drone-gcs
Listening on udpin:0.0.0.0:14550, and heartbeating SITL instances 0-9 on ports 18570-18579.
Linked to PX4 system 2. Datalink failsafe satisfied for 1 vehicle(s); they can arm.
Linked to PX4 system 3. Datalink failsafe satisfied for 2 vehicle(s); they can arm.
```

Pass `--expect 3` and it checks that count against what you launched, and
reports a link it loses or regains:

```bash
drone-gcs --expect 3
```

If that count stops short of the number you launched, the vehicles it never
names are the ones that will sit on the pad unable to arm, because each vehicle
enforces the datalink failsafe separately.

That first line is worth reading, because it names the file that is actually
running.

In the dev container the scripts are baked into the image, so a copy you edited
in the repo is not the copy on your `PATH` until the image is rebuilt. Run the
repo copy directly to test a change:

```bash
.devcontainer/scripts/drone-gcs
```

Natively, `drone-gcs` runs that same repo file already, so an edit takes effect
on the next run.

Two drones flying the same path independently is a fleet, not a swarm. Making
them hold a formation needs a shared frame and a shared clock.
`src/swarm_naive` shows what you get without either, `src/swarm_comms` solves
both, and [docs/SWARM.md](docs/SWARM.md) explains how.

## Problems

Run the smoke test first. It names the broken piece:

```bash
./smoke-test.sh                     # dev container
./scripts/native/smoke-test.sh      # native
```

| Symptom | Cause | Fix |
|---|---|---|
| `Arming denied: Resolve system health failures first` | No GCS link | Run `drone-gcs`. Not `NAV_DLL_ACT 0` |
| Gazebo shows demo worlds, no drone | You ran `gz sim` | `drone-sim` |
| `Waiting for Gazebo world...` then a timeout | The world's name does not match its file name | [docs/WORLDS.md](docs/WORLDS.md) |
| Nothing at localhost:6080, or VNC will not connect (dev container) | Desktop not up | `drone-gui` |
| No `/fmu/` topics | Agent not running, or started after PX4 | Start agent, restart PX4 |
| Callback never fires but `ros2 topic echo` shows data | QoS mismatch | `PX4_QOS` |
| `Standalone PX4 launch, waiting for Gazebo` | `PX4_GZ_STANDALONE` set | `unset PX4_GZ_STANDALONE` |
| Gazebo at 2 fps | Software rendering | The smoke test's graphics section names the renderer |
| Offboard rejected | Setpoint stream not running | Publish setpoints before the mode change |
| Arms then disarms | Failsafe tripped | `pxh> commander status` |
| `ros2 run drone_vip` not found | Overlay not sourced | `source install/setup.bash` |
| Node runs but never arms | Wrong `system_id` for that instance | Instance N needs system id N + 1 |
| First drone flies, the rest never arm | `drone-gcs` is serving only one vehicle | It should print one linked system per drone. Check the path it prints is the copy you edited |
| New drones spawn into a field that already has drones in it | A Gazebo server survived the last run | `pkill -f 'gz sim'` |

## Reference

- [docs/HOW-IT-WORKS.md](docs/HOW-IT-WORKS.md), the pipeline, the three rules
  PX4 imposes, and how to go from one drone to a swarm. Start here for code.
- [docs/SWARM.md](docs/SWARM.md), the two multi-drone examples, the shared frame,
  the shared clock, and what drones say to each other.
- [docs/UNDER-THE-SCRIPTS.md](docs/UNDER-THE-SCRIPTS.md), the same stack with no
  scripts of ours in it, and what each process becomes on real hardware.
- [docs/WORLDS.md](docs/WORLDS.md), loading a different map, writing your own,
  and the four things in a world the flight code can feel.
- [PX4 ROS 2 User Guide](https://docs.px4.io/v1.16/en/ros2/user_guide)
- [PX4 Offboard Control Example](https://docs.px4.io/v1.16/en/ros2/offboard_control)
- [Multi-Vehicle Simulation](https://docs.px4.io/main/en/ros2/multi_vehicle)
- [uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [px4_msgs release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg)
- [docs/SIMULATION-FIDELITY.md](docs/SIMULATION-FIDELITY.md)
