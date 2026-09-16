# Running the simulator and writing your first node

Everything runs in the container. `Terminal > New Terminal` in VS Code puts you
in it. Not set up yet: [docs/SETUP.md](docs/SETUP.md).

## Start the stack

Four terminals:

```bash
MicroXRCEAgent udp4 -p 8888     # 1. PX4 to ROS 2 bridge
drone-gcs                       # 2. ground control station link
drone-sim                       # 3. simulator, gives you a pxh> prompt
ros2 topic list | grep fmu      # 4. your side
```

Watch it at <http://localhost:6080>, or in a VNC client on `localhost:5901`.
Both are covered in [docs/SETUP.md](docs/SETUP.md). Terminal 4 printing nothing
means the agent is not connected.

`drone-gcs` is required. PX4's x500 airframe sets `NAV_DLL_ACT 2`, return on
datalink loss, so it will not arm without a GCS, same as a real aircraft.
Skipping it gives `Arming denied: Resolve system health failures first`. Do not
fix that with `param set NAV_DLL_ACT 0`: that deletes the failsafe instead of
supplying the link, and the vehicle stops matching the real one when telemetry
drops. Run QGroundControl against udp 14550 if you prefer a real GCS. Killing
`drone-gcs` mid-flight tests the failsafe.

`drone-sim` wraps `make px4_sitl gz_x500`. Bare `gz sim` does not start the
drone, it opens Gazebo's world chooser. For swarms use
`HEADLESS=1 make px4_sitl gz_x500`.

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
far out. Scroll to zoom.

## Build and run your code

Write code in `/workspaces/DroneSwarming`, the repo bind-mounted from your
laptop. `~/PX4-Autopilot` and `~/ros2_ws` are recreated on container rebuild.

```bash
cd /workspaces/DroneSwarming
colcon build --symlink-install
source install/setup.bash
```

New terminals source the overlay for you. `--symlink-install` means editing a
Python node takes effect on the next `ros2 run`; rebuild after adding files or
changing `setup.py`.

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

If that count stops short of the number you launched, the vehicles it never
names are the ones that will sit on the pad unable to arm, because each vehicle
enforces the datalink failsafe separately.

That first line is worth reading. The scripts are baked into the container
image, so a copy you edited in the repo is not the copy on your PATH until the
image is rebuilt; run the repo copy directly to test a change:

```bash
/workspaces/DroneSwarming/.devcontainer/scripts/drone-gcs
```

Two drones flying the same path independently is a fleet, not a swarm. Making
them hold a formation needs a shared frame and a shared clock, both covered in
[docs/HOW-IT-WORKS.md](docs/HOW-IT-WORKS.md).

## Problems

Run `./smoke-test.sh` first.

| Symptom | Cause | Fix |
|---|---|---|
| `Arming denied: Resolve system health failures first` | No GCS link | Run `drone-gcs`. Not `NAV_DLL_ACT 0` |
| Gazebo shows demo worlds, no drone | You ran `gz sim` | `drone-sim` |
| Nothing at localhost:6080, or VNC will not connect | Desktop not up | `drone-gui` |
| No `/fmu/` topics | Agent not running, or started after PX4 | Start agent, restart PX4 |
| Callback never fires but `ros2 topic echo` shows data | QoS mismatch | `PX4_QOS` |
| `Standalone PX4 launch, waiting for Gazebo` | `PX4_GZ_STANDALONE` set | `unset PX4_GZ_STANDALONE` |
| Gazebo at 2 fps | Software rendering | Smoke test section 5 names the renderer |
| Offboard rejected | Setpoint stream not running | Publish setpoints before the mode change |
| Arms then disarms | Failsafe tripped | `pxh> commander status` |
| `ros2 run drone_vip` not found | Overlay not sourced | `source install/setup.bash` |
| Node runs but never arms | Wrong `system_id` for that instance | Instance N needs system id N + 1 |
| First drone flies, the rest never arm | `drone-gcs` is serving only one vehicle | It should print one linked system per drone. Check the path it prints is the copy you edited |

## Reference

- [docs/HOW-IT-WORKS.md](docs/HOW-IT-WORKS.md) — the pipeline, the three rules
  PX4 imposes, and how to go from one drone to a swarm. Start here for code.
- [PX4 ROS 2 User Guide](https://docs.px4.io/v1.16/en/ros2/user_guide)
- [PX4 Offboard Control Example](https://docs.px4.io/v1.16/en/ros2/offboard_control)
- [Multi-Vehicle Simulation](https://docs.px4.io/main/en/ros2/multi_vehicle)
- [uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [px4_msgs release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg)
- [docs/SIMULATION-FIDELITY.md](docs/SIMULATION-FIDELITY.md)
