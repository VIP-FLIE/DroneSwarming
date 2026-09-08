# Running the simulator and writing your first node

Everything here runs inside the dev container. Open a terminal in VS Code
(`Terminal > New Terminal`) and you are already in it.

## The three terminals

PX4 SITL and ROS 2 do not talk to each other directly. The Micro XRCE-DDS
Agent sits between them and translates PX4's internal messages into ROS 2
topics. So you need three things running.

**Terminal 1, the bridge:**

```bash
MicroXRCEAgent udp4 -p 8888
```

Leave it running. It prints nothing useful until PX4 connects, then it logs a
burst of `create_topic` lines. That burst is how you know the link came up.

**Terminal 2, the simulator:**

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

A Gazebo window opens with an X500 quadrotor on the ground, and the terminal
drops into PX4's own shell, the `pxh>` prompt. First launch takes longer
because Gazebo is loading model and world assets.

If the window is painfully slow, your machine is doing software rendering.
Run it without a GUI instead:

```bash
HEADLESS=1 make px4_sitl gz_x500
```

**Terminal 3, your side:**

```bash
ros2 topic list | grep fmu
```

You should see a long list of `/fmu/out/...` and `/fmu/in/...` topics. If that
list is empty, the agent is not connected. Check Terminal 1.

## Fly it before you write any code

At the `pxh>` prompt in Terminal 2:

```
pxh> commander takeoff
pxh> commander land
```

That is the whole loop. The drone climbs to its default takeoff altitude,
hovers, and comes back down, and you can watch it in the Gazebo window. Other
things worth trying from that prompt:

```
pxh> commander status          # arming state, current flight mode, failsafes
pxh> listener vehicle_local_position   # raw uORB message, PX4's own view
pxh> commander mode posctl     # switch flight mode
```

`listener` at the `pxh>` prompt is PX4's internal message dumper. It is a
useful sanity check because it works with no ROS 2 and no agent involved. If
`pxh> listener vehicle_local_position` shows data and `ros2 topic echo` does
not, the problem is the bridge and not the simulator.

## Where your code goes, and why it matters

| Path | What it is | Survives a container rebuild? |
|---|---|---|
| `~/PX4-Autopilot` | PX4 source, cloned by the Dockerfile | No |
| `~/ros2_ws` | px4_msgs, built by build-workspace.sh | No |
| `/workspaces/<your-repo>` | Your git repo, bind-mounted from your machine | **Yes** |

The first two live inside the container image. Rebuild the container and they
are recreated from scratch, so anything you write there is gone. Your repo is
a bind mount, meaning it is the same directory on your laptop that you cloned.

Write all of your code in the repo.

## Build and run the starter package

The `src/drone_vip` package in this repo has two nodes to start from.

```bash
cd /workspaces/<your-repo>
colcon build --symlink-install
source install/setup.bash
```

New terminals source the overlay automatically. You only need that `source`
line in the terminal where you just ran `colcon build`.

`--symlink-install` symlinks Python files instead of copying them, so editing
a node takes effect on the next `ros2 run` with no rebuild. You still need to
rebuild after adding a new file or changing `setup.py`.

**Node 1, read the drone's state:**

```bash
ros2 run drone_vip listener
```

Prints position, altitude, heading, arming state and flight mode once a
second. Run `commander takeoff` in Terminal 2 and watch the numbers move.

**Node 2, fly a square:**

```bash
ros2 run drone_vip fly
ros2 run drone_vip fly --ros-args -p altitude:=8.0 -p side:=10.0
```

Arms the drone, engages offboard mode, climbs, flies a square, and lands.
Roughly 120 lines, and it is the skeleton every formation-flight behavior you
write this semester will be built on.

## The one gotcha that will cost you an afternoon

PX4 publishes its topics as **BEST_EFFORT** with **TRANSIENT_LOCAL**
durability. ROS 2 subscribers default to **RELIABLE** with **VOLATILE**. Those
are incompatible, so the subscription gets created without any error, never
matches the publisher, and your callback never fires. There is no warning.

`ros2 topic echo` has the same default, which is why this looks like a dead
topic:

```bash
ros2 topic echo /fmu/out/vehicle_local_position                          # silent
ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort   # works
```

In your own nodes, import the profile from `drone_vip.px4_qos` and pass it to
every publisher and subscriber that touches a `/fmu/` topic:

```python
from drone_vip.px4_qos import PX4_QOS

self.create_subscription(
    VehicleLocalPosition, '/fmu/out/vehicle_local_position',
    self.callback, PX4_QOS)
```

## Offboard mode has a rule

PX4 refuses to enter offboard mode until it is already receiving a stream of
setpoints, and it drops out of offboard if that stream stops for more than
about half a second. So you publish `OffboardControlMode` on every timer tick,
before the mode switch and after it, for as long as you want to stay in
offboard. `takeoff_and_fly.py` does this at 10 Hz and sends 10 setpoints
before requesting the mode change.

If your drone refuses to arm or immediately falls back to another mode, this
is almost always why.

## More than one drone

Start the agent as usual, then launch instances by index. Start the first one
and wait for Gazebo to come up before starting the second.

```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
  ./build/px4_sitl_default/bin/px4 -i 1

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_SIM_MODEL=gz_x500 \
  ./build/px4_sitl_default/bin/px4 -i 2
```

`PX4_GZ_MODEL_POSE="0,3"` places the second drone 3 metres east so they do not
spawn inside each other.

Topic namespacing follows the instance index. Instance 0 publishes on bare
`/fmu/...` to match how a real vehicle behaves. Every instance above 0 gets a
`px4_<N>` prefix:

```
/px4_1/fmu/out/vehicle_local_position
/px4_2/fmu/out/vehicle_local_position
```

Both starter nodes take a `px4_ns` parameter for this:

```bash
ros2 run drone_vip listener --ros-args -p px4_ns:=px4_1
ros2 run drone_vip fly --ros-args -p px4_ns:=px4_2 -p sys_id:=2
```

One agent handles every instance. You do not start a second one.

## When something is wrong

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` shows no `/fmu/` topics | Agent not running, or started after PX4 | Start `MicroXRCEAgent udp4 -p 8888`, then restart PX4 |
| `ros2 topic echo` prints nothing, topic exists | QoS mismatch | Add `--qos-reliability best_effort` |
| Node runs, callback never fires | QoS mismatch | Use `PX4_QOS` from `drone_vip.px4_qos` |
| `INFO [init] Standalone PX4 launch, waiting for Gazebo` | `PX4_GZ_STANDALONE` is set in your shell | `unset PX4_GZ_STANDALONE` |
| Gazebo window never opens | X11 forwarding | On the host: `xhost +local:docker`, then rebuild |
| Gazebo opens but runs at 2 fps | Software rendering | Use `HEADLESS=1`, or switch to the `gpu` devcontainer config |
| Offboard mode rejected | Setpoint stream not running yet | Publish setpoints before requesting the mode change |
| Arms, then instantly disarms | A failsafe tripped | `pxh> commander status` names it |
| `ros2 run drone_vip ...` package not found | Overlay not sourced | `source install/setup.bash`, or open a new terminal |

## Reference

- [PX4 ROS 2 User Guide](https://docs.px4.io/v1.16/en/ros2/user_guide)
- [PX4 ROS 2 Offboard Control Example](https://docs.px4.io/v1.16/en/ros2/offboard_control)
- [Multi-Vehicle Simulation with ROS 2](https://docs.px4.io/main/en/ros2/multi_vehicle)
- [uXRCE-DDS bridge internals](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [px4_msgs, release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg)
