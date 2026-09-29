# py_single_square

One drone, two Python nodes, and the QoS module the other Python code in this
repo imports.

| Node | Command | What it does |
|---|---|---|
| `listener` | `ros2 run py_single_square listener` | Prints the drone's position, altitude, heading, arming state and flight mode once a second. Sends nothing to the drone |
| `fly` | `ros2 run py_single_square fly` | Arms, takes off to 5 m, flies a 5 m square, lands |

## 1. What the drone does

### `fly`

The drone climbs straight up to 5 m, then flies the four sides of a square and
lands where it started.

| Waypoint | North | East | Altitude | Leg |
|---|---|---|---|---|
| 0 | 0 | 0 | 5 m | Straight up off the pad |
| 1 | 5 | 0 | 5 m | North |
| 2 | 5 | 5 | 5 m | East |
| 3 | 0 | 5 | 5 m | South |
| 4 | 0 | 0 | 5 m | West, back over the pad. Then land |

The nose faces north the whole time (yaw 0), so the drone flies forwards on the
first side, sideways on the second and fourth, and backwards on the third.

### `listener`

Nothing moves. It reads two PX4 topics and prints one line a second. Run it in
a spare terminal alongside `fly`, or alongside any other flight node, to watch
what PX4 reports.

## 2. What you learn from it

| Concept | Where |
|---|---|
| The PX4 offboard handshake in Python: stream, arm, switch to offboard, fly, land | `takeoff_and_fly.py`, `_tick()` |
| The QoS profiles PX4 requires, in one importable module | `px4_qos.py` |
| Reading PX4 telemetry and caching the latest value | `listener.py` |
| Turning PX4's numeric enums into names at runtime | `_constants()` in `listener.py` |
| A flight as a list of waypoints and an index | `self.waypoints` in `takeoff_and_fly.py` |
| The NED frame, where up is negative | Both nodes |
| Packaging Python for ROS 2: entry points, `setup.cfg`, the resource marker | `setup.py`, `setup.cfg`, `resource/` |

`fly` is the Python counterpart of [`cpp_single_hop`](../cpp_single_hop/README.md).
Section 4 lists where the two differ. Read `hop.cpp` for the fully commented
version of the PX4 rules.

## 3. How to run it

### Build

From the repo root, the directory that contains `src/`:

```bash
colcon build --symlink-install --packages-select py_single_square
source install/setup.bash
```

With `--symlink-install`, an edit to an existing `.py` file takes effect on the
next `ros2 run`. Rebuild after adding a file or changing `setup.py`.

### Run in the simulator

Four terminals, plus a fifth for `listener` if you want it. In the dev container
every terminal is ready to use. On a native install, each terminal needs
`source scripts/native/env.sh` first, unless it is in your `~/.bashrc`.

```bash
MicroXRCEAgent udp4 -p 8888      # 1. PX4 to ROS 2 bridge. Start it first
```

```bash
drone-gcs                        # 2. ground station link. PX4 will not arm without it
```

```bash
drone-sim                        # 3. PX4 and Gazebo, one x500
```

```bash
ros2 run py_single_square fly    # 4. the flight
```

```bash
ros2 run py_single_square listener   # 5. optional, watch it
```

Start `fly` after terminal 3 prints `Ready for takeoff!`. `fly` only retries
arming for 3 s, so starting it on a vehicle that is still booting can fail.

`HEADLESS=1 drone-sim` skips rendering. The flight is identical.

`fly` output from a real run on a freshly started vehicle:

```
[INFO] [takeoff_and_fly]: target altitude 5.0 m, 5.0 m square, talking to /fmu/...
[INFO] [takeoff_and_fly]: sending arm command (will retry)
[INFO] [takeoff_and_fly]: armed, switching to offboard mode
[INFO] [takeoff_and_fly]: waypoint 1/4: n=5.0 e=0.0 alt=5.0
[INFO] [takeoff_and_fly]: waypoint 2/4: n=5.0 e=5.0 alt=5.0
[INFO] [takeoff_and_fly]: waypoint 3/4: n=0.0 e=5.0 alt=5.0
[INFO] [takeoff_and_fly]: waypoint 4/4: n=0.0 e=0.0 alt=5.0
[INFO] [takeoff_and_fly]: square complete, landing
```

`waypoint 1/4` prints when the drone reaches waypoint 0 and starts towards 1.
There is no message for touchdown. Watch Gazebo or `listener` for `DISARMED`,
then Ctrl-C the node.

`listener` output with the drone on the pad:

```
[INFO] [listener]: listening on /fmu/out/...
[INFO] [listener]: n=   0.02  e=  -0.04  d=   0.05  (alt -0.05 m)  heading=  1.67 rad  DISARMED/AUTO_LOITER
```

A few centimetres of position on the pad is estimator noise. `AUTO_LOITER` is
PX4's Hold mode, which is where a freshly booted vehicle sits. Heading is in
radians, measured from north towards east.

### Running `fly` a second time

The line `armed, switching to offboard mode` can be missing on a second run,
with the flight otherwise normal. After a landing, PX4 returns to the last mode
the vehicle was able to arm in, which is offboard from the previous run
(`UserModeIntention::onDisarm()` in PX4 v1.16.1). The vehicle arms directly into
offboard and the node skips the mode request.

### Known problem: the offboard request is sent once

`fly` sends the switch-to-offboard command a single time. In one of the runs
used to write this README, on a freshly started vehicle, PX4 armed and never
entered offboard. The PX4 console showed:

```
INFO  [commander] Armed by external command
INFO  [commander] Disarmed by auto preflight disarming
```

PX4 disarms an armed vehicle that does not take off within its preflight
timeout. The node then printed `arming failed after 30 retries` on every tick
until it was stopped. The next run on the same vehicle flew normally.

If this happens, Ctrl-C `fly` and run it again. `hop` in `cpp_single_hop`
repeats the offboard request every tick until PX4 confirms it, which is the fix.

### Parameters

`fly`:

| Parameter | Default | Meaning |
|---|---|---|
| `altitude` | 5.0 | Height of the square above the pad, in metres |
| `side` | 5.0 | Side length of the square, in metres |
| `px4_ns` | empty | Topic namespace. Empty means bare `/fmu/...`, which is SITL instance 0 |
| `sys_id` | 1 | MAVLink system id of the vehicle. SITL instance N is system N + 1 |

`listener`:

| Parameter | Default | Meaning |
|---|---|---|
| `px4_ns` | empty | Same as above |

The names differ from `hop`'s: `sys_id` here is `system_id` there, and
`altitude` here is `altitude_m` there.

```bash
ros2 run py_single_square fly --ros-args -p altitude:=8.0 -p side:=10.0
ros2 run py_single_square fly --ros-args -p px4_ns:=px4_1 -p sys_id:=2
ros2 run py_single_square listener --ros-args -p px4_ns:=px4_1
```

### Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `listener` prints `no messages yet` every second | Agent not running, agent started after PX4, or `px4_ns` wrong | Start the agent, then restart `drone-sim` |
| `arming failed after 30 retries` within a few seconds of starting | PX4 was not ready to arm within 3 s, usually no `drone-gcs` or a vehicle still booting | Check `drone-gcs` is running, wait for `Ready for takeoff!`, run again |
| `armed, switching to offboard mode`, then `arming failed` repeating | The offboard request was not acted on and PX4 disarmed | See the known problem above |
| `No executable found` | Built before an entry point was added | Rebuild, then `source install/setup.bash` |
| `ModuleNotFoundError: No module named 'px4_msgs'` | The `~/ros2_ws` overlay is not sourced | Open a fresh terminal, or source `scripts/native/env.sh` |

## 4. How it works

### Files

| File | What it is |
|---|---|
| `py_single_square/px4_qos.py` | `PX4_QOS`, `PX4_CMD_QOS` and `topic_prefix()`. Imported by both nodes, and by any other Python package that talks to PX4 |
| `py_single_square/listener.py` | The `listener` node |
| `py_single_square/takeoff_and_fly.py` | The `fly` node |
| `py_single_square/__init__.py` | Empty. Makes the directory an importable Python package |
| `setup.py` | What gets installed, and the two entry points |
| `setup.cfg` | Installs the entry-point scripts into `lib/py_single_square`, the directory `ros2 run` searches |
| `resource/py_single_square` | Empty marker file that registers the package in the ament index |
| `package.xml` | Package name, build type `ament_python`, runtime dependencies `rclpy` and `px4_msgs` |

### `px4_qos.py`

| Name | Value | Used for |
|---|---|---|
| `PX4_QOS` | Best effort, transient local, keep last 1 | Every `/fmu/out/` subscription, and the setpoint publishers |
| `PX4_CMD_QOS` | Reliable, transient local, keep last 1 | The `/fmu/in/vehicle_command` publisher |
| `topic_prefix(ns)` | `''` for an empty namespace, `'/px4_1'` for `'px4_1'` | Building topic names for any instance |

PX4 publishes `/fmu/out/` as best effort with transient local durability. A
default rclpy subscription is reliable and volatile, which is incompatible: the
subscription is created, never connects, and its callback never fires.
`ros2 topic echo` negotiates QoS for itself, so the topic looks healthy from the
command line while the node receives nothing.

`vehicle_command` is the exception. PX4 serves it as a reliable DDS replier, so
a best effort publisher's commands are dropped without a warning.

The module is separate so other packages can import it instead of copying it.
[docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md) shows a new package doing
exactly that, with `<depend>py_single_square</depend>` in its `package.xml`.

### `listener.py`

| Part | What it does |
|---|---|
| `__init__` | Declares `px4_ns`, subscribes to `vehicle_local_position` and `vehicle_status_v1` with `PX4_QOS`, starts a 1 s timer |
| `_on_position`, `_on_status` | Store the latest message. Nothing else |
| `_report` | Once a second, prints the latest position and status, or warns if nothing has arrived |
| `_constants()` | Builds `{4: 'AUTO_LOITER', 14: 'OFFBOARD', ...}` from the constants on the `px4_msgs` class |

Decoupling the callbacks from the printing means the node prints at 1 Hz no
matter how fast PX4 publishes, and always prints the newest value.

`_constants()` reads the names off `VehicleStatus` at runtime instead of
hardcoding a table. If `px4_msgs` changes its enums, the labels change with it.

`alt` is printed as `-z`, because z in NED points down.

### `takeoff_and_fly.py`

Everything runs in `_tick()`, on a 10 Hz timer. There is no state variable.
Each phase is an early return:

| Step in `_tick()` | Condition to get past it | What it does |
|---|---|---|
| Stream | None, runs every tick | Publishes `OffboardControlMode` with `position = True` |
| Landing check | `self.landing` is false | Once landing, returns here, so no more setpoints are sent |
| Setpoint | None | Publishes the current waypoint as a `TrajectorySetpoint` |
| Phase 0 | 10 ticks have passed | Builds up 1 s of setpoint stream before arming |
| Phase 1 | PX4 reports armed | Sends an arm command every tick for 30 ticks (3 s), then logs an error every tick |
| Phase 2 | PX4 reports offboard | Sends `DO_SET_MODE` with `param1 = 1`, `param2 = 6` (offboard), once |
| Phase 3 | Position has arrived | Advances `waypoint_index` when within 0.5 m of the target. Past the last one, sends `NAV_LAND` and sets `self.landing` |

The setpoint during phases 0 to 2 is waypoint 0, above the pad. PX4 ignores it
until offboard is engaged, and the stream has to exist first or PX4 refuses the
mode switch.

Distance to a waypoint is the straight-line 3D distance between PX4's position
estimate and the target, north, east and down together. It moves on once that
distance is under 0.5 m.

`_stamp()` converts ROS time in nanoseconds to PX4's microseconds. PX4 ignores
setpoints with wrong timestamps without saying so.

`_send_command()` sets `target_system` to `sys_id`. PX4 drops commands
addressed to a different system, so a wrong `sys_id` looks like a node that runs
while nothing arms.

### Compared with `hop`

| | `fly` | `hop` in `cpp_single_hop` |
|---|---|---|
| Path | A list of waypoints and an index | One phase per leg |
| Arm retries | Every tick, for 3 s | Every 0.5 s, for 60 s |
| Arm failure message | Generic, every tick | Names PX4's failing check, every 5 s |
| Offboard request | Sent once | Sent every tick until PX4 confirms |
| After landing | Keeps publishing `OffboardControlMode` | Stops both streams |
| End of flight | No message at touchdown | Waits for PX4 to disarm and says so |
| Yaw | Explicitly 0, north | Left at the default, 0, north |
| Arrival radius | 0.5 m | 0.5 m |

Both keep running after the flight until Ctrl-C.

### Packaging

`setup.py` maps executable names to functions:

```python
'listener = py_single_square.listener:main',
'fly = py_single_square.takeoff_and_fly:main',
```

The left side is what `ros2 run` accepts, which is why the executable is `fly`
while the file is `takeoff_and_fly.py`. A node missing from this list builds and
then reports `No executable found`.

`setup.cfg` sends the generated scripts to `lib/py_single_square`. Without it
they go to `bin/`, where `ros2 run` does not look.

### Design decisions

| Decision | Reason |
|---|---|
| QoS in its own module | One definition for every Python node in the repo |
| `topic_prefix()` and `px4_ns` on single-drone nodes | The same node runs against instance 0, instance N, and a real aircraft |
| Enum names read from `px4_msgs` at runtime | Cannot drift from the message definitions the stack was built with |
| Waypoints as a list | Adding a corner is one line. Compare `hop`, where each leg is a phase |
| Landing handed to PX4 with `NAV_LAND` | PX4's land mode does ground detection and disarming, as on the real aircraft |
| Python | Edits take effect on the next run with `--symlink-install`, with no compile step |

### Where to go next

| To | Read |
|---|---|
| The fully commented C++ version | [`../cpp_single_hop`](../cpp_single_hop/README.md) |
| The pipeline behind both | [docs/HOW-IT-WORKS.md](../../docs/HOW-IT-WORKS.md) |
| Start your own Python package | [docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md) |
| Several drones | [`../swarm_naive`](../swarm_naive/README.md) |
