# swarm_naive

Several drones fly one shared ring, with no communication between them. Each
drone is given a slot number and a flight plan at launch and flies it open
loop.

This is the first of the two swarm packages. [`swarm_comms`](../swarm_comms/README.md)
is the same flight with the drones talking to each other. Read
[`cpp_single_hop`](../cpp_single_hop/README.md) before either: this package
uses its PX4 code unchanged.

## 1. What the drones do

Three drones by default, on pads in a row 5 m apart along east. Each one:

1. Climbs straight up off its own pad to its own altitude.
2. Flies to its own vertex of a shared ring, then once round the ring.
3. Flies back over its own pad and lands.

```
        north
          ^
          |              v0 (21,5)
          |            /           \
          |          /               \
          |  v2 (9,-1.9) ------------ v1 (9,11.9)
          |
          |    drone0    drone1    drone2
          +------o---------o---------o------> east
               (0,0)     (0,5)     (0,10)
```

The ring has radius 8 m, centre (13, 5), and one vertex per drone.
Coordinates are (north, east) in metres in the shared swarm frame, whose origin
is drone 0's pad. The legs of the ring are straight lines between vertices.

| Slot | Pad | Altitude | Ring order |
|---|---|---|---|
| 0 | (0, 0) | 4 m | v0, v1, v2, v0 |
| 1 | (0, 5) | 6 m | v1, v2, v0, v1 |
| 2 | (0, 10) | 8 m | v2, v0, v1, v2 |

Each drone enters at the vertex matching its slot, so they chase each other
round the ring. Each drone flies at a different altitude, so their paths cross
in plan view without the aircraft meeting.

No drone knows where any other drone is. Each starts its lap when it has
finished arming, and lands when its own route runs out.

## 2. What you learn from it

| Concept | Where |
|---|---|
| Running several PX4 instances: one process per drone, and what `-i` changes | `launch/swarm.launch.py` |
| Topic namespaces and MAVLink system ids per instance | `px4_ns`, `system_id` in the launch file |
| The shared-frame problem: every PX4 measures from its own pad | `to_local()`, `SwarmPoint`, `LocalPoint` |
| Using the type system to make frame mix-ups a compile error | `SwarmPoint` and `LocalPoint` |
| Gazebo spawn poses are ENU while PX4 is NED | `model_pose` in the launch file |
| Generating a route from a slot number | `build_route()`, `ring_vertex()` |
| Altitude layering as static deconfliction, and its limits | `station_altitude_m()` |
| Steering the nose along the direction of travel | `update_yaw_setpoint()` |
| ROS 2 launch files: arguments, `OpaqueFunction`, `ExecuteProcess`, `TimerAction`, shutdown handlers | `launch/swarm.launch.py` |
| What a swarm without communication cannot do | Header comment of `waypoint_drone.hpp` |

## 3. How to run it

### Build

From the repo root, the directory that contains `src/`:

```bash
colcon build --symlink-install --packages-select swarm_naive
source install/setup.bash
```

`--symlink-install` means an edit to the launch file takes effect on the next
launch. An edit to `waypoint_drone.cpp` needs a rebuild.

### Run in the simulator

Three terminals. Each terminal needs `source scripts/native/env.sh` first,
unless it is in your `~/.bashrc`.

Do not run `drone-sim`. The launch file starts its own PX4 instances and
Gazebo. If `drone-sim` is already running, stop it first, or the swarm spawns
into its world.

```bash
MicroXRCEAgent udp4 -p 8888              # 1. PX4 to ROS 2 bridge. Start it first
```

```bash
drone-gcs --expect 3                     # 2. ground station link for every drone
```

```bash
ros2 launch swarm_naive swarm.launch.py  # 3. PX4 instances, Gazebo, and the nodes
```

`drone-gcs --expect 3` prints one line per drone as each links up, then
`Every vehicle accounted for.` A drone it never names cannot arm.

What happens after launch, with default arguments:

| Time after launch | Event |
|---|---|
| 0 s | PX4 instance 1 starts and loads Gazebo |
| 30 s | Instance 2 starts and spawns into the world |
| 38 s | Instance 3 starts |
| 51 s | The three nodes start |

In a headless verification run for this README, the flight from the first drone
arming to the last drone landing took 41 s.

Output from that run, with the PX4 console lines removed:

```
[INFO] [launch.user]: swarm_naive: 3 drones, 8.0 m ring centred on north 13.0 east 5.0, altitudes 4.0 m to 8.0 m. SITL instances 1-3, so topics are under /px4_1/fmu/ upward. MicroXRCEAgent and drone-gcs must already be running.
[INFO] [drone_0]: slot 0 of 3. Pad at north 0.0 east 0.0, flying at 4.0 m, 6 waypoints round a 8.0 m ring. Commanding /px4_1/fmu/ as system 2.
[INFO] [drone_1]: slot 1 of 3. Pad at north 0.0 east 5.0, flying at 6.0 m, 6 waypoints round a 8.0 m ring. Commanding /px4_2/fmu/ as system 3.
[INFO] [drone_2]: slot 2 of 3. Pad at north 0.0 east 10.0, flying at 8.0 m, 6 waypoints round a 8.0 m ring. Commanding /px4_3/fmu/ as system 4.
[INFO] [drone_0]: armed.
[INFO] [drone_0]: offboard engaged, climbing to 4.0 m.
[INFO] [drone_0]: waypoint 1 of 5: swarm frame north 21.0 east 5.0 alt 4.0
[INFO] [drone_0]: waypoint 2 of 5: swarm frame north 9.0 east 11.9 alt 4.0
[INFO] [drone_0]: waypoint 3 of 5: swarm frame north 9.0 east -1.9 alt 4.0
[INFO] [drone_0]: waypoint 4 of 5: swarm frame north 21.0 east 5.0 alt 4.0
[INFO] [drone_0]: waypoint 5 of 5: swarm frame north 0.0 east 0.0 alt 4.0
[INFO] [drone_0]: route complete over its own pad, landing.
[INFO] [drone_0]: landed and disarmed. Flight complete.
```

Drones 1 and 2 print the same sequence, interleaved, starting from their own
vertex. Waypoint 0 is the climb and has no line of its own.

Normal lines that look like problems:

| Line | Why it is normal |
|---|---|
| One `No telemetry from the flight controller yet` per drone at startup | The first control tick runs before PX4's first message arrives |
| `[px4_1-1] [Err] ... Failed to load system plugin [libGstCameraSystem.so]` | Gazebo's camera streaming plugin is not installed. Nothing here uses a camera |

The nodes keep running after `Flight complete`. Ctrl-C in terminal 3 stops
every PX4 instance and node, and asks Gazebo to shut down.

`--show-args` prints every argument with its description, without launching:

```bash
ros2 launch swarm_naive swarm.launch.py --show-args
```

### Launch arguments

| Argument | Default | Effect |
|---|---|---|
| `count` | 3 | Number of drones, and of PX4 processes. 1 to 10 |
| `pad_spacing_m` | 5.0 | Gap between pads along east |
| `radius_m` | 8.0 | Radius of the ring |
| `altitude_m` | 4.0 | Altitude of drone 0 |
| `layer_m` | 2.0 | Extra altitude per slot. Drone k flies at `altitude_m + k * layer_m` |
| `laps` | 1 | Laps of the ring |
| `centre_north_m` | `radius_m + pad_spacing_m` | Ring centre, north, shared frame |
| `centre_east_m` | Middle of the pad row | Ring centre, east, shared frame |
| `headless` | false | Skip Gazebo rendering. Use it above about four drones, or with no GPU |
| `world` | default | Gazebo world by file name. See [docs/WORLDS.md](../../docs/WORLDS.md) |
| `px4_dir` | `$PX4_DIR` or `~/PX4-Autopilot` | PX4 checkout to run |
| `gazebo_boot_s` | 30.0 | Head start for the first vehicle, which loads Gazebo |
| `stagger_s` | 8.0 | Gap between the other vehicles |
| `px4_output` | log | `log` sends PX4's console to `~/.ros/log/<run>/launch.log`. `screen` prints it in the terminal |

### Things to try

| Command | What it shows |
|---|---|
| `ros2 launch swarm_naive swarm.launch.py layer_m:=0.0` | Every drone on one level. Nothing keeps them apart |
| `ros2 launch swarm_naive swarm.launch.py count:=5 radius_m:=12.0 headless:=true` | A bigger ring. Start `drone-gcs --expect 5` |
| `ros2 launch swarm_naive swarm.launch.py laps:=3` | Drift over several laps. Nothing corrects the spacing |
| Kill `drone-gcs` mid-flight | Every vehicle's datalink-loss failsafe fires, and each node reports `PX4 left offboard mode` |
| `ros2 launch swarm_naive swarm.launch.py px4_output:=screen` | PX4's own console, for a vehicle that will not boot |

### Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Drone 0 flies, the others never arm | `drone-gcs` is not serving every vehicle | `drone-gcs --expect 3` names the ones it has not linked |
| `Timed out waiting for Gazebo world` | A later vehicle started before the first finished loading Gazebo | Raise `gazebo_boot_s` |
| New drones appear in a field that already has drones in it | A Gazebo server survived the last run, or `drone-sim` is running | `pkill -f 'gz sim'` |
| `No telemetry from the flight controller yet` repeating | Agent started after PX4 | Start the agent, then relaunch |
| A drone flies to the wrong part of the ring | Its pad parameters disagree with where it spawned | Both come from the launch file. The spawn pose is `east,north` |
| The formation is mirrored | North and east swapped in the spawn pose | Gazebo is ENU, PX4 is NED |
| `count must be between 1 and 10` | `count` outside 1 to 10 | See section 4, "Why ten" |
| Launch file not found | Package not built, or overlay not sourced | Build, then `source install/setup.bash` |
| Everything runs slowly | Too many vehicles for the machine | `headless:=true` or fewer drones. PX4 and Gazebo run in lockstep, so the flight is unchanged |

The full table is in [docs/SWARM.md](../../docs/SWARM.md).

## 4. How it works

### Files

| File | What it is |
|---|---|
| `include/swarm_naive/waypoint_drone.hpp` | The declarations: the tunables, the QoS helpers, the `SwarmPoint`/`LocalPoint`/`Phase` types, and the `WaypointDrone` class |
| `src/waypoint_drone.cpp` | The method bodies |
| `src/main.cpp` | `main()`, and nothing else |
| `launch/swarm.launch.py` | Starts every PX4 instance and every node, and decides each drone's identity and pad |
| `CMakeLists.txt` | Builds `waypoint_drone`, and installs `launch/` into `share/swarm_naive` so `ros2 launch` can find it |
| `package.xml` | Dependencies. `launch` and `launch_ros` are runtime-only, for the launch file |

### One PX4 process per drone

Every airframe carries its own autopilot and its own companion computer, so the
launch file runs `px4 -i <instance>` once per drone and one `waypoint_drone` per
drone. PX4 derives everything that differs between vehicles from the instance
number:

| From `-i N` | Value | Where PX4 sets it (v1.16.1) |
|---|---|---|
| Topic namespace | `/px4_N/fmu/...` | `rcS` |
| MAVLink system id | N + 1 | `rcS`, `param set MAV_SYS_ID $((px4_instance+1))` |
| GCS MAVLink port | 18570 + N | `px4-rc.mavlink` |
| Gazebo model name | `x500_N` | `px4-rc.gzsim` |

The launch file numbers instances from 1. Instance 0 publishes on bare
`/fmu/...` with no namespace, which would make drone 0 a special case.
Starting at 1 gives every drone the same shape.

Each node gets `px4_ns` and `system_id` from the launch file. A wrong `px4_ns`
means no telemetry. A wrong `system_id` means PX4 drops every command, so the
node runs and nothing arms.

### The shared frame

Every PX4 measures position from its own pad. Three vehicles spawned 6 m apart
each report a local position within 3 cm of zero. "North 8" is therefore a
different place for every drone.

The route is written in a **shared swarm frame**, whose origin is drone 0's pad.
No autopilot knows about this frame. Each node converts every point into its own
PX4 frame just before publishing it:

```cpp
LocalPoint to_local(const SwarmPoint & point) const
{
  return LocalPoint{
    point.north_m - home_.north_m,
    point.east_m - home_.east_m,
    point.down_m};
}
```

`home_` is the drone's pad in the shared frame, passed in as `home_north_m` and
`home_east_m`. Altitude passes through unchanged because every pad is on the
same flat ground.

`SwarmPoint` and `LocalPoint` hold the same three numbers and are separate types.
`publish_setpoint()` takes a `LocalPoint` and `build_route()` returns
`SwarmPoint`s, so skipping the conversion does not compile.

### Spawn poses and the ENU swap

The launch file tells Gazebo where to spawn each drone with
`PX4_GZ_MODEL_POSE`, which is in Gazebo's world frame. Gazebo is ENU: x is east,
y is north, z is up. PX4 is NED. So the launch file writes the pose as
`f'{pad_east_m},{pad_north_m}'`.

Verified in the simulator: `"6,0"` spawned a vehicle 5.98 m east of instance 1,
and `"0,6"` spawned one 5.99 m north. PX4's conversion is in
`src/modules/simulation/gz_bridge/GZBridge.cpp` at `position ENU -> NED`.

Swapping these mirrors the whole formation.

The same two numbers go to Gazebo as the spawn point and to the node as its pad.
The launch file is the only place that knows both, so they cannot disagree.

### The route

`build_route()` runs once in the constructor and produces the whole flight as a
list of `SwarmPoint`s:

| Index | Point |
|---|---|
| 0 | Above the drone's own pad, at its altitude |
| 1 to `count * laps + 1` | Ring vertices `slot`, `slot + 1`, ... `slot + count * laps` |
| Last | Above the drone's own pad again |

`ring_vertex(i)` puts vertex i at angle `2 * pi * i / count` measured clockwise
from north, seen from above: `north = centre + r cos`, `east = centre + r sin`.
Indices past `count` wrap round because cosine and sine are periodic, so
`build_route()` can count upwards without a modulo.

Visiting `count * laps + 1` vertices ends the lap at the vertex where the drone
joined, so every drone flies the same distance.

The climb is vertical so no drone crosses another drone's pad on the way out.

### Altitude layering

```cpp
double station_altitude_m() const
{
  return altitude_m_ + static_cast<double>(slot_) * layer_m_;
}
```

Drone k flies `k * layer_m` above drone 0 from the end of its climb until
PX4 takes over to land. Because every drone is on its own level, paths can cross
without collision. This is the only separation in the package. It is fixed
before takeoff and reacts to nothing. `layer_m:=0.0` removes it.

### The control loop

`run_control_tick()` runs at 10 Hz. It publishes `OffboardControlMode` and the
current target through `to_local()` first, then calls `advance_phase()`. This is
the order from `hop`, for the same reason: PX4 drops offboard if the stream
pauses.

| Phase | What happens | Moves on when |
|---|---|---|
| `Priming` | Streams the setpoint above the pad | 10 ticks |
| `Arming` | Arm request every 0.5 s | PX4 reports armed. Gives up after 120 s |
| `EngagingOffboard` | Offboard request every tick | PX4 reports offboard |
| `Flying` | Targets `route_[route_index_]`. Advances the index on arrival | The index runs off the end. Sends `NAV_LAND` |
| `Landing` | PX4's land mode descends and disarms | PX4 reports disarmed |
| `Done` | Timer cancelled | Nothing. The process stays up until Ctrl-C |

`hop` needs a phase per leg. Here the flight is a list and an index, so one
`Flying` phase covers all of it. `run_flying()` is nine lines.

### Yaw

`update_yaw_setpoint()` points the nose at the current target so the swarm is
readable on screen and so a forward-facing camera would point the right way.

- `atan2(east, north)` because NED yaw is measured from north towards east.
- Inside `YAW_DEADBAND_M` (1 m) of the target, the last heading is held. The
  bearing to a point the drone is sitting on is noise, and chasing it makes the
  drone spin.
- Before flight, the yaw target is the drone's current heading, so it does not
  turn on the pad.

### Tunables

| Constant | Value | Reason |
|---|---|---|
| `CONTROL_LOOP_HZ` | 10 | PX4's examples use 10 Hz. Anything over about 5 Hz keeps offboard alive |
| `ARMING_TIMEOUT_S` | 120 | Twice `hop`'s. Several PX4 instances on one machine take longer to settle and link up |
| `ARMING_QUIET_PERIOD_S` | 5 | A healthy vehicle arms inside this, so only a stuck one prints anything |
| `ARRIVAL_RADIUS_M` | 0.7 | Larger than `hop`'s 0.5 m so the drone rounds each vertex instead of stopping on it |
| `LEG_STALL_WARNING_S` | 30 | A leg that takes this long usually means PX4 has left offboard |
| `YAW_DEADBAND_M` | 1.0 | See yaw, above |
| `MAX_SWARM_SIZE` | 10 | See below |

### Why ten

`drone-gcs` heartbeats instance 0 and instances 1 through 10. The launch file
numbers vehicles `slot + 1` for slot 0..count-1, so ten drones fill the range.
An eleventh would have no ground station link and could never arm. The launch
file and the node both refuse `count` above 10.

### The launch file

| Piece | What it does |
|---|---|
| `_arguments()` | Declares every launch argument with a default and a description |
| `OpaqueFunction(_launch_swarm)` | `count` is a launch argument, which has no value until launch runs. `perform(context)` turns it into a Python int that can drive a loop |
| Validation | Refuses a bad `count` or a missing PX4 binary before starting anything |
| `os.environ.pop('PX4_GZ_STANDALONE')` | PX4 treats this variable as present or absent, so even `=0` makes every vehicle wait for a Gazebo nobody started |
| `ExecuteProcess([px4, '-i', N, '-d'])` | One PX4 per drone. `-d` skips the interactive `pxh>` shell, which has no terminal under launch |
| `PX4_SYS_AUTOSTART=4001`, `PX4_SIM_MODEL=gz_x500` | The x500 airframe. Its airframe file also sets `NAV_DLL_ACT 2`, which is why `drone-gcs` is required |
| `HEADLESS=1` | Set on every vehicle when `headless:=true`. PX4 also reads it as present or absent |
| `TimerAction` on each PX4 | Instance 1 at 0 s. The rest at `gazebo_boot_s + (slot - 1) * stagger_s` |
| `TimerAction` on each node | `gazebo_boot_s + (count - 1) * stagger_s + 5`. The nodes wait for telemetry anyway, so this only keeps the output readable |
| `Node(..., parameters=[...])` | Identity (`px4_ns`, `system_id`, `slot`, `count`), pad, and the shared plan. The plan is identical for every drone |
| `OnShutdown` handler `_stop_gazebo` | Gazebo is started by the first PX4 as a detached process and outlives it. This asks the server to stop over `gz service` |

The shutdown handler is an `OpaqueFunction` running `subprocess.run`. An
`ExecuteProcess` in an `OnShutdown` handler never runs, because launch is
already tearing down and starts nothing new. It does not log that it skipped it.

The agent and `drone-gcs` are left out of the launch file. The agent has to be
running before any PX4 starts, and both are shared infrastructure that outlives
a single flight.

### What this package deliberately does not do

| Missing | Consequence |
|---|---|
| Collision avoidance | Nothing notices two aircraft converging. Altitude layering is the only protection |
| A shared clock | Each drone starts its lap when it finishes arming. A drone that arms 20 s late is permanently in the wrong part of the ring |
| Formation keeping | Spacing holds only while the drones fly at the same speed. Nothing corrects drift |

[`swarm_comms`](../swarm_comms/README.md) fixes the second and third, and
measures the first.

### Design decisions

| Decision | Reason |
|---|---|
| One node binary for every drone, differing only by parameters | The slot is the only thing that makes one drone behave differently. The same holds on real aircraft |
| Pad offset passed as a parameter | Simple and exact. On real aircraft this number comes from a survey or an RTK fix |
| The launch file owns both the spawn pose and the node's pad | One source of truth, so they cannot drift apart |
| Separate `SwarmPoint` and `LocalPoint` types | A frame mix-up fails to compile |
| Route built once in the constructor | The plan is fixed and printable. `announce_plan()` logs it |
| PX4 code copied from `hop` | The file reads start to finish on its own |
| Constructor validates every parameter and throws | A drone with a nonsense plan never takes off |
| Landing handed to PX4 with `NAV_LAND` | Same reason as `hop`: ground detection and disarming, as on the real aircraft |

### Where to go next

| To | Read |
|---|---|
| The same flight with the drones talking | [`../swarm_comms`](../swarm_comms/README.md) |
| Both swarm packages side by side | [docs/SWARM.md](../../docs/SWARM.md) |
| What the launch file replaces, by hand | "More than one drone" in [QUICKSTART.md](../../QUICKSTART.md) |
| The PX4 binary without any scripts | [docs/UNDER-THE-SCRIPTS.md](../../docs/UNDER-THE-SCRIPTS.md) |
