# Flying more than one drone

Two C++ packages, both worked examples.

| Package | Drones do | Know about each other |
|---|---|---|
| `swarm_naive` | Each flies a fixed waypoint route round a shared ring | Nothing |
| `swarm_comms` | Slot 0 flies the ring, rest chase the drone ahead | Position and state, 10 Hz, `/swarm/state` |

Read [HOW-IT-WORKS.md](../simulation/HOW-IT-WORKS.md) and [`src/cpp_single_hop/src/hop.cpp`](../../src/cpp_single_hop/src/hop.cpp) first. One drone taking off, flying forward and landing is the whole PX4 conversation. Neither package changes it.

Read `swarm_naive`, then `swarm_comms`. The second repeats the first's PX4 handshake verbatim. Strip comments and normalize names: 428 of `waypoint_drone.cpp`'s 450 lines appear unchanged in `formation_drone.cpp`.

Everything communication adds is in six places in `formation_drone.cpp`:

| Function | Purpose |
|---|---|
| `swarm_qos()` | QoS for drone-to-drone telemetry |
| `publish_swarm_state()` and `on_swarm_state()` | Sending position, tracking others |
| `to_swarm()` | Converting out of local frame |
| `run_waiting_for_swarm()` | The barrier (shared clock) |
| `chase_target()` and `run_follower()` | Formation, and behavior on message loss |
| `check_separation()` | Measurement `swarm_naive` cannot make |

## Run it

Three terminals:

```bash
MicroXRCEAgent udp4 -p 8888              # 1. bridge
drone-gcs                                # 2. GCS link
ros2 launch swarm_naive swarm.launch.py  # 3. PX4 instances and flight nodes
```

The launch file starts every PX4 instance and every node. It does not start the agent or `drone-gcs`. The agent must be running before any PX4 starts.

Terminal 2 output while booting:

```
Linked to PX4 system 2. Datalink failsafe satisfied for 1 vehicle(s); they can arm.
Linked to PX4 system 3. Datalink failsafe satisfied for 2 vehicle(s); they can arm.
Linked to PX4 system 4. Datalink failsafe satisfied for 3 vehicle(s); they can arm.
```

One line per drone. Systems are 2 to `count + 1` (SITL instance N answers to system id N + 1, launch files number from 1).

```bash
drone-gcs --expect 3
```

```
Expecting 3 vehicle(s). SITL instance N answers to system id N+1, so instances 1-3 are systems 2-4.
Linked to PX4 system 2. 1 of 3 vehicles linked.
Linked to PX4 system 3. 2 of 3 vehicles linked.
Linked to PX4 system 4. 3 of 3 vehicles linked.
Every vehicle accounted for.
```

A count that stops short names vehicles that will sit reporting `Arming denied`. Reports lost and regained links at PX4's `COM_DL_LOSS_T` threshold (10 s):

```
Lost the link to PX4 system 3 after 10 s of silence. Its datalink-loss failsafe
will be firing now. 2 of 3 vehicles linked.
PX4 system 3 is back. 3 of 3 vehicles linked.
```

Flight takes ~3 minutes from launch, mostly Gazebo and PX4 boot.

```bash
ros2 launch swarm_naive swarm.launch.py count:=5 radius_m:=12.0
ros2 launch swarm_naive swarm.launch.py headless:=true count:=6
ros2 launch swarm_comms swarm.launch.py follow_distance_m:=6.0
ros2 launch swarm_naive swarm.launch.py --show-args
```

Use `headless:=true` above ~4 drones or without a GPU. Skips rendering. PX4 and Gazebo run in lockstep; costs wall-clock time and GUI framerate, not flight behavior.

### Stopping

`Ctrl-C` in terminal 3 stops PX4 and nodes. Gazebo is started by the first PX4 as a detached background process. The launch file asks it to stop on shutdown. If one survives:

```bash
pkill -f 'gz sim'
```

A leftover Gazebo: next launch finds a world already running, joins it, and you get a field with six drones where half do not respond.

## The flight

Pads are a row along east, 5 m apart, at north 0. Drone 0's pad is the shared swarm frame origin. Ring centre is north of the row at `centre_north_m` (`radius_m + pad_spacing_m` default).

```
        north
          ^         v1 . - - - . v0          ring, one vertex per drone,
          |            \       /             radius_m across
          |             \     /
          |              ` v2 '
          |
          |      drone0    drone1    drone2
          +--------o---------o---------o--------> east
                 (0,0)     (0,5)     (0,10)
```

| | `swarm_naive` | `swarm_comms` |
|---|---|---|
| Climb | Straight up off pad | Same |
| Altitude | `altitude_m + slot * layer_m` | Same |
| Start | When that drone finishes arming | When every drone reports ready |
| Pattern | Every drone flies ring, entering at vertex `slot` | Slot 0 flies ring, slot k chases slot k-1 at `follow_distance_m` |
| Finish | Each lands when its route runs out | Slot 0 says `done`, each peels off when the one ahead does |

Both end with `VEHICLE_CMD_NAV_LAND` over the drone's own pad. PX4's landing handles ground detection and disarming.

## What a swarm needs that one drone does not

### One PX4 process per drone

Every airframe carries its own autopilot and companion computer. The launch file runs `px4 -i <instance>` once per vehicle.

`-i` is PX4's multi-vehicle support. Derived from it, v1.16.1:

| Derived from `-i N` | Value | Source |
|---|---|---|
| Topic namespace | `/px4_N/fmu/...` | `rcS` line 299 |
| MAVLink system id | `N + 1` | `rcS` line 140 |
| GCS MAVLink port | `18570 + N` | `px4-rc.mavlink` |
| Gazebo model name | `x500_N` | `px4-rc.gzsim` |

Instance 0 publishes on bare `/fmu/...` (like a real vehicle). Launch files number from 1.

Wrong system id: PX4 drops `VehicleCommand` addressed to a different system. Node runs, nothing arms.

### A frame more than one drone can use

Every PX4 measures position from its own takeoff point. Three vehicles 6 m apart each report `vehicle_local_position` within 3 cm of zero.

Both packages solve it the same way: the plan is in a shared swarm frame (origin: drone 0's pad). Each node converts:

```cpp
LocalPoint to_local(const SwarmPoint & point) const
{
  return LocalPoint{
    point.north_m - home_.north_m,
    point.east_m - home_.east_m,
    point.down_m};
}
```

Two subtractions of the home offset. `SwarmPoint` and `LocalPoint` are separate types: wrong-frame usage is a compile error. `down_m` passes through (all pads on flat ground).

| | How the node learns its pad position |
|---|---|
| Here | Launch parameter. Same launch file spawns the vehicle and tells the node |
| Real aircraft | Survey or RTK fix. Plain GPS: metres. RTK: centimetres |

### Spawn poses are ENU

`PX4_GZ_MODEL_POSE` is `"x,y,z,roll,pitch,yaw"` in Gazebo's world frame (ENU: x east, y north, z up). PX4's frame is NED. East and north are swapped relative to flight code.

Verified:

| Spawned with | Measured offset from instance 1 |
|---|---|
| `PX4_GZ_MODEL_POSE="6,0"` | north +0.02 m, east +5.98 m |
| `PX4_GZ_MODEL_POSE="0,6"` | north +5.99 m, east -0.03 m |

Conversion in PX4 v1.16.1: `src/modules/simulation/gz_bridge/GZBridge.cpp`, comment `position ENU -> NED`. Swapped east/north mirrors the formation.

### Staggered startup

First vehicle loads Gazebo and the world. Others spawn into an existing world. Launching all at once: later ones look for a world not yet there.

| Argument | Default | Meaning |
|---|---|---|
| `gazebo_boot_s` | 30.0 | Head start for first vehicle |
| `stagger_s` | 8.0 | Gap between rest |

PX4's startup script waits up to 30 s for the world. Raise `gazebo_boot_s` on slow machines or heavy worlds.

## swarm_naive

Files: [`src/swarm_naive/`](../../src/swarm_naive/)

| Read this | To understand |
|---|---|
| Header comment in `waypoint_drone.cpp` | The package and three things it does not do |
| `SwarmPoint` and `LocalPoint` | Two frames, why two types |
| `build_route()` | Flight as a list of points in shared frame |
| `to_local()` | The conversion |
| `station_altitude_m()` | Only deconfliction |
| `run_flying()` | Nine lines |
| `launch/swarm.launch.py` | Instance numbering, spawn poses, staggering, parameters |

Every drone gets a slot number and flight plan at launch, flies open loop. Three consequences:

- **No collision avoidance.** Drone k flies at `altitude_m + k * layer_m`. Static altitude layering is the only separation.
- **No shared clock.** Each drone starts when it finishes arming. A 20-second delay is permanent.
- **No formation.** Spacing holds only if drones happen to fly the same speed.

All drones on one level:

```bash
ros2 launch swarm_naive swarm.launch.py layer_m:=0.0
```

## swarm_comms

Files: [`src/swarm_comms/`](../../src/swarm_comms/)

| Read this | To understand |
|---|---|
| `msg/SwarmState.msg` | Interface between drones, why every field is a level |
| `swarm_qos()` | BEST_EFFORT, depth as function of swarm size |
| `to_swarm()` and `to_local()` | Exact inverses |
| `run_waiting_for_swarm()` | Shared clock without a clock |
| `chase_target()` | Four lines that make it a swarm |
| `run_follower()` | Behavior when messages stop |
| `check_separation()` | Measurement `swarm_naive` cannot make |
| `CMakeLists.txt` | Generating and using a message from same package |

### The message

One topic: `/swarm/state`. Every drone publishes and subscribes. No per-drone topics, no point-to-point addressing. Matches radio link behavior.

Outside any `px4_` namespace. On real aircraft every autopilot occupies bare `/fmu/...`. The `slot` field identifies the sender. Code does not change when leaving the simulator.

Every field is a level, never an event. "I am ready" at 10 Hz, not "I became ready" once. A receiver that misses a message learns the same thing 100 ms later. BEST_EFFORT: a retransmitted position half a second old is stale. Were `ready` an announcement, one dropped packet deadlocks the swarm.

| QoS | `/fmu/` topics | `/swarm/state` |
|---|---|---|
| Reliability | BEST_EFFORT (PX4) | BEST_EFFORT (chosen) |
| Durability | TRANSIENT_LOCAL (PX4) | VOLATILE |
| Depth | 1 | `2 * count` |

Depth: every drone publishes to this one topic. `count` messages can arrive between two control loop ticks. Depth 1 means hearing from one drone per tick.

`/fmu/in/vehicle_command` exception: PX4 serves it as RELIABLE DDS replier. Needs RELIABLE publisher.

### Shared clock, solved without a clock

Nobody moves until everybody reports ready. Converts "start at the same time" (needs synchronized clocks) into "start at the same event" (needs one bit per drone).

```
[INFO] [drone_1]: at station, waiting for the rest of the swarm.
[INFO] [drone_1]: holding at station, waiting for slot 2 (never heard from) (5 s of 180).
[INFO] [drone_1]: whole swarm is ready. Following slot 0.
```

Swarm moves at speed of slowest member. `BARRIER_TIMEOUT_S` (180 s) ends the wait. Warning names missing slots.

Message age is measured by arrival time on the receiver's clock. Comparing sender's `stamp_us` against receiver's measures clock skew and age, indistinguishable.

### The rule that makes it a swarm

Sit `follow_distance_m` behind the drone ahead, on the line from you to it:

```cpp
return SwarmPoint{
  ahead.north_m - follow_distance_m_ * north_gap / range_m,
  ahead.east_m  - follow_distance_m_ * east_gap  / range_m,
  -station_altitude_m()};
```

One vector subtraction, one normalization. The chain arranges itself: whatever the leader does, each follower converges on a point a fixed distance back along its line of sight. Formation bends round corners. Closer than `follow_distance_m`: target lands behind follower, it backs off. Stops chain concertinaing.

Target cannot be computed without a message from another aircraft. Recomputed 10 Hz from live data. Horizontal only: each drone stays on its altitude layer.

### When messages stop

| Silence from drone ahead | Follower behavior |
|---|---|
| < 1 s (`LINK_STALE_S`) | Nothing. Ten missed messages normal on lossy link |
| 1-15 s | Holds last target, warns. Keeps streaming setpoints (pausing drops offboard) |
| > 15 s (`LINK_LOST_ABORT_S`) | Abandons formation, returns home |

Stale position data puts two aircraft in the same place. Last received position stays numerically plausible after it stops being true.

### Separation is measured, not avoided

`check_separation()` reports distance to nearest other aircraft, warns below `SEPARATION_WARNING_M`. Changes no setpoint.

```
[INFO] [drone_2]: closest neighbour is slot 1 at 5.1 m.
```

On the default three-drone triangle, the warning fires once as the chain rounds a corner:

```
[WARN] [drone_1]: slot 2 is 2.5 m away, inside the 2.5 m we would like.
                  Nothing in this node will do anything about that.
```

The pursuit rule: a follower cuts the corner its leader turns. Chain compresses on every turn. 5 m follow distance closes to ~2.5 m. Raise `radius_m` or lower `follow_distance_m` to stop it.

The chase rule constrains each follower against the drone ahead. Nothing stops slot 3 from flying through slot 1. Altitude layering (from `swarm_naive`) keeps vertical separation.

## Parameters

`--show-args` prints all with descriptions.

| Argument | Default | Effect |
|---|---|---|
| `count` | 3 | Drones and PX4 processes. Max 10 |
| `pad_spacing_m` | 5.0 | Gap between pads along east |
| `radius_m` | 8.0 | Ring radius |
| `altitude_m` | 4.0 | Altitude of drone 0 |
| `layer_m` | 2.0 | Vertical gap between slots |
| `laps` | 1 | Laps of the ring |
| `centre_north_m` | `radius_m + pad_spacing_m` | Ring centre, shared frame |
| `centre_east_m` | middle of pad row | Ring centre, shared frame |
| `headless` | false | Skip Gazebo rendering |
| `world` | default | Gazebo world, see [WORLDS.md](../simulation/WORLDS.md) |
| `px4_dir` | `$PX4_DIR` or `~/PX4-Autopilot` | PX4 checkout |
| `gazebo_boot_s` | 30.0 | Head start for first vehicle |
| `stagger_s` | 8.0 | Gap between rest |
| `px4_output` | log | PX4 console output. `log`: `~/.ros/log/<run>/launch.log`. `screen`: terminal |

`swarm_comms` adds:

| Argument | Default | Effect |
|---|---|---|
| `follow_distance_m` | 5.0 | Distance behind drone ahead |

Ten drones max: `drone-gcs` heartbeats instances 0-9. An eleventh could never satisfy its datalink failsafe. Both nodes refuse to construct above it.

## Things to try

| Command | Shows |
|---|---|
| `ros2 launch swarm_naive swarm.launch.py layer_m:=0.0` | No altitude layering |
| `ros2 launch swarm_comms swarm.launch.py follow_distance_m:=2.0` | Separation warning firing |
| Kill `drone-gcs` mid-flight | Every vehicle's datalink-loss failsafe |
| Kill one node mid-flight in `swarm_comms` | Follower holding, then going home |
| `ros2 topic echo /swarm/state` | Whole swarm conversation |
| `ros2 topic hz /swarm/state` | Rate is `10 * count` |
| `ros2 launch swarm_naive swarm.launch.py world:=lawn` | Scenery |
| `ros2 launch swarm_comms swarm.launch.py count:=6 headless:=true laps:=2` | Six drones, two laps, no rendering |

## Problems

| Symptom | Cause | Fix |
|---|---|---|
| Drone 0 flies, rest never arm | `drone-gcs` serving one vehicle | Should name systems 2 to `count + 1` |
| `drone-gcs` says ignoring `UNKNOWN_<n>` messages | PX4 streams messages newer than pymavlink's dialect | Normal. Does not count them as vehicles |
| `ERROR [init] Timed out waiting for Gazebo world` | Later vehicle started before first finished loading | Raise `gazebo_boot_s` |
| Drones spawn into field with existing drones | Gazebo server survived | `pkill -f 'gz sim'` |
| `No telemetry from the flight controller yet`, forever | Agent started after PX4, or wrong `px4_ns` | Start agent first, relaunch |
| Node runs, nothing arms | Wrong `system_id` | Instance N needs system id N + 1 |
| Drone flies to wrong part of ring | `home_north_m`/`home_east_m` disagree with spawn | Both from launch file. Check `PX4_GZ_MODEL_POSE` is `east,north` |
| Formation mirrors itself | East/north swapped in spawn pose | Gazebo ENU, PX4 NED |
| `swarm_comms` drones hold station forever | Barrier waiting on a slot | Warning names which slots. `ros2 topic echo /swarm/state` |
| `Heard slot N, but this swarm only has M drones` | Another swarm on same `ROS_DOMAIN_ID`, or leftover node | Different `ROS_DOMAIN_ID`. `env.sh` restricts to localhost |
| Everything at 0.2x real time | Too many vehicles | `headless:=true` or fewer drones |
| `No executable found` | Package built before executable added | `colcon build --symlink-install`, `source install/setup.bash` |
| Launch file not found | `install(DIRECTORY launch ...)` missing from `CMakeLists.txt` | Add it |

## What changes on real aircraft

| | Simulation | Field |
|---|---|---|
| Network | One machine, no loss | WiFi/radio mesh. Real latency, loss, partitions. Test separately |
| Agent | One `MicroXRCEAgent` for fleet | One per companion computer |
| Shared frame | Launch parameter | Survey or RTK fix. GPS: metres. RTK: centimetres |
| Position estimate | Near perfect | Limit on formation spacing |
| Bad setpoint | Costs a restart | Does not |

## Reference

- [HOW-IT-WORKS.md](../simulation/HOW-IT-WORKS.md)
- [`src/cpp_single_hop/src/hop.cpp`](../../src/cpp_single_hop/src/hop.cpp)
- [NEW-PACKAGE.md](../development/NEW-PACKAGE.md)
- [UNDER-THE-SCRIPTS.md](../simulation/UNDER-THE-SCRIPTS.md)
- [SIMULATION-FIDELITY.md](../simulation/SIMULATION-FIDELITY.md)
- [WORLDS.md](../simulation/WORLDS.md)
- [Multi-vehicle with ROS 2](https://docs.px4.io/v1.16/en/ros2/multi_vehicle)
- [Multi-vehicle in Gazebo](https://docs.px4.io/v1.16/en/sim_gazebo_gz/multi_vehicle_simulation.html)
- [px4_msgs release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg)
- [PX4 frame conventions](https://docs.px4.io/v1.16/en/contribute/notation.html)
- [ROS 2 launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 custom messages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2 QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)