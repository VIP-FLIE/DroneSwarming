# swarm_comms

The same drones, pads and ring as [`swarm_naive`](../swarm_naive/README.md),
with the drones telling each other where they are. Nobody moves until every
drone is ready. Then drone 0 leads round the ring and each other drone follows
the one ahead of it.

Read `swarm_naive` first. This package keeps its PX4 code, launch file and
shared frame, and adds communication on top.

## 1. What the drones do

Three drones by default, on pads in a row 5 m apart along east.

1. **Climb.** Each drone climbs straight up off its own pad to its own altitude:
   4 m, 6 m, 8 m.
2. **Wait.** Each drone holds there and broadcasts that it is ready. Nobody
   moves until every drone has reported ready.
3. **Fly.** Drone 0 flies once round the ring. Drone 1 follows 5 m behind
   drone 0. Drone 2 follows 5 m behind drone 1. The followers have no route. They
   chase a point computed ten times a second from the position of the drone
   ahead, so they cut the corners the leader turns.
4. **Go home.** When drone 0 finishes the ring it broadcasts `done` and returns
   over its pad. Drone 1 sees that and returns. Drone 2 sees drone 1's `done`
   and returns. Nobody sends a "land" order.
5. **Land.** Each drone lands on its own pad.

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
             leader   follows 0  follows 1
```

Coordinates are (north, east) in metres in the shared swarm frame, whose origin
is drone 0's pad. Only the leader flies to the ring vertices, in the order v0,
v1, v2, v0.

| | `swarm_naive` | `swarm_comms` |
|---|---|---|
| Start | Each drone starts when it finishes arming | Everyone starts when the last drone is ready |
| Drone 0 | Flies the ring from v0 | Same |
| Drones 1 and up | Fly the ring from their own vertex | Chase the drone ahead at `follow_distance_m` |
| Finish | Each lands when its own route runs out | Each goes home when the drone ahead says `done` |
| Knows where the others are | No | Yes, 10 Hz, over `/swarm/state` |

## 2. What you learn from it

| Concept | Where |
|---|---|
| Defining your own message type and using it in the same package | `msg/SwarmState.msg`, `CMakeLists.txt` |
| Designing a drone-to-drone broadcast topic | `SWARM_STATE_TOPIC`, `on_swarm_state()`, `publish_swarm_state()` |
| Messages that repeat a state every tick, so a lost one is harmless | `msg/SwarmState.msg` header |
| Choosing QoS for telemetry you own: best effort, volatile, depth scaled with swarm size | `swarm_qos()` |
| A readiness barrier as a replacement for a shared clock | `run_waiting_for_swarm()` |
| Measuring message age on the receiver's clock | `Neighbour::received`, `link_age_s()` |
| Converting both ways between the drone's frame and the shared frame | `to_local()`, `to_swarm()` |
| A leader-follower pursuit rule | `chase_target()` |
| Handling stale and lost links | `fresh_state()`, `run_follower()` |
| Measuring separation, and why measuring is not avoiding | `check_separation()` |
| A cascade that ends the flight in order with no coordinator | `done` field, `run_follower()` |

## 3. How to run it

### Build

From the repo root, the directory that contains `src/`:

```bash
colcon build --symlink-install --packages-select swarm_comms
source install/setup.bash
```

The build generates C++ code from `msg/SwarmState.msg` before compiling the
node. Check the message was generated:

```bash
ros2 interface show swarm_comms/msg/SwarmState
```

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
ros2 launch swarm_comms swarm.launch.py  # 3. PX4 instances, Gazebo, and the nodes
```

Startup timing is the same as `swarm_naive`: with the defaults, the nodes start
51 s after launch.

To watch the drones talk, in a fourth terminal:

```bash
ros2 topic echo /swarm/state     # every message from every drone
ros2 topic hz /swarm/state       # about 10 x count messages per second
```

Output from a headless verification run for this README, with the PX4 console
lines and the startup `No telemetry` warnings removed:

```
[INFO] [drone_0]: slot 0 of 3, LEADER. Pad at north 0.0 east 0.0, flying at 4.0 m, 4 waypoints round a 8.0 m ring. Commanding /px4_1/fmu/ as system 2.
[INFO] [drone_1]: slot 1 of 3, following slot 0 at 5.0 m. Pad at north 0.0 east 5.0, flying at 6.0 m. Commanding /px4_2/fmu/ as system 3.
[INFO] [drone_2]: slot 2 of 3, following slot 1 at 5.0 m. Pad at north 0.0 east 10.0, flying at 8.0 m. Commanding /px4_3/fmu/ as system 4.
[INFO] [drone_0]: armed.
[INFO] [drone_1]: armed.
[INFO] [drone_0]: offboard engaged, climbing to 4.0 m.
[INFO] [drone_1]: offboard engaged, climbing to 6.0 m.
[INFO] [drone_2]: armed.
[INFO] [drone_2]: offboard engaged, climbing to 8.0 m.
[INFO] [drone_0]: at station, waiting for the rest of the swarm.
[INFO] [drone_0]: holding at station, waiting for slot 1, slot 2 (0 s of 180).
[INFO] [drone_1]: at station, waiting for the rest of the swarm.
[INFO] [drone_1]: holding at station, waiting for slot 2 (0 s of 180).
[INFO] [drone_2]: at station, waiting for the rest of the swarm.
[INFO] [drone_2]: whole swarm is ready. Following slot 1.
[INFO] [drone_0]: whole swarm is ready. Leading off round the ring.
[INFO] [drone_1]: whole swarm is ready. Following slot 0.
[INFO] [drone_2]: closest neighbour is slot 1 at 5.2 m.
[INFO] [drone_0]: waypoint 1 of 3: swarm frame north 9.0 east 11.9 alt 4.0
[INFO] [drone_0]: waypoint 2 of 3: swarm frame north 9.0 east -1.9 alt 4.0
[INFO] [drone_0]: waypoint 3 of 3: swarm frame north 21.0 east 5.0 alt 4.0
[INFO] [drone_1]: closest neighbour is slot 2 at 2.9 m.
[INFO] [drone_0]: ring complete, heading home. Swarm can follow.
[INFO] [drone_1]: slot 0 is done, heading home.
[INFO] [drone_2]: slot 1 is done, heading home.
[INFO] [drone_0]: over its own pad, landing.
[INFO] [drone_2]: over its own pad, landing.
[INFO] [drone_1]: over its own pad, landing.
[INFO] [drone_0]: landed and disarmed. Flight complete.
[INFO] [drone_1]: landed and disarmed. Flight complete.
[INFO] [drone_2]: landed and disarmed. Flight complete.
```

Some `closest neighbour` lines are trimmed. They print every 5 s per drone.

The leader's first target, v0, has no line of its own. `waypoint 1 of 3`
prints when it reaches v0 and turns for v1.

The same normal-but-alarming lines as `swarm_naive` apply: one
`No telemetry from the flight controller yet` per drone at startup, and a
`libGstCameraSystem.so` plugin error from Gazebo.

The nodes keep running after `Flight complete`. Ctrl-C in terminal 3 stops
everything and asks Gazebo to shut down.

### Launch arguments

Every argument from `swarm_naive` applies, with the same defaults. See
[its table](../swarm_naive/README.md#launch-arguments), or run:

```bash
ros2 launch swarm_comms swarm.launch.py --show-args
```

This package adds one:

| Argument | Default | Effect |
|---|---|---|
| `follow_distance_m` | 5.0 | How far behind the drone ahead each follower sits. Must be more than 0.5 |

`radius_m`, `laps` and the ring centre describe the leader's route only.
`layer_m` still separates every drone vertically.

### Things to try

| Command or action | What it shows |
|---|---|
| `ros2 launch swarm_comms swarm.launch.py follow_distance_m:=2.0` | A tight chain. The separation warning fires on the corners |
| `ros2 launch swarm_comms swarm.launch.py count:=5 headless:=true laps:=2` | A longer chain. Start `drone-gcs --expect 5` |
| `pkill -f '__node:=drone_1'` mid-flight, which stops drone 1's node | Drone 2 holds its last target, warns, then gives up after 15 s and goes home |
| `pkill -f 'bin/px4 -i 3'` before the swarm is ready | Drone 2 never reaches its station. The barrier holds the others, names slot 2, and lands them after 180 s |
| `ros2 topic echo /swarm/state` | The whole conversation between the drones |
| Kill `drone-gcs` mid-flight | Every vehicle's datalink-loss failsafe fires and PX4 takes control from the nodes |

### Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Every drone holds at station forever | The barrier is waiting on a slot | The `waiting for ...` line names the slot and says `never heard from` or `link stale` |
| `Gave up after 180 s waiting for ...` | A drone never reached its station | Usually it never armed. Check `drone-gcs --expect` |
| `Heard slot N, but this swarm only has M drones` | Another swarm on the same `ROS_DOMAIN_ID`, or a node left over from the last run | Kill stale nodes. Machines on one network need different `ROS_DOMAIN_ID`s |
| `slot N has gone quiet` | The drone ahead stopped broadcasting | Check that drone's node. After 15 s the follower goes home |
| Formation collapses onto one point | A position published without `to_swarm()` | Every broadcast position must be in the shared frame |
| Link errors: undefined reference to `SwarmState` type support | `rosidl_get_typesupport_target` missing from `CMakeLists.txt` | See the build section in section 4 |
| `ros2 interface show` cannot find the message | `<member_of_group>rosidl_interface_packages</member_of_group>` missing, or not rebuilt | Rebuild after fixing `package.xml` |

The `swarm_naive` troubleshooting table covers the PX4 and launch problems,
which are the same here.

## 4. How it works

### Files

| File | What it is |
|---|---|
| `include/swarm_comms/formation_drone.hpp` | The declarations: the tunables, the QoS helpers, the `SwarmPoint`/`LocalPoint`/`Phase` types, and the `FormationDrone` class |
| `src/formation_drone.cpp` | The method bodies |
| `src/main.cpp` | `main()`, and nothing else |
| `msg/SwarmState.msg` | The message every drone broadcasts |
| `launch/swarm.launch.py` | Same as `swarm_naive`'s, running `formation_drone` and passing `follow_distance_m` |
| `CMakeLists.txt` | Generates the message, builds the node, links the generated type support |
| `package.xml` | Adds the message generator and runtime dependencies |

The PX4 code is copied from `waypoint_drone.cpp` so this file reads on its own.
Everything communication adds is in these places:

| In `formation_drone.cpp` | What it is |
|---|---|
| `swarm_qos()` | QoS for drone-to-drone telemetry |
| `publish_swarm_state()`, `on_swarm_state()` | Broadcasting, and keeping the latest report from every other drone |
| `to_swarm()` | Converting this drone's position into the shared frame before broadcasting it |
| `run_waiting_for_swarm()` | The barrier |
| `chase_target()`, `run_follower()` | The formation rule, and what to do when the messages stop |
| `check_separation()` | Distance to the nearest drone |

### The message

`/swarm/state`, type `swarm_comms/msg/SwarmState`:

| Field | Type | Meaning |
|---|---|---|
| `stamp_us` | `uint64` | Sender's clock in microseconds. Logged, never compared |
| `slot` | `uint8` | Which drone sent it |
| `ready` | `bool` | "I am at my station." The barrier is built from this |
| `done` | `bool` | "I have finished and am going home." The drone behind watches for it |
| `north_m`, `east_m`, `down_m` | `float32` | Position in the **shared swarm frame**, NED |

Every field is a state, repeated ten times a second for as long as it is true.
No field announces a change. A receiver that misses a message gets the same
information 100 ms later. If `ready` were sent once as an announcement, one lost
packet would stop the whole swarm at the barrier.

The topic has no `px4_` namespace and there is one topic for all drones. On
real aircraft every autopilot publishes on bare `/fmu/...`, so namespaces cannot
tell drones apart. The `slot` field identifies the sender, which works the same
in simulation and in the field.

### QoS

| | `/fmu/` topics | `/swarm/state` |
|---|---|---|
| Chosen by | PX4 | This package |
| Reliability | Best effort | Best effort |
| Durability | Transient local | Volatile |
| Depth | 1 | 2 x `count` |

Best effort, because a retransmitted position arrives late, and a late position
is worse than a lost one: the formation would steer by where a drone used to
be. The state design makes loss harmless.

Volatile, because a report from before this node started is out of date.

Depth `2 * count`, because every drone publishes to this one topic. Up to
`count` messages can arrive between two control ticks. With depth 1 the node
would hear from one drone per tick and treat the others as stale.

### Two frames, both directions

`swarm_naive` only converts shared to local, because its plan is fixed. This
package converts both ways:

| Function | Direction | Used on |
|---|---|---|
| `to_local()` | Shared frame to this drone's PX4 frame. Subtract the pad | Every setpoint sent to PX4 |
| `to_swarm()` | This drone's PX4 frame to the shared frame. Add the pad | Every position broadcast to the swarm |

Without `to_swarm()`, every drone would broadcast a position near zero, because
every drone is near its own pad in its own frame. The followers would all chase
the same point. `SwarmPoint` and `LocalPoint` are separate types, so the
missing conversion does not compile.

### The flight, phase by phase

`run_control_tick()` runs at 10 Hz. In order, it publishes the PX4 stream,
publishes `/swarm/state`, then calls `advance_phase()`. Both publishes happen on
every tick: PX4 drops offboard if its stream pauses, and the other drones treat
a silent drone as failed.

| Phase | Target | Moves on when |
|---|---|---|
| `Priming` | Station: above its own pad at its own altitude | 10 ticks |
| `Arming` | Station | PX4 reports armed. Gives up after 120 s |
| `EngagingOffboard` | Station | PX4 reports offboard |
| `Climbing` | Station | Within 0.7 m. Sets `ready` |
| `WaitingForSwarm` | Station | Every other slot reports `ready` within the last 1 s. After 180 s, goes to `Returning` |
| `Flying` | Leader: next ring vertex. Follower: `chase_target()` | Leader: route ends. Follower: drone ahead says `done`, or 15 s of silence from it. Sets `done` |
| `Returning` | Station | Within 0.7 m. Sends `NAV_LAND` |
| `Landing` | None, PX4's land mode has control | PX4 reports disarmed |
| `Done` | None | Timer cancelled. The node stops publishing and stays up until Ctrl-C |

The climb and the trip home are phases here, where `swarm_naive` has them as
the first and last waypoints. A follower has no route to put them in.

### The barrier

`run_waiting_for_swarm()` holds the drone at its station until
`slots_not_ready()` is empty. A slot counts as ready only if its latest report
says `ready` and arrived less than `LINK_STALE_S` (1 s) ago.

This replaces a shared clock. Two drones cannot cheaply agree on the time over a
radio. They can agree on an event: "everyone is at station". Every drone
releases within a tick or two of the last drone's `ready` arriving.

| Cost | Answer in the code |
|---|---|
| The swarm starts at the speed of its slowest drone | Accepted |
| One drone that never becomes ready holds everyone | `BARRIER_TIMEOUT_S` (180 s), after which the drone returns and lands |
| A stuck barrier is hard to diagnose | The wait message names each missing slot and says `never heard from` or `link stale` |

A drone's own readiness comes from `at_station_` directly. `on_swarm_state()`
drops the drone's own messages, which it receives because it subscribes to the
topic it publishes on.

### Message age

`Neighbour::received` is the receiver's clock at arrival. The sender's
`stamp_us` is never compared with it. Two drones do not share a clock, so
comparing their stamps would mix clock offset into the age, with no way to
separate the two. Arrival time on the receiver's own clock needs no agreement.

### The leader

Slot 0 builds a route of `count * laps + 1` ring vertices starting at v0, and
flies it with the same arrive-and-advance logic as `swarm_naive`. When the index
runs off the end it sets `done_` and switches to `Returning`. `done_` is set
first, so the very next broadcast carries it.

### The follower rule

```cpp
return SwarmPoint{
  ahead.north_m - follow_distance_m_ * north_gap / range_m,
  ahead.east_m  - follow_distance_m_ * east_gap  / range_m,
  -station_altitude_m()};
```

`north_gap` and `east_gap` point from this drone to the drone ahead, and
`range_m` is their length. The target is the point `follow_distance_m` back
from the drone ahead, along the line to this drone.

| Situation | Result |
|---|---|
| Further than `follow_distance_m` behind | Target is ahead. The follower closes up |
| Closer than `follow_distance_m` | Target is behind. The follower backs off, so the chain does not bunch up when the leader slows |
| The leader turns | Each follower aims along its own line of sight, so it cuts the corner and the chain bends |
| Under 0.5 m horizontally (`MIN_CHASE_BASELINE_M`) | The bearing is noise. The follower holds its position |

The target is recomputed every tick from a message less than a second old.
That is the difference between this package and `swarm_naive`: the setpoint
cannot be computed without another drone's position.

The rule is horizontal. Each follower stays on its own altitude layer, so the
distance held is a ground distance and the vertical separation from `layer_m`
remains.

`follow_distance_m` must be more than 0.5 m. At or below that, the drone ahead
would sit inside the deadband and the formation would never converge. The node
refuses to start.

### When the messages stop

`run_follower()` checks the age of the latest report from the drone ahead:

| Age | What the follower does |
|---|---|
| Under 1 s (`LINK_STALE_S`) | Chases as normal |
| 1 s to 15 s | Holds its last target and warns every 5 s. It keeps streaming setpoints to PX4 |
| Over 15 s (`LINK_LOST_ABORT_S`) | Sets `done`, returns over its pad and lands |

Chasing a stale position is how two aircraft end up in the same place. Setting
`done` on abort releases the drone behind it too, so a break in the chain sends
everything behind the break home in order.

### Separation

`check_separation()` runs every tick while flying. It finds the nearest drone
with a fresh report, in 3D, and prints it every 5 s. Below
`SEPARATION_WARNING_M` (2.5 m) it prints a warning instead.

It changes no setpoint. It is a measurement. Avoidance would have to handle two
drones each moving out of the other's way into the other's new path.

On the default three-drone ring the chain compresses at each corner, because a
follower cuts the corner its leader turns. In the run above the smallest gap
reported was 2.9 m. [docs/SWARM.md](../../docs/SWARM.md) records a run where the
2.5 m warning fired once on a corner. `follow_distance_m:=2.0` makes it fire
reliably.

The chase rule only constrains the drone directly ahead. Nothing stops slot 3
flying through where slot 1 is. The altitude layers are what keep non-adjacent
drones apart.

### Unknown senders

`on_swarm_state()` ignores any `slot` of `count` or higher and warns. The usual
cause is a second swarm on the same `ROS_DOMAIN_ID`, or a node left running
from the previous launch. A second swarm whose slot numbers overlap this one's
cannot be detected this way and would be mixed into the formation, which is why
each machine on a shared network needs its own `ROS_DOMAIN_ID`.

### Build: a message in the same package

`CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/SwarmState.msg")

rosidl_get_typesupport_target(swarm_state_typesupport
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(formation_drone PRIVATE
  rclcpp::rclcpp ${px4_msgs_TARGETS} ${swarm_state_typesupport})
```

`rosidl_generate_interfaces` generates the header
`swarm_comms/msg/swarm_state.hpp` (the message name in snake_case) and its type
support library. A package that uses another package's message links it through
`find_package`. That target does not exist yet while this package is being
built, so `rosidl_get_typesupport_target` names the one being generated. Without
it the code compiles and then fails to link.

`package.xml` adds:

| Line | Why |
|---|---|
| `<buildtool_depend>rosidl_default_generators</buildtool_depend>` | Generates the code at build time |
| `<exec_depend>rosidl_default_runtime</exec_depend>` | Type support at run time |
| `<member_of_group>rosidl_interface_packages</member_of_group>` | Registers the package as defining interfaces, so `ros2 interface show` and other packages can find the message |

Large projects usually put messages in a separate interface package, so other
packages can depend on the message without depending on the node. One message
used by one node does not need that.

### The launch file

Identical in structure to `swarm_naive`'s: same pad layout, instance numbering,
spawn poses, staggering and Gazebo shutdown. It runs `formation_drone` and adds
one argument, `follow_distance_m`. Everything that makes this a swarm happens at
run time over `/swarm/state`, so none of it is configured in the launch file.

The start delay on the nodes matters slightly more here: a node that starts
late holds the whole swarm at the barrier.

### What this package still does not do

| Missing | Consequence |
|---|---|
| Collision avoidance | `check_separation()` measures, and nothing acts on it |
| Constraints on non-adjacent drones | Only the drone directly ahead shapes a follower's path |
| Redundancy or acknowledgements | One topic, no retries. A drone that loses the link holds, then goes home |

### Design decisions

| Decision | Reason |
|---|---|
| One broadcast topic for all drones | Simpler than per-drone topics, and a radio broadcasts anyway |
| Sender identified by the `slot` field | Works unchanged on real aircraft, where every autopilot is on bare `/fmu/` |
| Every field a repeated state | Loss costs nothing, so best effort is safe |
| Barrier instead of synchronised clocks | Needs one bit per drone and no time agreement |
| Age measured on the receiver's clock | Clock offset between drones cannot be mistaken for delay |
| Positions broadcast in the shared frame | The receiver can use them without knowing the sender's pad |
| Horizontal-only chase | Keeps the altitude layers intact |
| Hold, then go home, on a lost link | The same behaviour as PX4's own datalink-loss failsafe |
| Separation only measured | Avoidance is its own problem. This gives the number it would start from |
| Every drone receives the leader's route parameters | Every node gets identical parameters apart from identity, so which slot leads is decided in the code alone |
| PX4 code copied from `swarm_naive` | The file reads start to finish on its own |

### Where to go next

| To | Read |
|---|---|
| Both swarm packages side by side | [docs/SWARM.md](../../docs/SWARM.md) |
| Custom messages in general | "Your own message type" in [docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md) |
| What changes on real aircraft | "What changes on real aircraft" in [docs/SWARM.md](../../docs/SWARM.md) |
