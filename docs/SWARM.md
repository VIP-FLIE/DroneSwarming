# Flying more than one drone

Two packages, both C++, both worked examples meant to be read.

| Package | What the drones do | What they know about each other |
|---|---|---|
| `swarm_naive` | Each flies a fixed waypoint route round a shared ring | Nothing at all |
| `swarm_comms` | Slot 0 flies the ring, the rest chase the drone ahead | Position and state, 10 Hz, over `/swarm/state` |

Read [HOW-IT-WORKS.md](HOW-IT-WORKS.md) and
[`src/cpp_single_hop/src/hop.cpp`](../src/cpp_single_hop/src/hop.cpp) first. One drone
taking off, flying forward and landing is the whole of the PX4 conversation, and
neither package here changes any of it.

Then read `swarm_naive`, then `swarm_comms`.

The second package repeats the first one's PX4 handshake verbatim instead of
sharing a library, so each package reads start to finish on its own. Strip the
comments and normalise the names and 428 of `waypoint_drone.cpp`'s 450 lines of
code appear unchanged in `formation_drone.cpp`.

Diffing the two files is not useful. The comments are rewritten throughout, so
`diff` reports most of both. Everything communication adds is in six places:

| In `formation_drone.cpp` | What it is |
|---|---|
| `swarm_qos()` | The QoS for drone-to-drone telemetry, which is the one QoS choice in this repo that is ours rather than PX4's |
| `publish_swarm_state()` and `on_swarm_state()` | Saying where you are, and keeping track of where everybody else said they were |
| `to_swarm()` | Converting out of your own frame before anybody else reads it |
| `run_waiting_for_swarm()` | The barrier, which is the shared clock |
| `chase_target()` and `run_follower()` | The formation, and what to do when the messages stop |
| `check_separation()` | The measurement `swarm_naive` cannot make at all |

## Run it

Three terminals. The first two are the same simulator infrastructure as always,
and they have to be up before the third.

```bash
MicroXRCEAgent udp4 -p 8888              # 1. PX4 to ROS 2 bridge
```

```bash
drone-gcs                                # 2. ground station link PX4 requires
```

```bash
ros2 launch swarm_naive swarm.launch.py  # 3. PX4 instances and flight nodes
```

The launch file starts every PX4 instance and every node. It does not start the
agent or `drone-gcs`, because the agent has to be running before any PX4 starts
or that vehicle's `/fmu/` topics never appear at all.

On a machine you share with other people, 8888 is not yours alone and neither
is instance 1. Source `scripts/native/env.sh` and use your own agent port, which
is what `$PX4_UXRCE_DDS_PORT` holds:

```bash
MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT
```

`drone-gcs` and the launch files read the rest from the environment, so nothing
else changes. [LAB-SERVER.md](LAB-SERVER.md) explains what is being kept apart
and why.

Watch terminal 2 while the swarm boots:

```
Linked to PX4 system 2. Datalink failsafe satisfied for 1 vehicle(s); they can arm.
Linked to PX4 system 3. Datalink failsafe satisfied for 2 vehicle(s); they can arm.
Linked to PX4 system 4. Datalink failsafe satisfied for 3 vehicle(s); they can arm.
```

One line per drone. Your drones are systems 2 to `count + 1`, because SITL
instance N answers to system id N + 1 and the launch files number instances
from 1.

Tell it how many you launched and it does the counting for you. Do this for
anything bigger than a pair:

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

A count that stops short names the vehicles that will sit on their pads
reporting `Arming denied`. It also reports a link it loses and a link that comes
back, at PX4's own `COM_DL_LOSS_T` threshold of 10 s, so killing `drone-gcs`
or one vehicle mid-flight shows up on both sides:

```
Lost the link to PX4 system 3 after 10 s of silence. Its datalink-loss failsafe
will be firing now. 2 of 3 vehicles linked.
PX4 system 3 is back. 3 of 3 vehicles linked.
```

The whole flight takes about three minutes from launch, most of it waiting for
Gazebo and for three PX4 instances to boot in sequence.

```bash
ros2 launch swarm_naive swarm.launch.py count:=5 radius_m:=12.0
ros2 launch swarm_naive swarm.launch.py headless:=true count:=6
ros2 launch swarm_comms swarm.launch.py follow_distance_m:=6.0
ros2 launch swarm_naive swarm.launch.py --show-args      # every argument
```

Use `headless:=true` above about four drones, and on any machine without a GPU.
It skips rendering entirely. PX4 and Gazebo run in lockstep, so this costs
wall-clock time and GUI framerate, never flight behaviour. See
[SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md).

### Stopping

`Ctrl-C` in terminal 3 stops PX4 and the nodes. Gazebo is started by the first
PX4 through its own startup script, as a detached background process, so it does
not die with it. The launch file asks it to stop on shutdown. If one survives:

```bash
pkill -f 'gz sim'
```

A leftover Gazebo has a distinctive symptom. The next launch finds a world already
running, joins that one instead of starting a fresh one, and you get a field with
six drones in it where half of them do not respond.

## The flight

Pads are a row along east, 5 m apart, at north 0. Drone 0's pad is the origin of
the shared swarm frame. The ring is centred just north of the row.

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
| Climb | Straight up off its own pad | Same |
| Altitude | `altitude_m + slot * layer_m` | Same |
| Start | Whenever that drone finished arming | When every drone reports ready |
| Pattern | Every drone flies the ring, entering at vertex `slot` | Slot 0 flies the ring, slot k chases slot k-1 at `follow_distance_m` |
| Finish | Each drone lands when its own route runs out | Slot 0 says `done`, and each drone peels off when the one ahead does |

Both end with `VEHICLE_CMD_NAV_LAND` over the drone's own pad. PX4's own landing
routine handles ground detection and disarming, and it is the same code the real
aircraft runs.

## What a swarm needs that one drone does not

### One PX4 process per drone

Not a simulator quirk. Every airframe carries its own autopilot and its own
companion computer, and each companion computer only ever sees its own autopilot
on `/fmu/`. The launch file runs `px4 -i <instance>` once per vehicle.

`-i` is the whole of PX4's multi-vehicle support. Everything below is derived
from it, verified in PX4 v1.16.1:

| Derived from `-i N` | Value | Source |
|---|---|---|
| Topic namespace | `/px4_N/fmu/...` | `rcS` line 299 |
| MAVLink system id | `N + 1` | `rcS` line 140, `param set MAV_SYS_ID $((px4_instance+1))` |
| GCS MAVLink port | `18570 + N` | `px4-rc.mavlink`, `udp_gcs_port_local` |
| Gazebo model name | `x500_N` | `px4-rc.gzsim`, `MODEL_NAME_INSTANCE` |

Instance 0 is the exception: it publishes on bare `/fmu/...` with no namespace,
exactly as a real vehicle does. Both launch files here number from 1 so that
every drone is the same shape.

Every value in that table is derived from a single small integer, which is what
makes sharing a machine tractable. The `instance_base` launch argument shifts
the whole swarm by ten, and with it every port PX4 derives. Slot 1 flies
instances 11 to 20 and never touches slot 0's ports. It defaults to
`PX4_INSTANCE_BASE`, which is 0 unless `env.sh` has a `DRONE_SLOT` to work
from.

The system id is the one that bites quietly. PX4 drops any `VehicleCommand`
addressed to a different system, which is the only thing stopping one node from
arming every other drone on the field. Get it wrong and the node runs perfectly
while nothing ever arms.

### A frame that more than one drone can use

Every PX4 measures position from its own takeoff point. Three vehicles spawned
6 m apart each report `vehicle_local_position` within 3 cm of zero.

So `north 8` is a different place for every drone, and a flight plan written in
any one drone's frame is meaningless to the rest. Both packages solve it the
same way: the plan is written in a **shared swarm frame** whose origin is drone
0's pad, and each node converts.

```cpp
LocalPoint to_local(const SwarmPoint & point) const
{
  return LocalPoint{
    point.north_m - home_.north_m,
    point.east_m - home_.east_m,
    point.down_m};
}
```

That is the entire shared-frame problem. Two subtractions, and knowing which
two numbers to subtract.

`SwarmPoint` and `LocalPoint` hold the same three doubles and are separate types
on purpose. Handing one to something that wants the other is then a compile
error rather than a drone flying to the wrong field.

`down_m` passes through untouched because every pad is on the same flat ground
and shares an altitude datum. On sloping ground each drone needs a third offset
for the same reason it needs these two.

Where the offset comes from:

| | How the node learns where its pad is |
|---|---|
| Here | A launch parameter. The same launch file spawns the vehicle there and tells the node about it, so the two cannot disagree |
| Real aircraft | A survey, or an RTK fix taken on the pad. Formation spacing is only ever as good as this number: plain GPS is metres, RTK is centimetres |

### Spawn poses are in Gazebo's frame, which is ENU

`PX4_GZ_MODEL_POSE` is `"x,y,z,roll,pitch,yaw"` in Gazebo's world frame. Gazebo
is ENU: **x is east, y is north, z is up**. PX4's own frame is NED. So east and
north are swapped relative to everything else in the flight code.

Verified in the simulator rather than assumed:

| Spawned with | Measured offset from instance 1 |
|---|---|
| `PX4_GZ_MODEL_POSE="6,0"` | north +0.02 m, east +5.98 m |
| `PX4_GZ_MODEL_POSE="0,6"` | north +5.99 m, east -0.03 m |

Measured from the difference in `ref_lat` and `ref_lon` that the two estimators
reported. The conversion is in PX4 v1.16.1,
`src/modules/simulation/gz_bridge/GZBridge.cpp`, at the comment
`position ENU -> NED`.

Reverse this and every formation mirrors itself, which is difficult to see and
easy to write.

### Staggered startup

The first vehicle loads Gazebo and the world. Every other vehicle spawns a model
into a world that already exists. Launching them all at once means the later
ones look for a world that is not there yet.

| Argument | Default | What it is |
|---|---|---|
| `gazebo_boot_s` | 30.0 | Head start for the first vehicle |
| `stagger_s` | 8.0 | Gap between the rest |

PX4's own startup script waits up to 30 s for the world before giving up, so
these have that much slack on top. Raise `gazebo_boot_s` on a slow machine or a
heavy world.

## swarm_naive

Files: [`src/swarm_naive/`](../src/swarm_naive/)

| Read this | To understand |
|---|---|
| The header comment in `waypoint_drone.cpp` | What the package is, and the three things it deliberately does not do |
| `SwarmPoint` and `LocalPoint` | The two frames, and why they are two types |
| `build_route()` | The whole flight, as a list of points in the shared frame |
| `to_local()` | The conversion, which is the entire swarm problem |
| `station_altitude_m()` | The only deconfliction in the package |
| `run_flying()` | Nine lines. Everything that makes it a swarm happened before takeoff |
| `launch/swarm.launch.py` | Instance numbering, spawn poses, staggering, and where each node's parameters come from |

Every drone is handed a slot number and a flight plan at launch and flies it open
loop. Three consequences, all of them the reason `swarm_comms` exists:

**No collision avoidance.** Nothing in the program would notice if two aircraft
converged. Drone k flies at `altitude_m + k * layer_m`, and that static guess,
made before takeoff, is the only thing keeping them apart.

**No shared clock.** Each drone starts its lap when it personally finished
arming. A drone that took twenty seconds longer to arm is not twenty seconds
late, it is permanently in the wrong part of the ring.

**No formation.** Spacing holds only as long as the drones happen to fly at the
same speed. Nothing corrects a drift.

Run this to see what the altitude layering is buying:

```bash
ros2 launch swarm_naive swarm.launch.py layer_m:=0.0
```

## swarm_comms

Files: [`src/swarm_comms/`](../src/swarm_comms/)

| Read this | To understand |
|---|---|
| `msg/SwarmState.msg` | The entire interface between drones, and why every field is a level rather than an event |
| `swarm_qos()` | Why drone-to-drone telemetry is BEST_EFFORT, and why the depth is a function of swarm size |
| `to_swarm()` and `to_local()` | The two conversions, which are exact inverses |
| `run_waiting_for_swarm()` | The shared clock, solved without a clock |
| `chase_target()` | The four lines that make it a swarm |
| `run_follower()` | What to do when the messages stop |
| `check_separation()` | The measurement `swarm_naive` cannot make at all |
| `CMakeLists.txt` | Generating a message and using it from the same package |

### The message

One topic, `/swarm/state`, every drone publishing to it and subscribing to it.
No per-drone topics and no point-to-point addressing: a broadcast everybody can
hear is both simpler and closer to how a radio behaves.

It sits outside any `px4_` namespace deliberately. On real aircraft every
autopilot occupies bare `/fmu/...`, so there is no namespace left to tell drones
apart, and the `slot` field has to be what identifies the sender. Doing it that
way here means the code does not change when it leaves the simulator.

Every field is a **level**, never an **event**. Nothing says "I have just become
ready"; it says "I am ready", ten times a second, for as long as it stays true.
A receiver that misses a message learns the same thing 100 ms later.

That design is what lets the topic run BEST_EFFORT, which is what you want over
a radio: a retransmitted position half a second old is worse than the one that
arrived on time. Had `ready` been an announcement, one dropped packet would
deadlock the swarm.

| QoS | `/fmu/` topics | `/swarm/state` |
|---|---|---|
| Reliability | BEST_EFFORT, set by PX4 | BEST_EFFORT, chosen here |
| Durability | TRANSIENT_LOCAL, set by PX4 | VOLATILE |
| Depth | 1 | `2 * count` |

The depth matters. Every drone publishes to this one topic, so `count` messages
can arrive between two ticks of your control loop. A depth of 1 means hearing
from one drone per tick and treating every other drone as stale.

The `/fmu/in/vehicle_command` exception still applies: PX4 serves that one as a
RELIABLE DDS replier, so it needs a RELIABLE publisher or arm and mode commands
are dropped without warning.

### The shared clock, solved without a clock

Nobody moves until everybody reports ready. That converts "start at the same
time", which needs synchronised clocks and is hard over a radio, into
"start at the same event", which needs one bit per drone.

```
[INFO] [drone_1]: at station, waiting for the rest of the swarm.
[INFO] [drone_1]: holding at station, waiting for slot 2 (never heard from) (5 s of 180).
[INFO] [drone_1]: whole swarm is ready. Following slot 0.
```

The cost: the swarm now moves at the speed of its slowest member, and a drone
that never becomes ready stops everybody. `BARRIER_TIMEOUT_S` answers the second
half. Naming the missing slots in the warning is the difference between a bug
you can fix and a swarm that just sits there.

Message age is measured by **arrival time on the receiver's own clock**, never
by comparing the sender's `stamp_us` against the receiver's. Two drones do not
share a clock, so that comparison measures clock skew as well as age and the two
are indistinguishable.

### The rule that makes it a swarm

Sit `follow_distance_m` behind the drone ahead, on the line from you to it.

```cpp
return SwarmPoint{
  ahead.north_m - follow_distance_m_ * north_gap / range_m,
  ahead.east_m  - follow_distance_m_ * east_gap  / range_m,
  -station_altitude_m()};
```

One vector subtraction and one normalisation. The chain arranges itself:
whatever the leader does, each follower converges on a point a fixed distance
back along its own line of sight, so the formation bends round corners instead
of breaking. Closer than `follow_distance_m` and the target lands behind the
follower, so it backs off, which is what stops the chain concertinaing when the
leader slows.

This target cannot be computed without a message from another aircraft, and it
is recomputed ten times a second from live data. That is the whole difference
between the two packages.

It is horizontal only. Each drone stays on its own altitude layer, so the
distance being held is a ground distance and the vertical separation survives.

### When the messages stop

| Silence from the drone ahead | What the follower does |
|---|---|
| Under 1 s (`LINK_STALE_S`) | Nothing. Ten missed messages in a row is normal on a lossy link |
| 1 s to 15 s | Holds its last target and warns. It keeps streaming setpoints, because pausing the stream makes PX4 drop offboard regardless of why you stopped |
| Over 15 s (`LINK_LOST_ABORT_S`) | Abandons the formation and returns home |

Flying a formation off stale data is how two aircraft end up in the same place.
The number looks perfectly reasonable, it is just no longer true.

Coming home on a lost link is the same thing PX4's own datalink-loss failsafe
does, for the same reason.

### Separation is measured, not avoided

`check_separation()` reports the distance to the nearest other aircraft and
complains below `SEPARATION_WARNING_M`. It changes no setpoint and moves no
drone.

```
[INFO] [drone_2]: closest neighbour is slot 1 at 5.1 m.
```

It is in there because it is the number that tells you whether the formation is
working, and because it is exactly the measurement `swarm_naive` cannot make at
all. Real avoidance starts here and then has to answer the hard part, which is
that two aircraft both politely giving way can give way into each other.

On the default three-drone triangle the warning fires, once, as the chain rounds
a corner:

```
[WARN] [drone_1]: slot 2 is 2.5 m away, inside the 2.5 m we would like.
                  Nothing in this node will do anything about that.
```

That is the pursuit rule behaving correctly, not a fault. A follower cuts the
corner its leader turns, so the chain compresses on every turn and a 5 m
follow distance closes to about half that. Raise `radius_m` or lower
`follow_distance_m` and it stops. That is the
gap between "holding a formation" and "guaranteeing separation".

The chase rule also only constrains the drone directly ahead of you. Nothing
stops slot 3 from flying through where slot 1 is, so the altitude layering
inherited from `swarm_naive` is still doing real work.

## Parameters

Both packages take these. `--show-args` on either launch file prints them with
their descriptions.

| Argument | Default | Effect |
|---|---|---|
| `count` | 3 | Drones, and PX4 processes. Maximum 10 |
| `pad_spacing_m` | 5.0 | Gap between pads along east |
| `radius_m` | 8.0 | Radius of the ring |
| `altitude_m` | 4.0 | Altitude of drone 0 |
| `layer_m` | 2.0 | Vertical gap between slots |
| `laps` | 1 | Laps of the ring |
| `centre_north_m` | `radius_m + pad_spacing_m` | Ring centre, shared frame |
| `centre_east_m` | middle of the pad row | Ring centre, shared frame |
| `headless` | false | Skip Gazebo rendering |
| `world` | default | Gazebo world by file name, see [WORLDS.md](WORLDS.md) |
| `px4_dir` | `$PX4_DIR` or `~/PX4-Autopilot` | Which PX4 checkout to run |
| `gazebo_boot_s` | 30.0 | Head start for the first vehicle |
| `stagger_s` | 8.0 | Gap between the rest |
| `px4_output` | log | Where PX4's console goes. `log` is `~/.ros/log/<run>/launch.log`; `screen` puts it in the terminal, which is what you want when a vehicle will not boot |

`swarm_comms` adds one:

| Argument | Default | Effect |
|---|---|---|
| `follow_distance_m` | 5.0 | How far behind the drone ahead each follower sits |

Ten drones is the ceiling because `drone-gcs` heartbeats SITL instances 0 to 9
only, so an eleventh vehicle could never satisfy its datalink failsafe and could
never arm. Both nodes refuse to construct above it rather than taking off into a
swarm that cannot assemble.

## Things to try

| Command | What it shows |
|---|---|
| `ros2 launch swarm_naive swarm.launch.py layer_m:=0.0` | What the altitude layering was buying. Put every drone on one level and watch the ring |
| `ros2 launch swarm_comms swarm.launch.py follow_distance_m:=2.0` | The separation warning firing, and the chain running tight through corners |
| Kill `drone-gcs` mid-flight | Every vehicle's datalink-loss failsafe firing at once. PX4 takes the aircraft back and the nodes say so |
| Kill one node mid-flight in `swarm_comms` | The follower behind it holding station, then giving up and going home |
| `ros2 topic echo /swarm/state` | The whole swarm conversation, in one stream |
| `ros2 topic hz /swarm/state` | Should be `10 * count` |
| `ros2 launch swarm_naive swarm.launch.py world:=lawn` | Somewhere with scenery in it, see [WORLDS.md](WORLDS.md) |
| `ros2 launch swarm_comms swarm.launch.py count:=6 headless:=true laps:=2` | Six drones, two laps, no rendering |

## Problems

| Symptom | Cause | Fix |
|---|---|---|
| Drone 0 flies, the rest never arm | `drone-gcs` is serving one vehicle | It should name systems 2 to `count + 1`. Each vehicle enforces the datalink failsafe separately |
| `drone-gcs` says it is ignoring `UNKNOWN_<n>` messages | Normal. PX4 streams messages newer than pymavlink's bundled dialect, so it cannot read their headers | Nothing. It says so once and does not count them as vehicles |
| `ERROR [init] Timed out waiting for Gazebo world` | A later vehicle started before the first one finished loading | Raise `gazebo_boot_s` |
| Drones spawn into a field that already has drones in it | A Gazebo server survived the last run | `pkill -f 'gz sim'` |
| `No telemetry from the flight controller yet`, forever | The agent was started after PX4, or `px4_ns` is wrong | Start the agent first, then relaunch |
| Node runs, nothing ever arms | Wrong `system_id`. PX4 drops commands for other systems | Instance N needs system id N + 1 |
| A drone flies to the wrong part of the ring | Its `home_north_m` and `home_east_m` disagree with where it was spawned | Both come from the launch file. Check `PX4_GZ_MODEL_POSE` is `east,north` |
| Formation mirrors itself | East and north swapped in the spawn pose | Gazebo is ENU, PX4 is NED |
| `swarm_comms` drones all hold station forever | The barrier is waiting on a slot | The warning names which slots and why. `ros2 topic echo /swarm/state` shows who is talking |
| `Heard slot N, but this swarm only has M drones` | Another swarm on the same `ROS_DOMAIN_ID`, or a node left over from the last run | Give each person a different `ROS_DOMAIN_ID`. `scripts/native/env.sh` already restricts discovery to localhost |
| Everything runs at 0.2x real time | Too many vehicles for the machine | `headless:=true`, or fewer drones. Flight behaviour is unaffected, PX4 and Gazebo are in lockstep |
| `No executable found` | Package built before the executable was added | `colcon build --symlink-install`, then `source install/setup.bash` |
| Launch file not found | `install(DIRECTORY launch ...)` missing from `CMakeLists.txt` | It is what puts the file in `share/<package>` |

## What changes on real aircraft

| | Simulation | Field |
|---|---|---|
| Network | One machine, DDS discovery free, no loss | WiFi or a radio mesh. Real latency, loss and partitions. Test it separately |
| Agent | One `MicroXRCEAgent` for the fleet | One per companion computer |
| Shared frame | A launch parameter that cannot be wrong | A survey or an RTK fix. Plain GPS is metres, RTK is centimetres, and that decides how tight a formation you can fly |
| Position estimate | Near perfect | The limit on formation spacing |
| A bad setpoint | Costs a restart | Does not |

## Reference

- [HOW-IT-WORKS.md](HOW-IT-WORKS.md), the pipeline and the three rules PX4 imposes
- [`src/cpp_single_hop/src/hop.cpp`](../src/cpp_single_hop/src/hop.cpp), one drone, read it first
- [NEW-PACKAGE.md](NEW-PACKAGE.md), starting a package of your own
- [UNDER-THE-SCRIPTS.md](UNDER-THE-SCRIPTS.md), what the launch files are doing when they run the PX4 binary directly
- [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md), what the simulator gets right
- [WORLDS.md](WORLDS.md), flying somewhere other than the grey plane
- [Multi-vehicle with ROS 2](https://docs.px4.io/v1.16/en/ros2/multi_vehicle)
- [Multi-vehicle in Gazebo](https://docs.px4.io/v1.16/en/sim_gazebo_gz/multi_vehicle_simulation.html)
- [px4_msgs release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg)
- [PX4 frame and notation conventions](https://docs.px4.io/v1.16/en/contribute/notation.html)
- [ROS 2 launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 custom messages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2 Quality of Service](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
