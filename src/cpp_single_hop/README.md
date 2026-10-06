# cpp_single_hop

One drone, one C++ node. The drone takes off, flies 3 m north, and lands.

This is the first package to read for flight code. `swarm_naive` and
`swarm_comms` repeat its PX4 code almost line for line.

## 1. What the drone does

| Phase | What happens | Moves on when |
|---|---|---|
| `Priming` | The drone sits disarmed on its pad. The node streams setpoints at 10 Hz | 10 setpoints (1 s) have gone out |
| `Arming` | The node asks PX4 to arm every 0.5 s | PX4 reports armed. Gives up after 60 s |
| `EngagingOffboard` | The node asks PX4 to switch to offboard mode every tick | PX4 reports it is in offboard |
| `Climbing` | Straight up to `altitude_m` (3 m) above the pad | Within 0.5 m of the target |
| `FlyingForward` | `forward_m` (3 m) north, holding altitude | Within 0.5 m of the target. The node sends a land command |
| `Landing` | PX4's own land mode descends and disarms | PX4 reports disarmed |
| `Done` | The control timer is cancelled | Nothing. The process stays up until Ctrl-C |

The node never controls a motor. It sends PX4 a target position ten times a
second, and PX4's position controller flies the drone there.

## 2. What you learn from it

| Concept | Where |
|---|---|
| The three rules PX4 imposes on any offboard program | Marked `RULE 1`, `RULE 2`, `RULE 3` in the code |
| Rule 1: QoS has to match PX4's or subscriptions never fire | `px4_stream_qos()`, `px4_command_qos()` |
| Rule 2: PX4 needs a setpoint stream before and during offboard | `run_control_tick()`, `publish_offboard_control_mode()` |
| Rule 3: arming is a safety interlock with preflight checks | `run_arming()`, `describe_arming_blocker()` |
| The NED frame, where up is negative | `Position`, `target_position()` |
| PX4 timestamps in microseconds | `px4_timestamp_us()` |
| Sending arm, mode and land commands, and addressing them | `send_vehicle_command()` |
| A flight as a state machine on a 10 Hz timer | `Phase`, `advance_phase()` |
| Handling telemetry that has not arrived yet | `std::optional` members |
| ROS 2 parameters | The constructor |

[docs/HOW-IT-WORKS.md](../../docs/HOW-IT-WORKS.md) explains the whole
pipeline behind this node and is written to be read alongside it.

## 3. How to run it

### Build

From the repo root, the directory that contains `src/`:

```bash
colcon build --symlink-install --packages-select cpp_single_hop
source install/setup.bash
```

This is C++, so every edit to the source needs a rebuild.

### Run in the simulator

Four terminals. Each terminal needs `source scripts/env.sh` first, unless
it is in your `~/.bashrc`.

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
ros2 run cpp_single_hop hop      # 4. this package
```

Start terminal 4 once terminal 3 prints `Ready for takeoff!`.

| Where to watch | |
|---|---|
| Gazebo | A window on your desktop |
| No rendering | `HEADLESS=1 drone-sim` in terminal 3. The flight is identical |

Output from a real run:

```
[INFO] [hop]: will climb to 3.0 m, fly 3.0 m north, and land. Commanding /fmu/.
[WARN] [hop]: No telemetry from the flight controller yet. Is MicroXRCEAgent running, and did it start before PX4?
[INFO] [hop]: armed.
[INFO] [hop]: offboard engaged, climbing to 3.0 m.
[INFO] [hop]: at altitude, flying 3.0 m forward.
[INFO] [hop]: arrived, landing.
[INFO] [hop]: landed and disarmed. Flight complete.
```

One `No telemetry` warning at startup is normal. The first control tick runs
before PX4's first message arrives. The same warning repeating every 5 s is a
real problem, covered in the troubleshooting table below.

The node keeps running after `Flight complete`. Press Ctrl-C to exit it. To fly
again, run it again. PX4 does not need restarting.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `altitude_m` | 3.0 | Height above the pad. Zero or less and the node refuses to start |
| `forward_m` | 3.0 | Distance to fly north. Negative flies south |
| `px4_ns` | empty | Topic namespace. Empty means bare `/fmu/...`, which is SITL instance 0 and every real aircraft |
| `system_id` | 1 | MAVLink system id of the vehicle. SITL instance N is system N + 1 |

```bash
ros2 run cpp_single_hop hop --ros-args -p altitude_m:=6.0 -p forward_m:=10.0
ros2 run cpp_single_hop hop --ros-args -p px4_ns:=px4_1 -p system_id:=2
```

### Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `No telemetry from the flight controller yet` every 5 s | Agent not running, agent started after PX4, or `px4_ns` wrong | Start the agent, then restart `drone-sim` |
| `not armed yet ... no ground control station link` | `drone-gcs` is not running | Start it. Do not set `NAV_DLL_ACT 0` |
| `not armed yet ... preflight checks still failing` | PX4 is still settling its position estimate | Wait. Check `commander status` at the `pxh>` prompt if it lasts |
| `PX4 reports itself ready, so the command may not be reaching it` | `system_id` does not match the vehicle | Instance N needs `system_id` N + 1 |
| `PX4 left offboard mode` mid-flight | A failsafe fired, for example `drone-gcs` was killed | Expected in that case. PX4 is flying its failsafe |
| `Package 'cpp_single_hop' not found` | Overlay not sourced in this terminal | `source install/setup.bash` |
| Exits at once with `altitude_m must be greater than zero.` | Bad parameter | Pass a positive altitude |

The full table for the stack is in [HOW-IT-WORKS.md](../../docs/simulation/HOW-IT-WORKS.md).

## 4. How it works

### Files

| File | What it is |
|---|---|
| `include/cpp_single_hop/hop.hpp` | The declarations: the tunables, the QoS helpers, the `Position` and `Phase` types, and the `Hop` class |
| `src/hop.cpp` | The method bodies |
| `src/main.cpp` | `main()`, and nothing else |
| `CMakeLists.txt` | Build rules. Builds `hop`, links `rclcpp` and `px4_msgs`, installs it where `ros2 run` looks |
| `package.xml` | Package name and dependencies: `ament_cmake`, `rclcpp`, `px4_msgs` |

### Who flies the drone

```
hop  --setpoints-->  MicroXRCEAgent  -->  PX4 (commander, position controller)  -->  motors
     <--telemetry--                  <--
```

`hop` publishes ROS 2 topics. `MicroXRCEAgent` bridges them into PX4. PX4's
position controller turns each target position into attitude and thrust
commands. The position controller is the same code on a real Pixhawk. A bug in `hop` can
send the drone to the wrong place. It cannot bypass PX4's controller.

`drone-gcs` is a separate link. It sends MAVLink heartbeats so PX4's
datalink-loss failsafe has the ground station it expects. It sends no commands.

### Topics

| Direction | Topic | Message | QoS | Purpose |
|---|---|---|---|---|
| Out | `/fmu/in/offboard_control_mode` | `OffboardControlMode` | Best effort, transient local | "Act on the position field." Its arrival is what tells PX4 an offboard controller is alive |
| Out | `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Best effort, transient local | Where to be |
| Out | `/fmu/in/vehicle_command` | `VehicleCommand` | **Reliable**, transient local | Arm, change mode, land |
| In | `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | Best effort, transient local | PX4's position estimate |
| In | `/fmu/out/vehicle_status_v1` | `VehicleStatus` | Best effort, transient local | Armed or not, flight mode, failsafe flags |

PX4's DDS client creates all of these names. They cannot be renamed from the ROS
side. `VehicleStatus` is a versioned message in PX4 1.16
(`MESSAGE_VERSION = 1` in `msg/versioned/VehicleStatus.msg`), and the client
appends the version to the topic name, which is where `_v1` comes from.

### Rule 1: QoS

PX4 publishes `/fmu/out/` as best effort with transient local durability. A
default ROS 2 subscription is reliable and volatile. The two are incompatible,
so a default subscription is created without error, never connects, and its
callback never fires. Nothing is logged.

`ros2 topic echo` negotiates QoS for itself and prints data, so the topic looks
healthy from the command line while the node receives nothing.

`px4_stream_qos()` matches PX4. `px4_command_qos()` is the exception:
PX4 serves `vehicle_command` as a reliable DDS replier, so a best effort
publisher's commands are dropped without a warning.

Depth is 1 on everything. Only the newest position estimate is useful.

### Rule 2: the setpoint stream

PX4 refuses to enter offboard mode unless setpoints are already arriving, and
leaves offboard if they stop for about half a second.

`run_control_tick()` runs at 10 Hz and publishes `OffboardControlMode` and
`TrajectorySetpoint` first, on every tick, before it decides anything. That
starts in `Priming`, while the drone is still on the ground, and continues
until `Landing`. During `Priming` and `Arming` the setpoint is the point
directly above the pad. PX4 ignores it until offboard is engaged, and the stream
has to exist regardless.

The stream stops at `Landing`, because PX4's land mode has the vehicle and no
longer wants setpoints.

### Rule 3: arming

PX4 runs preflight checks and refuses to arm until they pass. The check that
catches everyone is the datalink check. The x500 airframe sets
`NAV_DLL_ACT 2`, return home on datalink loss, and PX4 will not take off with a
failsafe whose precondition is already broken. `drone-gcs` supplies the link.

`run_arming()` retries at 2 Hz for up to 60 s. It stays quiet for the first
3 s, because a healthy vehicle arms inside that. After that it prints
`describe_arming_blocker()`, which reads PX4's own `gcs_connection_lost`,
`failsafe` and `pre_flight_checks_pass` fields and names the one that is
failing.

This repo satisfies PX4's checks and never disables them.
[docs/SIMULATION-FIDELITY.md](../../docs/SIMULATION-FIDELITY.md) explains why.

### The state machine

`advance_phase()` runs after the stream has gone out on each tick. It returns
early until both `local_position_` and `vehicle_status_` have arrived. Past that
guard, the accessors call `std::optional::value()`, which throws if the
assumption is ever wrong instead of reading garbage.

`target_position()` holds the entire flight path. Every phase except
`FlyingForward` targets the point above the pad. `FlyingForward` targets
`forward_m` north of it.

`EngagingOffboard` checks `is_offboard()` before sending the mode command, and
sends it again every tick until it takes. This also covers a second run: after
a landing, PX4 goes back to the last mode the vehicle was able to arm in, which
is offboard (`UserModeIntention::onDisarm()` in PX4 v1.16.1). On the next run
the vehicle arms directly into offboard, and this phase passes on its first
tick.

`warn_if_px4_took_control()` runs every tick while airborne. If PX4 leaves
offboard, for example because a failsafe fired, the node says so and keeps
streaming. PX4's failsafe takes priority over the node.

### Frames and units

| | |
|---|---|
| Frame | NED: north, east, down. Down is positive downward, so 3 m up is `down_m = -3` |
| Origin | Where this vehicle's position estimate started, which is its own pad |
| Distance | Metres |
| Timestamps | Microseconds. ROS time is nanoseconds, so `px4_timestamp_us()` divides by 1000. PX4 ignores setpoints with wrong timestamps without saying so |

Getting the sign of `down` backwards commands the drone into the ground.

### Commands

`send_vehicle_command()` fills one `VehicleCommand`. The same message carries
every command, and the meaning of `param1` to `param7` depends on `command`.

| Command | Params | Meaning |
|---|---|---|
| `VEHICLE_CMD_COMPONENT_ARM_DISARM` | `param1 = 1` | Arm |
| `VEHICLE_CMD_DO_SET_MODE` | `param1 = 1`, `param2 = 6` | Base mode "custom", PX4 custom main mode 6, which is offboard |
| `VEHICLE_CMD_NAV_LAND` | none | Land where you are |

`target_system` must match the vehicle's MAVLink system id. PX4 drops commands
addressed to any other system. That is what stops one node from arming every
drone in a swarm, and it is why a wrong `system_id` looks like a node that runs
correctly while nothing arms. `from_external = true` marks the command as
coming from outside the flight controller, as a companion computer's would.

### Design decisions

| Decision | Reason |
|---|---|
| One package, no helper library | The header declares, the `.cpp` defines, and `main()` starts it. Nothing to chase outside the package |
| Every number is a named constant at the top | No bare numbers in the logic |
| Telemetry held in `std::optional` | Empty means PX4 has not spoken yet, and the type forces the check |
| `ARRIVAL_RADIUS_M = 0.5` | A hovering multicopter drifts by tens of centimetres. A tighter radius means the drone hunts and never arrives. It is also why the drone lands about 0.4 m short of 3 m |
| Arm retried at 2 Hz | Asking faster does not change PX4's answer and floods the command link |
| Offboard requested until confirmed | A single request can be lost or refused. Checking `nav_state` is the only proof it took |
| Landing handed to PX4 with `NAV_LAND` | PX4's land mode does ground detection and disarming, and it is the code the real aircraft runs |
| Disarm is the end-of-flight signal | PX4 disarms only once it has detected the ground |
| `rclcpp::create_timer` on the node clock | The loop follows ROS time, which matters if simulated and wall time diverge |
| Constructor throws on `altitude_m <= 0` | The node never publishes a setpoint it cannot mean |
| `px4_ns` and `system_id` parameters on a one-drone program | The same binary runs against instance 0, instance N, and a real aircraft |
| Velocity and yaw fields left at zero | PX4 reads zero as a command: stop at the target, face north. `NaN` is how to tell PX4 not to control a field |

### Where to go next

| To | Read |
|---|---|
| Fly a path or a function of time | "Changing the flight" in [docs/HOW-IT-WORKS.md](../../docs/HOW-IT-WORKS.md) |
| Compare the same flight in Python | [`../py_single_square`](../py_single_square/README.md) |
| Fly several drones | [`../swarm_naive`](../swarm_naive/README.md) |
| Start your own package | [docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md) |
