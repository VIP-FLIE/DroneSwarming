# What the simulation gets right, and what it does not

_Short version up top: the work to make the container run on everybody's laptop did not cost us any physics or flight fidelity. There is exactly one place where a weak machine changes what your code actually sees, and it is cameras. The rest of this doc is the evidence for both halves of that sentence._

This matters more here than it would on a normal software project, because the code we write against this simulation is eventually going onto real X500s. So the question is never really 'does it run on my laptop', it is 'will it behave the same way once it is in the air'. Anywhere the answer is no, we should know about it on purpose rather than find out on a field day.

## Nothing about the simulation itself got changed:
The cross platform work touched packaging only. Everything that actually determines how the drone behaves is stock PX4, untouched:

| Thing | Value | Where it comes from |
|---|---|---|
| PX4 | v1.16.1 | pinned, unchanged |
| px4_msgs | `release/1.16` | pinned, unchanged |
| Physics engine | ODE | PX4's stock `default.sdf` |
| Max step size | 0.004 s | PX4's stock `default.sdf` |
| Update rate | 250 Hz | PX4's stock `default.sdf` |
| Gravity, magnetic field | stock | PX4's stock `default.sdf` |
| Airframe and SITL params | untouched | PX4 defaults |
| Lockstep scheduler | enabled | PX4 SITL default |

No airframe parameters, sensor models, noise models, or physics settings were loosened to make anything run faster or easier on a slow machine. If any of that ever does need to change, it should be its own commit with its own reasoning, not something that quietly rides along with a packaging change.

## Why a slow laptop does not mean bad physics:
This is the part that makes the whole 'run it on any laptop' idea defensible, and it is worth understanding rather than taking on faith.

PX4 SITL runs in what is called lockstep, meaning PX4 and Gazebo step the simulation forward together, and PX4's control loop runs on simulated time instead of wall clock time. The practical upshot is that a slower machine does not start dropping physics steps or integrating more coarsely to keep up. It just takes longer in real seconds to simulate the same flight. The controller sees the same sequence of states on somebody's MacBook Air as it does on the lab machine, it just gets there slower.

## Swarm scale turned out to be basically free:
I was expecting this to be the problem, and it is not. These are PX4 SITL instances running headless, no cameras, on software rendering:

| Vehicles | Real time factor |
|---|---|
| 1 | 0.988 |
| 4 | 1.000 |
| 8 | 0.999 |
| 8, limited to 4 CPU cores | 0.999 |

Eight X500s holding real time even when the container is deliberately choked down to four cores. Multi vehicle physics is cheap, so scale is not where our risk is.

## Cameras are where it actually bites:
Here is the one real limitation, and if you take nothing else from this doc take this. Same single vehicle, using the `x500_mono_cam` airframe with the camera configured at 30 Hz:

| Rendering | Frames actually delivered | Real time factor |
|---|---|---|
| Software (llvmpipe) | 11.5 Hz | 0.744 |
| Nvidia GPU (EGL offscreen) | 28.5 Hz | 0.999 |

Software rendering hands you roughly a third of the frame rate you asked for. That is not a cosmetic difference you can shrug at. A vision or optical flow algorithm genuinely receives fewer frames, further apart in simulated time, than it is going to get on the real aircraft. So do not tune vision based behavior against software rendered camera output and expect it to transfer, because it will not.

What to do about it, in rough order of preference:

- If you are doing anything vision based, validate it on a machine with a GPU. Set `DOCKER_RUNTIME=nvidia` in `.devcontainer/.env`, or just run `./scripts/doctor.sh --write` and it will do it for you.
- You do not need the native display config to get this. GPU sensor rendering happens offscreen, so somebody on Windows using the browser GUI config still gets full rate cameras as long as the Nvidia runtime is on. Only the GUI window itself is software rendered.
- No GPU available anywhere? Then turn the camera `<update_rate>` down in the model SDF to something the machine can actually hold, so that what your code receives matches what you told it to expect. An honest 10 Hz is a lot better than a 30 Hz setting that quietly gives you 11.
- If you are not touching cameras at all, none of this applies to you. State estimation, formation control, trajectory planning, collision avoidance off position data, all of that runs at full fidelity on anybody's machine.

## Keep an eye on the real time factor:
The RTF number in the bottom bar of Gazebo is the closest thing we have to a fidelity gauge, so it is worth glancing at.

When it sits around 1.0, wall clock time and simulated time agree, and a ROS 2 node using normal timers ticks at the same rate it will on the real drone. When it drops below 1.0 the physics is still correct, but a node running off wall clock timers is now effectively running faster relative to simulated time than it ever would in reality, which means control rates, timeouts, and staleness checks stop transferring honestly.

The nodes already in `src/drone_vip/` do the right thing here, they use `self.get_clock().now()` and `create_timer()` rather than `time.time()` and `sleep()`. Stick with that pattern. If you catch yourself reaching for `time.time()` inside a control loop, that is the bug, not the fix.

## This container cannot build flight firmware:
Worth being upfront about. The image sets PX4 up with `--no-nuttx`, so it builds SITL and only SITL. It cannot compile firmware for the Pixhawk on an actual X500. That was already true of the original container, it is not something the cross platform work took away.

For the way we are actually working, that is fine, and it is worth being clear on why. The ROS 2 nodes in `src/drone_vip/` are the thing that goes onto the drone's companion computer, and that code is identical between simulation and hardware. The PX4 firmware on the flight controller normally gets flashed from an official release through QGroundControl rather than compiled by us.

If we ever do need custom firmware, it is a deliberate change and not a hard one. Drop `--no-nuttx` from the `Tools/setup/ubuntu.sh` call in the Dockerfile and stop pruning `platforms/nuttx/NuttX`, both of which live in section 7. That pulls in the ARM cross toolchain and adds roughly 250 MB. Make it its own commit so the image getting bigger is traceable to a decision somebody made.

## Things SITL will never get right:
None of these are new and none of them are our fault, but design around them anyway:

- Networking. Inside the container everything is on one host. On real drones, DDS discovery is happening over WiFi across several machines, with real latency and real packet loss. Test multi machine discovery before you count on it.
- Sensor noise. PX4's SITL sensor models are simplified. Real IMUs, barometers, and GPS are noisier and drift more than what you see here.
- Compute. A companion computer is much weaker than your laptop. If a node is working your laptop hard in simulation, profile it on the actual target before assuming it fits.
- Timing jitter. Lockstep gives you clean deterministic stepping. Real hardware does not, ever.

None of that is a reason to distrust the simulation. It is the reason we still do hardware in the loop and actual flight testing rather than declaring victory from a laptop.
