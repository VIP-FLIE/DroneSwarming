# Simulation fidelity

Code written here goes onto real X500s, so the question is whether it behaves
the same in the air. This lists what is faithful, what is not, and the numbers.

## Nothing in the simulation was changed

| | Value | Source |
|---|---|---|
| PX4 | v1.16.1 | pinned |
| px4_msgs | `release/1.16` | pinned |
| Physics engine | ODE | PX4 stock `default.sdf` |
| Max step size | 0.004 s | PX4 stock |
| Update rate | 250 Hz | PX4 stock |
| Gravity, magnetic field | stock | PX4 stock |
| Airframe and SITL params | untouched | PX4 defaults |
| Lockstep | enabled | PX4 SITL default |

Check it yourself in the container:

```bash
cd ~/PX4-Autopilot && git status --short   # no modified tracked files
git diff --name-only ROMFS/                # empty: no airframe or param edits
```

The only deletions are the two NuttX submodules, which are firmware toolchain
for real flight controllers and unused by SITL. Nothing in this repo sets a PX4
parameter or touches a world, airframe or sensor definition.

## Your hardware does not change the physics

PX4 SITL runs lockstep: PX4 and Gazebo step together and PX4's control loop
runs on simulated time. A slow machine takes longer in wall-clock seconds to
simulate the same flight. It does not drop steps or integrate more coarsely.

Real-time factor, headless, no cameras, software rendering:

| Vehicles | RTF |
|---|---|
| 1 | 0.988 |
| 4 | 1.000 |
| 8 | 0.999 |
| 8, limited to 4 CPU cores | 0.999 |

## Cameras are the one real limit

Single vehicle, `x500_mono_cam`, camera configured at 30 Hz:

| Rendering | Frames delivered | RTF |
|---|---|---|
| Software (llvmpipe) | 11.5 Hz | 0.744 |
| NVIDIA GPU | 28.5 Hz | 0.999 |

Software rendering delivers a third of the frame rate you asked for. A vision
or optical-flow algorithm receives fewer frames, further apart in simulated
time, than it will on the aircraft. Do not tune vision behavior against it.

- Vision work needs a GPU. Set `DOCKER_RUNTIME=nvidia` in `.devcontainer/.env`,
  or run `./scripts/doctor.sh --write`.
- No GPU available: lower the camera `<update_rate>` in the model SDF to what
  the machine can hold, so your code receives what it was told to expect.
- Not using cameras: unaffected. State estimation, formation control,
  trajectory planning and collision avoidance run at full fidelity anywhere.

## Satisfy PX4's checks, never disable them

PX4's x500 airframe sets `NAV_DLL_ACT 2`, meaning return on datalink loss. With
that armed PX4 refuses to arm until it sees a ground control station, same as a
real aircraft:

```
WARN [commander] Arming denied: Resolve system health failures first
WARN [health_and_arming_checks] Preflight Fail: No connection to the ground control station
```

The common fix is `param set NAV_DLL_ACT 0`. That deletes the failsafe rather
than supplying the link, so the simulated vehicle stops matching the real one
when telemetry drops, and you never test the return it performs.

`drone-gcs` sends GCS heartbeats instead, the role QGroundControl plays, and
leaves `NAV_DLL_ACT` at 2. Arming goes from denied to `Armed by internal
command` with the failsafe intact.

It listens on udp 14550 and also heartbeats each SITL instance's own GCS port,
`18570 + instance` per `px4-rc.mavlink`. Both halves matter for a swarm: every
vehicle enforces this failsafe separately, so a station that reaches only one
of them leaves the rest correctly refusing to arm.

Killing `drone-gcs` mid-flight is now a test: it fires the datalink-loss
failsafe. Apply the same rule to any other PX4 refusal.

## Watch the real-time factor

RTF in Gazebo's bottom bar is the fidelity gauge.

- RTF near 1.0: wall-clock and simulated time agree. Wall-clock timers tick at
  the rate they will on the drone.
- RTF below 1.0: physics is still correct, but a node on wall-clock timers runs
  faster relative to simulated time than it will in reality. Control rates,
  timeouts and staleness checks stop transferring.

The nodes in `src/drone_vip/` use `self.get_clock().now()` and `create_timer()`
rather than `time.time()` and `sleep()`. Keep that. `time.time()` in a control
loop is the bug.

## This container cannot build firmware

PX4 is set up with `--no-nuttx`, so it builds SITL only. It cannot compile
firmware for a Pixhawk.

That fits the architecture: the ROS 2 nodes in `src/drone_vip/` are what goes
on the drone's companion computer, identical between sim and hardware, and PX4
firmware is flashed from an official release via QGroundControl.

For custom firmware, drop `--no-nuttx` from the `Tools/setup/ubuntu.sh` call in
the Dockerfile and stop pruning `platforms/nuttx/NuttX`, both in section 7.
Adds the ARM cross toolchain and about 250 MB. Make it its own commit.

## What SITL never gets right

- Networking. Everything is on one host here. On real drones DDS discovery runs
  over WiFi across machines, with real latency and loss. Test that separately.
- Sensor noise. PX4's SITL sensor models are simplified. Real IMUs, barometers
  and GPS are noisier and drift more.
- Compute. A companion computer is much weaker than your laptop. Profile on the
  target.
- Timing jitter. Lockstep is deterministic. Hardware is not.

These are reasons to keep hardware-in-the-loop and flight testing, not reasons
to distrust the simulation.
