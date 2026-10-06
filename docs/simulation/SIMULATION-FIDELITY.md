# Simulation fidelity

Code here goes onto real X500s. What is faithful, what is not.

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

Verify:

```bash
cd ~/PX4-Autopilot && git status --short   # the two deleted NuttX submodules
git diff --name-only ROMFS/                # empty: no airframe or param edits
git -C Tools/simulation/gz status --short  # added links only
```

The only deletions are two NuttX submodules (firmware toolchain, unused by SITL). Nothing in this repo sets a PX4 parameter or changes PX4's worlds, airframes or sensor definitions.

After `./scripts/install-worlds.sh`, the first command lists `Tools/simulation/gz` (this repo's worlds and models linked beside PX4's own). The links add files, change none of PX4's. The third command shows every line as `??`. `scripts/smoke-test.sh` reads the tree by the same rule.

### Smoke test submodule drift

Section 2 prints two `note` lines on a fresh install:

```
plus 7 submodule(s) at a different commit than PX4 records, expected from the installer's shallow clone
plus 9 uninitialized submodule(s), which the installer's --recursive clone does not populate
```

Both are expected. The installer clones with `--shallow-submodules` (~1 GB of submodule history omitted). A shallow submodule clone may not land on the exact commit PX4 records. `--recursive` does not populate every submodule PX4 has (including nested ones). The drifted and uninitialized submodules are alternative simulators and flight-controller board trees. The Gazebo Harmonic SITL build compiles none of them. The check fails on a genuine edit to a tracked PX4 file. These notes cover submodule entries (installer artifacts).

## Hardware does not change physics

PX4 SITL runs lockstep: PX4 and Gazebo step together, PX4's control loop runs on simulated time. A slow machine takes longer in wall-clock seconds. It does not drop steps or integrate more coarsely.

Real-time factor, headless, no cameras, software rendering:

| Vehicles | RTF |
|---|---|
| 1 | 0.988 |
| 4 | 1.000 |
| 8 | 0.999 |
| 8, limited to 4 CPU cores | 0.999 |

## Cameras

Single vehicle, `x500_mono_cam`, camera at 30 Hz:

| Rendering | Frames delivered | RTF |
|---|---|---|
| Software (llvmpipe) | 11.5 Hz | 0.744 |
| NVIDIA GPU | 28.5 Hz | 0.999 |

Software rendering delivers one third of the requested frame rate. Vision or optical-flow algorithms receive fewer frames, further apart in simulated time, than on the aircraft. Do not tune vision behavior against software rendering.

- Vision work needs a GPU.
- No GPU: lower `<update_rate>` in the model SDF to what the machine can hold.
- Not using cameras: unaffected. State estimation, formation control, trajectory planning and collision avoidance run at full fidelity.

## Satisfy PX4's checks, never disable them

PX4's x500 airframe sets `NAV_DLL_ACT 2` (return on datalink loss). PX4 refuses to arm until it sees a ground station:

```
WARN [commander] Arming denied: Resolve system health failures first
WARN [health_and_arming_checks] Preflight Fail: No connection to the ground control station
```

`param set NAV_DLL_ACT 0` deletes the failsafe. `drone-gcs` sends GCS heartbeats and leaves `NAV_DLL_ACT` at 2. Arming goes from denied to `Armed by internal command`.

`drone-gcs` listens on udp 14550 and heartbeats each SITL instance's port (`18570 + instance` per `px4-rc.mavlink`). Every vehicle enforces this failsafe separately. Killing `drone-gcs` mid-flight fires the datalink-loss failsafe.

## Watch RTF

RTF in Gazebo's bottom bar:

- RTF near 1.0: wall-clock and simulated time agree.
- RTF below 1.0: physics correct, but wall-clock timers run faster relative to simulated time. Control rates, timeouts and staleness checks stop transferring.

Nodes in `src/py_single_square/` use `self.get_clock().now()` and `create_timer()`. `time.time()` in a control loop is the bug.

## No firmware

PX4 is built with `--no-nuttx`: SITL only, no Pixhawk firmware. ROS 2 nodes in `src/py_single_square/` go on the companion computer, identical between sim and hardware. PX4 firmware is flashed from an official release via QGroundControl.

For custom firmware: drop `--no-nuttx` from `Tools/setup/ubuntu.sh`, stop pruning `platforms/nuttx/NuttX`, remove `--shallow-submodules` from the PX4 clone (`src/lib/version/px_update_git_header.py` reads the newest NuttX tag; a shallow checkout has none, crashes every build including SITL). Adds the ARM cross toolchain and ~250 MB.

## What SITL never gets right

- Networking. One host here. Real drones: WiFi, real latency and loss. Test separately.
- Sensor noise. SITL sensor models are simplified. Real IMUs, barometers and GPS are noisier.
- Compute. Companion computer is weaker than a laptop. Profile on target.
- Timing jitter. Lockstep is deterministic. Hardware is not.

These require hardware-in-the-loop and flight testing.