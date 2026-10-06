# DroneSwarming
University of Delaware VIP Drone team, Swarm subteam.

AI was used to port code from the ROS1 version, create examples, and document. Code run on a physical drone is written and reviewed by humans.

## Setup

ROS 2 Jazzy, Gazebo Harmonic and PX4 install on Ubuntu 24.04 and derivatives.

**Linux.** [docs/setup/NATIVE-SETUP.md](docs/setup/NATIVE-SETUP.md).

```bash
git clone git@github.com:VIP-FLIE/DroneSwarming.git && cd DroneSwarming
./scripts/install.sh
```

**macOS.** Ubuntu 24.04 in a UTM VM, then the same install inside it. [docs/setup/NATIVE-SETUP.md](docs/setup/NATIVE-SETUP.md).

**Windows.** Ubuntu 24.04 under WSL2, then the same install inside it. [docs/setup/NATIVE-SETUP.md](docs/setup/NATIVE-SETUP.md).

## Documentation

| Document | Covers |
|---|---|
| [docs/setup/NATIVE-SETUP.md](docs/setup/NATIVE-SETUP.md) | Install, environment, troubleshooting |
| [docs/simulation/HOW-IT-WORKS.md](docs/simulation/HOW-IT-WORKS.md) | Pipeline from your code to the rotors, commands, QoS, offboard, multi-vehicle |
| [docs/swarm/SWARM.md](docs/swarm/SWARM.md) | Multi-drone examples in `src/swarm_naive` and `src/swarm_comms` |
| [docs/development/NEW-PACKAGE.md](docs/development/NEW-PACKAGE.md) | Starting a package in Python or C++ |
| [docs/simulation/UNDER-THE-SCRIPTS.md](docs/simulation/UNDER-THE-SCRIPTS.md) | Stack without `drone-sim` or `drone-gcs`, mapped onto aircraft hardware |
| [docs/simulation/WORLDS.md](docs/simulation/WORLDS.md) | Flying in worlds other than the default, building a map |
| [docs/simulation/SIMULATION-FIDELITY.md](docs/simulation/SIMULATION-FIDELITY.md) | What the simulator reproduces and what it does not |

Three drones taking off, flying a ring and landing:

```bash
ros2 launch swarm_naive swarm.launch.py    # drones unaware of each other
ros2 launch swarm_comms swarm.launch.py    # same flight, with inter-drone communication
```

## Git workflow

- Branches: `main` (stable) ← `dev` (integration) ← `feature` (work in progress).
- Never commit to `main` without team review.
- Start each session from a new branch off `dev`: `feature-date-author`.
- Scope changes to the smallest functional unit.
- Revert to the last known safe state when the process breaks. Ask for help.

## Testing

[docs/setup/NATIVE-SETUP.md](docs/setup/NATIVE-SETUP.md) lists tested configurations.

Verified: Pop!_OS 24.04, NVIDIA GPU. If your setup is untested, run the smoke test and open an Issue:

```bash
./scripts/smoke-test.sh
```

Machines without a GPU use software rendering. This costs GUI framerate and camera sensor rate. Flight behavior is unchanged: PX4 runs in lockstep. 8 vehicles held RTF 0.999 on 4 CPU cores. See [docs/simulation/SIMULATION-FIDELITY.md](docs/simulation/SIMULATION-FIDELITY.md).

## Communication

Use GitHub Issues for bugs, package requests, and long-standing problems. Use GroupMe for urgent items.