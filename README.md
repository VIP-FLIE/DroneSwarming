# DroneSwarming

The central repository for the University of Delaware's VIP Drone team's Swarm
subteam.

The example packages `cpp_single_hop`, `py_single_square`, `swarm_naive` and
`swarm_comms`, and the docs that cover them, were written with AI assistance
and flown in the simulator before commit.

Use GitHub Issues for problems and questions, and for anything that must not
get buried. Use GroupMe when something needs immediate attention.

## The two ways to run

There are two ways to run ROS 2 Jazzy, Gazebo Harmonic and PX4. Both install
the same pinned versions and fly the same. Pick one.

**Dev container.** Any OS. You install Docker and VS Code, nothing else. Start
with [docs/setup/SETUP.md](docs/setup/SETUP.md), and check [docs/setup/COMPATIBILITY.md](docs/setup/COMPATIBILITY.md)
for your machine.

```bash
git clone git@github.com:VIP-FLIE/DroneSwarming.git && cd DroneSwarming
./scripts/doctor.sh --write      # Linux and MacOS
.\scripts\doctor.ps1 -Write      # Windows PowerShell
```

Then in VS Code: `F1`, `Dev Containers: Reopen in Container`. Gazebo runs on a
desktop inside the container. You view it either in your browser at
<http://localhost:6080> or in a VNC client on port 5901. VirtualGL renders on
your GPU if you have one.

**Native.** Ubuntu 24.04 and derivatives. Use it if you already run the ROS 2
stack natively, or you want Gazebo in a window on your own desktop and GPU.
Start with [docs/setup/NATIVE-SETUP.md](docs/setup/NATIVE-SETUP.md).

```bash
git clone git@github.com:VIP-FLIE/DroneSwarming.git && cd DroneSwarming
./scripts/native/install.sh
```

**The lab machine.** The box in Evans 153 runs the stack natively on its NVIDIA
card and takes SSH from anywhere, on any OS. You get a shell, a GPU, and a
Gazebo window you view in a browser or a VNC client. It is the way to run on a
Mac, or to run a long job off your own laptop. [docs/setup/LAB-SERVER.md](docs/setup/LAB-SERVER.md)
covers how it is set up and how you get an account. You do not need it to
develop.

## The docs

| Doc | Contains |
|---|---|
| [docs/setup/SETUP.md](docs/setup/SETUP.md) | Dev container installation and first flight |
| [docs/setup/NATIVE-SETUP.md](docs/setup/NATIVE-SETUP.md) | Native installation on Ubuntu 24.04 |
| [docs/setup/COMPATIBILITY.md](docs/setup/COMPATIBILITY.md) | The matrix of operating systems, architectures and GPUs, and what has been tested |
| [docs/setup/LAB-SERVER.md](docs/setup/LAB-SERVER.md) | The shared machine in Evans 153 |
| [docs/setup/CONTAINER-INTERNALS.md](docs/setup/CONTAINER-INTERNALS.md) | Container design notes |
| [docs/simulation/QUICKSTART.md](docs/simulation/QUICKSTART.md) | Running the simulator and writing your first node |
| [docs/simulation/HOW-IT-WORKS.md](docs/simulation/HOW-IT-WORKS.md) | The whole pipeline from your code down to the rotors, starting from one drone |
| [docs/simulation/UNDER-THE-SCRIPTS.md](docs/simulation/UNDER-THE-SCRIPTS.md) | The same stack run with the raw PX4 SITL command and `drone-gcs` instead of this repo's tools, and how each piece maps onto what is physically on the aircraft |
| [docs/simulation/WORLDS.md](docs/simulation/WORLDS.md) | Flying somewhere other than the blank grey plane, including building a map |
| [docs/simulation/SIMULATION-FIDELITY.md](docs/simulation/SIMULATION-FIDELITY.md) | What the simulation reproduces, what it does not, and the measurements |
| [docs/swarm/SWARM.md](docs/swarm/SWARM.md) | Several drones at once, and the two worked swarm examples in [src/swarm_naive](src/swarm_naive) and [src/swarm_comms](src/swarm_comms) |
| [docs/development/NEW-PACKAGE.md](docs/development/NEW-PACKAGE.md) | Starting a package of your own, in Python or C++ |

Package READMEs stay in the packages: [src/tutorial_package/README.md](src/tutorial_package/README.md),
[src/cpp_single_hop/README.md](src/cpp_single_hop/README.md),
[src/py_single_square/README.md](src/py_single_square/README.md),
[src/swarm_naive/README.md](src/swarm_naive/README.md),
[src/swarm_comms/README.md](src/swarm_comms/README.md).

Three drones taking off, flying a ring and landing, once the simulator is up:

```bash
ros2 launch swarm_naive swarm.launch.py    # no drone knows the others exist
ros2 launch swarm_comms swarm.launch.py    # the same flight, with them talking
```

## Git(Hub) etiquette

- Work on a `main <- dev <- feature` branch workflow. The most stable branch is
  `main`. The branch everybody pushes completed changes to and pulls new
  branches from is `dev`. Feature branches are for working on new features and
  merge back into `dev` once they are complete.
- Never commit to `main` unless the team has reviewed and green-lit the
  changes. `main` is the last version of the project that works.
- When starting a fresh working session, create a new branch from `dev`, and
  follow the feature-date-author naming convention. The name states the who,
  what and when of the branch. For example, `docs-sept6-dan`.
- Scope changes down to the smallest functional unit possible.
- If the process breaks, revert to the last known safe version of the codebase.
  Ask for help rather than leaving it broken.

## Testing

[docs/setup/COMPATIBILITY.md](docs/setup/COMPATIBILITY.md) holds the full matrix of
operating systems, architectures and GPUs, and what has actually been tested,
for both the dev container and a native install.

Verified so far: the dev container on Linux x86_64 with and without an NVIDIA
GPU, and headless over SSH; the native install on Pop!_OS 24.04 with an NVIDIA
GPU. Windows and macOS are implemented but untested on hardware. If your setup
is listed as untested, run the smoke test and open an Issue with the output:

```bash
./smoke-test.sh                     # dev container
./scripts/native/smoke-test.sh      # native
```

Machines without a GPU get software rendering. Software rendering costs GUI
framerate and camera sensor rate, not flight behavior. PX4 runs in lockstep,
and 8 vehicles held a real-time factor of 0.999 on 4 CPU cores. Details and
numbers are in [docs/simulation/SIMULATION-FIDELITY.md](docs/simulation/SIMULATION-FIDELITY.md).