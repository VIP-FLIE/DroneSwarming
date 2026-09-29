# DroneSwarming
The central repository for the University of Delaware's VIP Drone team's Swarm subteam.


## Welcome! A few brief 'getting started' points:
_For all of this, don't hesistate to message me (Dan Mihovch) if you run into any problems or have any questions.
There will be (if not already), more documents explaining everything covered here in greater detail, including resources to help you get started._

### Communication of problems and questions:
Here we will use GitHub Issues to communicate problems and questions. For anything that must not be forgotten or buried in messages, please use GitHub Issues. It will allow us to keep everything tidy in the repo and not lose important threads. Of course, if something needs immediate attention, just use the GroupMe. I am thinking things like package requests, bug reports, long-standing issues, etc. should go in the Issues.

### Development Workflow:
There are two ways to run ROS 2 Jazzy, Gazebo Harmonic and PX4. Both install the same pinned versions and fly the same. Pick one.

**Dev container.** Any OS. You install Docker and VS Code, nothing else. Start with [docs/SETUP.md](docs/SETUP.md), and check [COMPATIBILITY.md](COMPATIBILITY.md) for your machine.

```bash
git clone git@github.com:VIP-FLIE/DroneSwarming.git && cd DroneSwarming
./scripts/doctor.sh --write      # Linux and macOS
```

Then in VS Code: `F1`, `Dev Containers: Reopen in Container`. Gazebo runs on a desktop inside the container. You view it either in your browser at <http://localhost:6080> or in a VNC client on port 5901, whichever you prefer. Both are set up in [docs/SETUP.md](docs/SETUP.md). VirtualGL renders on your GPU if you have one.

**Native.** Ubuntu 24.04 and derivatives. Use it if you already run the ROS 2 stack natively, or you want Gazebo in a window on your own desktop and GPU. Start with [docs/NATIVE-SETUP.md](docs/NATIVE-SETUP.md).

```bash
git clone git@github.com:VIP-FLIE/DroneSwarming.git && cd DroneSwarming
./scripts/native/install.sh
```

Either way, once it runs, [docs/HOW-IT-WORKS.md](docs/HOW-IT-WORKS.md) explains the whole
pipeline from your code down to the rotors, starting from one drone.
[docs/SWARM.md](docs/SWARM.md) is the same thing for several drones, and covers the two
worked swarm examples in `src/swarm_naive` and `src/swarm_comms`.
[docs/NEW-PACKAGE.md](docs/NEW-PACKAGE.md) is how you start a package of your own, in Python or C++.
[docs/UNDER-THE-SCRIPTS.md](docs/UNDER-THE-SCRIPTS.md) runs the same stack without `drone-sim` or `drone-gcs`, and maps each piece onto what is physically on the aircraft.
[docs/WORLDS.md](docs/WORLDS.md) is how you fly somewhere other than the blank grey plane, including building a map of your own.

Three drones taking off, flying a ring and landing, once the simulator is up:

```bash
ros2 launch swarm_naive swarm.launch.py    # no drone knows the others exist
ros2 launch swarm_comms swarm.launch.py    # the same flight, with them talking
```

### Git(Hub) Etiquette:
This may seem pedantic, but trust that things can go real bad real quick without proper version control, and it isn't too hard to pick up some good habits. A few rules of thumb:
- We will operate on a `main <- dev <- feature` branch workflow. The most stable branch is main. The branch everybody should be pushing completed changes to and pulling new branches from is dev (development). Feature branches are for working on new features and should be merged back into dev once they are complete.
- NEVER commit to main, unless the team has reviewed and green-lit the changes. Main should be treated as 'the last version of the project that works'.
- When starting a fresh working session, always create a new branch from dev, and follow the feature-date-author naming convention. This just makes it easier to figure out the who, what, and when of the branch just by looking at a name in a list. For example, I am writing this doc on branch 'docs-sept6-dan'.
- Try to scope down your changes to the smallest functional unit possible. It is much easier to reason about a change that tackles one issue across a few files rather than sweeping changes across the entire codebase. This also comes with the practical upshot of changes _usually_ being small enough to be merged without conflicts.
- If someone messes this process up (you will, I will, we all will), best practice is to revert to the last known 'safe' version of the codebase. If you don't know how to get back to that point (also totally okay!), please ask for help ASAP. Git issues are an infinite rabbit hole of horror and despair, the closer to ground level we address them, the easier it is to get back on track.

## Testing:
See [COMPATIBILITY.md](COMPATIBILITY.md) for the full matrix of operating systems, architectures and GPUs, and what has actually been tested, for both the dev container and a native install.

Verified so far: the dev container on Linux x86_64 with and without an NVIDIA GPU, and headless over SSH; the native install on Pop!_OS 24.04 with an NVIDIA GPU. Windows and macOS are implemented but untested on hardware. If your setup is listed as untested, run the smoke test and open an Issue with the output:

```bash
./smoke-test.sh                     # dev container
./scripts/native/smoke-test.sh      # native
```

Machines without a GPU get software rendering. That costs GUI framerate and camera sensor rate, not flight behavior. PX4 runs in lockstep, and 8 vehicles held a real-time factor of 0.999 on 4 CPU cores. Details in [docs/SIMULATION-FIDELITY.md](docs/SIMULATION-FIDELITY.md).
