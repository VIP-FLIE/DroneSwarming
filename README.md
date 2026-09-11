# DroneSwarming
The central repository for the University of Delaware's VIP Drone team's Swarm subteam.


## Welcome! A few brief 'getting started' points:
_For all of this, don't hesistate to message me (Dan Mihovch) if you run into any problems or have any questions.
There will be (if not already), more documents explaining everything covered here in greater detail, including resources to help you get started._

### Communication of problems and questions:
Here we will use GitHub Issues to communicate problems and questions. For anything that must not be forgotten or buried in messages, please use GitHub Issues. It will allow us to keep everything tidy in the repo and not lose important threads. Of course, if something needs immediate attention, just use the GroupMe. I am thinking things like package requests, bug reports, long-standing issues, etc. should go in the Issues.

### Development Workflow:
The robotics stack we use (ROS 2 Jazzy, Gazebo Harmonic, PX4) is native to Ubuntu 24.04, which used to mean everybody had to get to a Linux machine one way or another. That is no longer the case. The entire environment now ships in this repo as a dev container, so you develop on your own laptop, whatever it happens to run, and everybody ends up with an identical toolchain.

The only two things you install are Docker and VS Code. No ROS, no Gazebo, no PX4, and on Windows and Mac no X server either. [docs/SETUP.md](docs/SETUP.md) walks through all of it per operating system, and that is where you should start.

The short version of it:

```bash
git clone <this repo> && cd DroneSwarming
./scripts/doctor.sh --write      # Linux / MacOS
.\scripts\doctor.ps1 -Write      # Windows PowerShell
```

That script looks at your machine and tells you which container config to open, so you do not have to work it out yourself. Then in VS Code, hit `F1` and run 'Dev Containers: Reopen in Container'.

On Windows and Mac the Gazebo window gets served to your browser at <http://localhost:6080>, which sounds strange but is by far the least painful way to do this cross platform. On Linux it draws on your actual desktop with the GPU, same as you would expect.

The lab machine in Evans 153 is still there and still worth using for long runs and for anything involving real hardware. It is just no longer something you need before you can get started.

### Git(Hub) Etiquette:
This may seem pedantic, but trust that things can go real bad real quick without proper version control, and it isn't too hard to pick up some good habits. A few rules of thumb:
- We will operate on a `main <- dev <- feature` branch workflow. The most stable branch is main. The branch everybody should be pushing completed changes to and pulling new branches from is dev (development). Feature branches are for working on new features and should be merged back into dev once they are complete.
- NEVER commit to main, unless the team has reviewed and green-lit the changes. Main should be treated as 'the last version of the project that works'.
- When starting a fresh working session, always create a new branch from dev, and follow the feature-date-author naming convention. This just makes it easier to figure out the who, what, and when of the branch just by looking at a name in a list. For example, I am writing this doc on branch 'docs-sept6-dan'.
- Try to scope down your changes to the smallest functional unit possible. It is much easier to reason about a change that tackles one issue across a few files rather than sweeping changes across the entire codebase. This also comes with the practical upshot of changes _usually_ being small enough to be merged without conflicts.
- If someone messes this process up (you will, I will, we all will), best practice is to revert to the last known 'safe' version of the codebase. If you don't know how to get back to that point (also totally okay!), please ask for help ASAP. Git issues are an infinite rabbit hole of horror and despair, the closer to ground level we address them, the easier it is to get back on track.

## Dev Container Testing:
Where each platform stands. 'Verified' means somebody actually built it and ran `./smoke-test.sh` on that setup, not that it ought to work in theory.

- Linux, Wayland session, with Nvidia GPU: 'verified', native X11 through XWayland, hardware accelerated
- Linux, any session, browser GUI: 'verified', software rendering
- Linux, Xorg session: same code path as the Wayland one, not separately run
- Headless Linux over SSH: 'verified', browser GUI, no host display needed at all
- MacOS Mx/ARM: not tested on hardware yet, image builds native arm64 (not emulated)
- MacOS x86 Intel: not tested on hardware yet
- Windows 11 w/ Nvidia GPU: not tested on hardware yet
- Windows 11 w/o Nvidia GPU: not tested on hardware yet

If you are on a Mac or a Windows machine, running `./smoke-test.sh` and pasting the output into an Issue is genuinely the single most useful thing you can do for this project right now. The browser GUI path those machines use is the same one that is verified on Linux and it deliberately has no host dependencies, but nobody has actually sat in front of one of those laptops yet, and I would rather know than assume.

### A note on machines without a GPU:
Worth heading this off because it comes up. Not having a GPU costs you frame rate in the Gazebo window, and it does not cost you anything in how the drone actually flies. PX4 SITL runs in lockstep, so a slower machine takes longer in real seconds but simulates identical physics, and no airframe or sensor or physics setting was changed to make any of this run better on weak hardware.

For scale, eight PX4 vehicles held a real time factor of 0.999 even with the container limited to four CPU cores.

The one real exception is cameras. A camera set to 30 Hz only actually delivers about 11.5 Hz under software rendering, versus about 28.5 Hz with an Nvidia GPU, so anything vision based needs to be checked on a machine with a GPU before you trust it. [docs/SIMULATION-FIDELITY.md](docs/SIMULATION-FIDELITY.md) goes through all of this properly, including what SITL will never get right no matter what hardware you throw at it.
