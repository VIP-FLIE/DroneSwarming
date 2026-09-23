# Compatibility

What runs where. Setup is in [docs/SETUP.md](docs/SETUP.md) for the dev
container, [docs/NATIVE-SETUP.md](docs/NATIVE-SETUP.md) for a native install,
and [docs/LAB-SERVER.md](docs/LAB-SERVER.md) for the shared machine in Evans
153.

**Status meanings**

| | |
|---|---|
| Verified | Built and run on that setup, the smoke test passed |
| Untested | Should work, nobody has run it yet. Please test and open an Issue |
| No | Does not work |

## Operating systems, dev container

You need Docker and VS Code. Nothing else, on any OS.

| Host OS | Arch | Image arch | Status |
|---|---|---|---|
| Ubuntu 24.04 | x86_64 | amd64 | Verified |
| Pop!_OS 24.04 | x86_64 | amd64 | Verified |
| Other Linux (Debian, Fedora, Arch) | x86_64 | amd64 | Untested. Only needs Docker, the container supplies Ubuntu 24.04 |
| Linux, headless or over SSH | x86_64 | amd64 | Verified |
| Linux | arm64 | arm64 | Untested |
| Windows 11 | x86_64 | amd64 | Untested |
| Windows 10, build 19041+ | x86_64 | amd64 | Untested. Same path as Windows 11 |
| Windows 10, older than 19041 | x86_64 | n/a | No. Docker Desktop needs WSL2 |
| Windows on ARM | arm64 | arm64 | Untested. Docker Desktop ARM support is preview |
| macOS, Apple Silicon | arm64 | arm64 | Untested. Native arm64, not emulated |
| macOS, Intel | x86_64 | amd64 | Untested |

Your host distro does not matter. ROS 2 Jazzy, Gazebo Harmonic and PX4 all live
in the container on Ubuntu 24.04.

Windows 10 and 11 follow the same path. You do not need WSLg, XQuartz, VcXsrv,
Xming, or any other X server.

## Operating systems, native

ROS 2 Jazzy and Gazebo Harmonic publish packages for Ubuntu 24.04 (noble)
only, so a native install needs noble or a distro built on it.

| Host OS | Arch | Status |
|---|---|---|
| Pop!_OS 24.04 | x86_64 | Verified, NVIDIA RTX 3070 |
| Ubuntu 24.04 | x86_64 | Untested. Same noble base as the row above |
| Linux Mint 22, elementary OS 8, Zorin OS 18 | x86_64 | Untested. Same noble base |
| Ubuntu 24.04 | arm64 | Untested |
| Ubuntu 22.04 or older, Debian, Fedora, Arch | any | No. Use the dev container |
| Windows, macOS | any | No. Use the dev container |

Native rendering goes through your own GL driver, so NVIDIA, AMD and Intel
graphics are all hardware accelerated, and none of them need a container
toolkit. This is the one place native beats the container: in the container
only NVIDIA can be passed through, and everything else falls back to software.

Verify a native machine with `./scripts/native/smoke-test.sh`.

## The shared lab machine

One native install on one Ubuntu 24.04 box with an NVIDIA card, that several
people SSH into at once. The stack is the native one above, unchanged and at
the same pins. What is added is per-user isolation and a way to see the GUI
from a machine that is not it.

| Your machine | Needs | Status |
|---|---|---|
| macOS, Apple Silicon or Intel | An SSH client and a browser | Untested |
| Windows 10 or 11 | An SSH client and a browser | Untested |
| Linux, any distro | An SSH client and a browser | Untested |
| iPad, Chromebook | A browser and an SSH client | Untested, should work |

Your own OS stops mattering: nothing of the stack runs on it. ROS 2, Gazebo and
PX4 all run on the lab machine, it renders on its own GPU, and you get pixels.
A TurboVNC Viewer is faster than the browser, and both show the same desktop.

Up to ten people can be assigned slots, though the machine runs out of cores
long before it runs out of slots. Setup, accounts and the slot table are in
[docs/LAB-SERVER.md](docs/LAB-SERVER.md).

## GPU, dev container

You view the GUI in a browser or a VNC client, and
[VirtualGL](https://virtualgl.org) renders it on the GPU when there is one.
Both viewing options get the same rendering.

| GPU | Needs | Rendering | Status |
|---|---|---|---|
| NVIDIA, Linux | [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) | Hardware | Verified on RTX 3070 |
| NVIDIA, Windows | Recent Game Ready or Studio driver, GPU enabled in Docker Desktop | Hardware | Untested |
| NVIDIA, no Container Toolkit | | Software | Verified |
| AMD | | Software | `/dev/dri` is not passed into the container |
| Intel integrated | | Software | Same |
| Apple Silicon or Intel Mac | | Software | macOS cannot pass a GPU to Docker at all |
| None | | Software | Verified |

Run `./smoke-test.sh` and read section 5. It prints the renderer it actually
got. `llvmpipe` means software.

## What software rendering costs

Measured in-container:

| | Software | NVIDIA GPU |
|---|---|---|
| Flight dynamics | identical | identical |
| RTF, 1 vehicle, GUI open | 0.99 | 0.99 |
| Gazebo GUI framerate | low | normal |
| Camera at 30 Hz configured | 11.5 Hz | 28.5 Hz |

Headless multi-vehicle does not render, so rendering is irrelevant to it:
8 vehicles held RTF 0.999, including when limited to 4 CPU cores.

PX4 runs in lockstep, so physics does not change with your hardware. Cameras
are the exception. Anything vision-based needs a GPU. See
[docs/SIMULATION-FIDELITY.md](docs/SIMULATION-FIDELITY.md).

## Requirements

| | |
|---|---|
| Disk | 15 GB |
| RAM | 8 GB minimum, 16 GB comfortable |
| First build | 20 to 40 minutes, once |

On macOS and Windows, set Docker Desktop to 8 GB or more under
Settings > Resources. Too little RAM looks like a hung build.

## Adding a row

If your setup is Untested, run `./smoke-test.sh` and open an Issue with the
output and your OS, arch and GPU. That is the most useful thing you can
contribute right now.
