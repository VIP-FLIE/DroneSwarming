# Compatibility

What runs where. Setup is in [docs/NATIVE-SETUP.md](docs/NATIVE-SETUP.md) for
Linux, and [docs/SETUP.md](docs/SETUP.md) for macOS and Windows.

**Status meanings**

| | |
|---|---|
| Verified | Built and run on that setup, the smoke test passed |
| Untested | Should work, nobody has run it yet. Please test and open an Issue |
| No | Does not work |

## Operating systems

ROS 2 Jazzy and Gazebo Harmonic publish packages for Ubuntu 24.04 (noble)
only, so the install needs noble or a distro built on it.

| Host OS | Arch | Status |
|---|---|---|
| Pop!_OS 24.04 | x86_64 | Verified, NVIDIA RTX 3070 |
| Ubuntu 24.04 | x86_64 | Untested. Same noble base as the row above |
| Linux Mint 22, elementary OS 8, Zorin OS 18 | x86_64 | Untested. Same noble base |
| Ubuntu 24.04 | arm64 | Untested |
| Ubuntu 22.04 or older, Debian, Fedora, Arch | any | No |
| macOS, Ubuntu 24.04 in a UTM VM | arm64 | Untested |
| Windows 11, Ubuntu 24.04 under WSL2 | x86_64 | Untested |

## GPU

Rendering goes through your own GL driver.

| GPU | Rendering | Status |
|---|---|---|
| NVIDIA, Linux | Hardware | Verified on RTX 3070 |
| AMD, Linux | Hardware | Untested |
| Intel integrated, Linux | Hardware | Untested |
| None | Software | Verified |
| Any GPU in a UTM VM | Software | macOS does not pass a GPU to a VM |

Run `./scripts/native/smoke-test.sh` and read section 6. It prints the renderer
it actually got. `llvmpipe` means software.

## What software rendering costs

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

## Adding a row

If your setup is Untested, run `./scripts/native/smoke-test.sh` and open an
Issue with the output and your OS, arch and GPU.
