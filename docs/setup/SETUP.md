# Setup

ROS 2 Jazzy, Gazebo Harmonic and PX4 install directly on Ubuntu 24.04. On
macOS and Windows, run Ubuntu 24.04 in a virtual machine or WSL2, then follow
[NATIVE-SETUP.md](NATIVE-SETUP.md) inside it.

Requirements: 15 GB disk and 8 GB RAM. The first build takes 20 to 40 minutes.

## macOS

UTM runs an Ubuntu 24.04 arm64 virtual machine on Apple Silicon.

1. Install UTM from <https://mac.getutm.app/>.
2. Download the Ubuntu 24.04 arm64 image from <https://ubuntu.com/download>.
3. Create a VM in UTM from that image. Give it 8 GB RAM or more and 40 GB disk.
4. Start the VM and install Ubuntu.
5. Inside the VM, follow [NATIVE-SETUP.md](NATIVE-SETUP.md).

Gazebo opens a window on the VM's desktop. macOS does not pass a GPU to a VM,
so rendering is software (`llvmpipe`). Flight behavior is unaffected. Camera
sensor rate is lower. See [SIMULATION-FIDELITY.md](../simulation/SIMULATION-FIDELITY.md).

## Windows

Windows 11 runs Ubuntu 24.04 under WSL2. Install the Ubuntu 24.04 distribution,
then follow [NATIVE-SETUP.md](NATIVE-SETUP.md) inside it.

This path is untested. Open an Issue with `./scripts/native/smoke-test.sh`
output if you run it.
