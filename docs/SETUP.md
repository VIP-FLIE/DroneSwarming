# Getting your machine set up

_If you get stuck anywhere in this doc, message me (Dan Mihovch) or open an Issue. If you hit a snag there is a solid chance somebody else will hit the same one, so it is genuinely worth writing down rather than suffering quietly._

The whole idea here is that you should not have to install ROS, Gazebo, or PX4 on your laptop at all. All of that lives inside a container, and it is the same container everybody else on the team is running. So if it works on my machine it works on yours, and when something does break, at least we are all broken in the same place.

Two things to know going in. The first time you open the project it has to build the container image, and that takes somewhere in the 20 to 40 minute range because it is compiling PX4 from source. Kick it off and go do something else. Every time after that it opens in a few seconds. You will also want around 15 GB of free disk and at least 8 GB of RAM, 16 if you have it.

## Step 1: Install Docker and VS Code:
That is the entire dependency list. Docker, and VS Code with the Dev Containers extension. The details differ a bit per OS, so find yourself below.

### Windows 11:
Install [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/) and leave the WSL 2 backend option checked when it asks. It is the default, and it is noticeably faster than the Hyper-V option for what we are doing. If the installer complains that WSL is missing, open PowerShell as Administrator, run `wsl --install`, reboot, and try the installer again. If it complains about virtualization instead, that one is a BIOS setting, usually called Intel VT-x or AMD-V or SVM depending on whose machine you have.

Then install [VS Code](https://code.visualstudio.com/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers), and that is you done. You do NOT need VcXsrv, Xming, or any other X server. If you have tried to run ROS on Windows before and remember fighting with one of those, you can forget it exists for this project.

If you have an Nvidia card, update to a recent Game Ready or Studio driver and turn on GPU support in Docker Desktop under Settings > Resources. One thing to avoid: do not install a Linux Nvidia driver inside WSL, that will actually break it. This is worth doing if you think you will touch cameras at any point, and [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md) explains why it matters more than you would guess.

### MacOS (Intel or Apple Silicon):
Install [Docker Desktop](https://docs.docker.com/desktop/install/mac-install/), making sure you grab the build for your chip. Then open Docker Desktop, go to Settings > Resources, and give it at least 8 GB of memory. I am calling that out specifically because a starved Docker is far and away the most common reason a build looks like it has hung on a Mac, when really it is just crawling.

Then install [VS Code](https://code.visualstudio.com/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers). You do NOT need XQuartz, which is the thing every other guide on the internet will tell you to install.

Two Mac specific notes. If you are on Apple Silicon, the image gets built natively for arm64, so you are not sitting under emulation and paying for it. And Macs cannot hand a GPU to Docker at all, which is a platform limitation rather than something we can configure around. That is fine for basically everything except cameras, and again, [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md) has the details.

### Linux:
Install [Docker Engine](https://docs.docker.com/engine/install/), then actually do the [post-install steps](https://docs.docker.com/engine/install/linux-postinstall/) so you can run `docker` without `sudo`. You have to log out and back in for that to take effect, which is easy to forget and then wonder why nothing works. Then install [VS Code](https://code.visualstudio.com/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

If you have an Nvidia card, install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). That is the piece that lets Docker see your GPU, and you will want it if you are doing anything with cameras.

## Step 2: Clone the repo and run the doctor:
```bash
git clone https://github.com/<org>/DroneSwarming.git
cd DroneSwarming
```

There is a script that looks at your machine and tells you which container config to open, so you do not have to reason about it yourself:

```bash
./scripts/doctor.sh --write
```

On Windows use PowerShell:

```powershell
.\scripts\doctor.ps1 -Write
```

If PowerShell refuses to run it and complains about scripts being disabled, use `powershell -ExecutionPolicy Bypass -File .\scripts\doctor.ps1 -Write` instead.

It will point you at one of three configs. 'Drone VIP - Universal (browser GUI)' is the default and works on anything, 'Drone VIP - Linux (native X11 + GPU)' is the faster option if you are on Linux with a desktop, and 'Drone VIP - Windows (WSLg native display)' is an optional faster path on Windows. If you skip the doctor entirely nothing bad happens, you just end up on the Universal one, which is the safe choice anyway.

## Step 3: Open the container:
Open the repo folder in VS Code, hit `F1`, and run 'Dev Containers: Reopen in Container'. Pick whichever config the doctor named.

This is the 20 to 40 minute part. It is compiling PX4, it has not frozen, let it run. If the team has already published the prebuilt image, you can skip almost all of that by putting `DRONE_IMAGE=ghcr.io/<org>/DroneSwarming-dev:jazzy` into `.devcontainer/.env` before this step, and it will pull a finished image instead of building one.

Once it finishes, check that the whole stack is actually working:

```bash
./smoke-test.sh
```

You want to see `Stack is good.` at the bottom. If something failed it will tell you which piece and what to do about it, so paste that output into an Issue rather than trying to decode it alone.

## Step 4: Find your Gazebo window:
This trips people up, because where the window shows up depends on which config you opened. On the Universal (browser GUI) config, the window is served to your browser, so go to <http://localhost:6080> and it is there. VS Code will usually offer to forward that port on its own, and if it does not, the Ports tab in VS Code will do it. On the Linux native or Windows WSLg configs it just opens on your actual desktop like a normal application.

If you are not sure which one you ended up on, run `echo $DRONE_DISPLAY_ACTIVE` in the container terminal and it will say either `native` or `vnc`.

## Step 5: Actually fly something:
Open three terminals in VS Code with `Terminal > New Terminal`. You are already inside the container, so there is nothing to activate or source.

Terminal 1, the bridge between PX4 and ROS 2:

```bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 2, the simulator itself:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

That drops you at a `pxh>` prompt and opens Gazebo.

Terminal 3, your side of things:

```bash
ros2 topic list | grep fmu
```

A long list of `/fmu/out/...` topics means everything is talking to everything else. If that list comes back empty, the agent in Terminal 1 is not connected, so start there.

Then from the `pxh>` prompt in Terminal 2:

```
pxh> commander takeoff
pxh> commander land
```

That is the whole loop. [QUICKSTART.md](../QUICKSTART.md) picks up from here and goes into writing your first node and what the topics actually mean.

## When things go wrong:
A few things that come up often enough to be worth listing:

- Gazebo's 3D view is just blank grey. This one looks broken but is not. PX4's default world is an untextured grey ground under a grey sky, and the camera starts far enough back that an X500 is only a few pixels. Scroll to zoom in and you will find it. If the Entity Tree panel on the right lists `x500_0`, the drone spawned and you are fine.
- Nothing at localhost:6080. Check the Ports tab in VS Code first. If that looks right, run `bash /opt/drone-vip/scripts/start-display.sh` inside the container and watch what it says.
- Gazebo is choppy. You are on software rendering, which is expected if there is no GPU in the picture. It does not affect how the drone actually flies. For anything heavy, run `HEADLESS=1 make px4_sitl gz_x500` and skip drawing entirely.
- The build seems frozen, on Mac or Windows. Almost always Docker memory. Docker Desktop, Settings, Resources, 8 GB or more.
- The build dies and your machine starts swapping. Too many parallel jobs for your RAM. Run `JOBS=2 bash .devcontainer/scripts/build-workspace.sh` instead.
- `cannot open display` on Linux. That is X access control. Re-run `./scripts/doctor.sh --write` and then rebuild the container.
- The container will not start and the error mentions nvidia. You have `DOCKER_RUNTIME=nvidia` sitting in `.devcontainer/.env` on a machine without the NVIDIA Container Toolkit. Delete that line.
- Smoke test says the image is stale. Somebody changed a version pin, so run 'Dev Containers: Rebuild Container'.

Still stuck after that, open an Issue with the output of `./smoke-test.sh` in it. That output tells us almost everything we would otherwise have to ask you for.

One last thing worth reading before you trust a result out of this, especially anything involving cameras or timing that has to work the same way on a real drone: [SIMULATION-FIDELITY.md](SIMULATION-FIDELITY.md). It lays out what the simulation gets right, what it does not, and the numbers behind both. If you want the reasoning behind how the container is built and every knob you can turn, that is in [.devcontainer/README.md](../.devcontainer/README.md).
