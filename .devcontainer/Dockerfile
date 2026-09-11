# syntax=docker/dockerfile:1
# =============================================================================
# UD Drone VIP - portable development image
#
# ONE image serves every supported host:
#   Windows 11 (Docker Desktop/WSL2)   x86_64
#   macOS Intel                        x86_64
#   macOS Apple Silicon                arm64   <- native, NOT emulated
#   Linux (X11 or Wayland)             x86_64 / arm64
#
# ---------------------------------------------------------------------------
# WHY THE BASE IMAGE IS `ros:jazzy-ros-base` AND NOT `osrf/ros:jazzy-desktop-full`
# ---------------------------------------------------------------------------
# osrf/ros:jazzy-desktop-full publishes a SINGLE-ARCH (amd64-only) manifest.
# Verify for yourself:
#     docker manifest inspect osrf/ros:jazzy-desktop-full   # one manifest, amd64
#     docker manifest inspect ros:jazzy-ros-base            # manifest LIST
#
# On an Apple Silicon Mac an amd64-only base forces qemu emulation of the whole
# stack. PX4 SITL plus Gazebo physics under emulation is not "slower", it is
# unusable. `ros:jazzy-ros-base` is a manifest list covering amd64 and arm64,
# and both `ros-jazzy-desktop-full` (packages.ros.org) and `gz-harmonic`
# (packages.osrfoundation.org) publish arm64 debs for noble, so the desktop
# tooling is layered on top by apt at build time instead.
#
# Every version below is pinned on purpose. Do not float them.
#
# THESE ARG DEFAULTS ARE THE ONLY PLACE VERSIONS ARE SET. Do not add a
# build.args block to devcontainer.json or compose: those values override
# these and the two silently drift apart.
#   ROS 2 Jazzy      LTS, supported to May 2029
#   Gazebo Harmonic  the version ROS 2 Jazzy officially pairs with
#   PX4 v1.16.1      pinned to the version this team has validated
#   px4_msgs         release/1.16 branch, matched to the PX4 version above
#   XRCE-DDS Agent   v2.4.3. PX4's docs say v2.4.2, but that version's superbuild
#                    pins Fast DDS branch 2.12.x, which eProsima has deleted, so it
#                    cannot build (upstream issue #370, still open). v2.4.3 pins
#                    Fast DDS 2.14.x and Fast CDR 2.2.x, both of which still exist,
#                    and stays on the 2.x client protocol that PX4 v1.16.1 speaks.
#                    Do NOT jump to v3.0.1: it pins Fast DDS 3.x and a v3.0.0 client,
#                    and agent 3.x is incompatible with clients older than 3.0.0.
#
# NOTE on Gazebo: PX4 docs state that on Ubuntu 24.04 the "latest installed
# release" of Gazebo is used by default, which could be Harmonic, Ionic, or
# Jetty. We install gz-harmonic explicitly so everyone gets the same one.
# =============================================================================

FROM ros:jazzy-ros-base

ARG PX4_VERSION=v1.16.1
ARG PX4_MSGS_BRANCH=release/1.16
ARG XRCE_AGENT_VERSION=v2.4.3
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# Set to 0 for a fast image that compiles PX4 on first container start instead.
# CI builds the published image with 1 so students never wait for a compile.
ARG PREBUILD_PX4=1

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------------
# 1. Base packages
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git curl wget lsb-release gnupg ca-certificates \
      build-essential cmake ninja-build ccache \
      python3-pip python3-venv python3-dev \
      libeigen3-dev libopencv-dev protobuf-compiler libprotobuf-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      cppzmq-dev libzmq3-dev \
      tmux vim nano less htop iputils-ping net-tools psmisc procps \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. ROS 2 desktop tooling (rviz2, rqt, demos) via apt.
#    The base image is ros-base; desktop is layered here so the image stays
#    multi-arch. See the header note on why we cannot use osrf/*-desktop-full.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. Gazebo Harmonic, pinned
# ---------------------------------------------------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends gz-harmonic \
 && rm -rf /var/lib/apt/lists/*

# ROS 2 <-> Gazebo bridge. On Jazzy this package is built against Harmonic.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 4. Self-contained display stack.
#
# This is what makes the container work on macOS and on Windows without
# Docker Desktop, XQuartz, VcXsrv or WSLg on the host.
#
# The container can render into its OWN X server (Xvfb) and publish that
# desktop over VNC and over HTTP (noVNC), so the GUI is reachable from any
# browser at http://localhost:6080 with ZERO host-side setup.
#
# Xvfb + x11vnc is used rather than TigerVNC's Xvnc on purpose: Xvfb is
# verified to expose Mesa llvmpipe with OpenGL 4.5, which is what Gazebo's
# ogre2 render engine needs (it requires 3.3+). TigerVNC's GLX support is
# more variable across versions.
#
# mesa-utils / libgl1-mesa-dri provide the llvmpipe software rasteriser that
# makes this work on machines with no usable GPU (every Mac, and any Windows
# box without an NVIDIA card).
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      xvfb x11vnc novnc websockify openbox xterm \
      x11-utils x11-xserver-utils xauth dbus-x11 \
      libgl1-mesa-dri mesa-utils \
      libxcb-cursor0 libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/* \
 && ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

# ---------------------------------------------------------------------------
# 5. Micro XRCE-DDS Agent (the PX4 <-> ROS 2 bridge)
# ---------------------------------------------------------------------------
RUN git clone -b ${XRCE_AGENT_VERSION} --depth 1 \
      https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
 && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/xrce/build -j"$(nproc)" \
 && cmake --install /tmp/xrce/build \
 && ldconfig /usr/local/lib/ \
 && rm -rf /tmp/xrce

# ---------------------------------------------------------------------------
# 6. Non-root user, so files you create in the repo are owned by you.
#
# Ubuntu 24.04 images ship an `ubuntu` account already sitting on UID 1000,
# so we rename it rather than fail on a duplicate uid. On Linux hosts VS Code
# additionally rewrites this uid to match yours (updateRemoteUserUID).
# ---------------------------------------------------------------------------
RUN if getent group ${USER_GID} >/dev/null; then groupmod -n ${USERNAME} "$(getent group ${USER_GID} | cut -d: -f1)"; else groupadd --gid ${USER_GID} ${USERNAME}; fi \
 && if getent passwd ${USER_UID} >/dev/null; then usermod -l ${USERNAME} -d /home/${USERNAME} -m "$(getent passwd ${USER_UID} | cut -d: -f1)"; else useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash; fi \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && for g in dialout video render plugdev; do \
        getent group "$g" >/dev/null 2>&1 || groupadd -r "$g"; \
        usermod -aG "$g" ${USERNAME}; \
    done

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# PX4's Tools/setup/ubuntu.sh runs `sudo usermod -aG dialout $USER`.
# Docker does not set USER, and an empty value makes that command fail.
ENV USER=${USERNAME}
ENV HOME=/home/${USERNAME}

# PX4's build scripts need these Python packages at these versions
RUN pip3 install --break-system-packages --user -U \
      empy==3.3.4 pyros-genmsg setuptools packaging jinja2 jsonschema \
      pyserial numpy toml pyyaml kconfiglib future lxml

ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# ---------------------------------------------------------------------------
# 7. PX4 source, pinned.
#
# Build output is redirected to a file rather than the build log on purpose.
# PX4's compile emits far more than BuildKit's 1 MiB per-step log limit, and
# hitting that limit terminates the build with exit code 7 while hiding the
# output that would explain why (docker/buildx issue #484). Redirecting and
# tailing only on failure keeps us well under the cap AND still surfaces the
# error when there is one.
# ---------------------------------------------------------------------------
# --shallow-submodules is NOT cosmetic. Without it, `--recursive` clones every
# submodule at FULL history, which alone accounts for ~1.1 GB of .git in the
# final image. Students pull this over home internet; it matters.
RUN git clone --depth 1 -b ${PX4_VERSION} --recursive --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git /home/${USERNAME}/PX4-Autopilot \
 && cd /home/${USERNAME}/PX4-Autopilot \
 && bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
 && sudo rm -rf /var/lib/apt/lists/* \
 \
 && : "Drop the NuttX submodules. They are the firmware toolchain for real" \
 && : "flight-controller hardware and are never used by SITL, and they cost" \
 && : "~250 MB. This MUST happen before the build, not after:" \
 && : "src/lib/version/px_update_git_header.py guards its NuttX block with" \
 && : "os.path.exists('platforms/nuttx/NuttX/nuttx/.git'), so an ABSENT tree" \
 && : "is skipped cleanly, whereas a present-but-tagless one (what" \
 && : "--shallow-submodules produces) makes it crash with IndexError." \
 && rm -rf platforms/nuttx/NuttX/nuttx platforms/nuttx/NuttX/apps

ENV PX4_DIR=/home/${USERNAME}/PX4-Autopilot

RUN if [ "${PREBUILD_PX4}" = "1" ]; then \
        echo ">>> Compiling PX4 SITL (output captured to /tmp/px4-build.log)"; \
        cd "${PX4_DIR}" && \
        { make px4_sitl_default -j"$(nproc)" > /tmp/px4-build.log 2>&1 \
          || { echo "PX4 BUILD FAILED - last 120 lines:"; tail -120 /tmp/px4-build.log; exit 1; }; } && \
        echo ">>> PX4 SITL built" && rm -f /tmp/px4-build.log; \
    else \
        echo ">>> Skipping PX4 prebuild (PREBUILD_PX4=0)"; \
    fi

# ---------------------------------------------------------------------------
# 8. ROS 2 workspace: px4_msgs
# ---------------------------------------------------------------------------
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
 && git clone --depth 1 -b ${PX4_MSGS_BRANCH} \
      https://github.com/PX4/px4_msgs.git /home/${USERNAME}/ros2_ws/src/px4_msgs

RUN if [ "${PREBUILD_PX4}" = "1" ]; then \
        source /opt/ros/jazzy/setup.bash && \
        cd /home/${USERNAME}/ros2_ws && \
        { colcon build --symlink-install > /tmp/ws-build.log 2>&1 \
          || { echo "px4_msgs BUILD FAILED - last 120 lines:"; tail -120 /tmp/ws-build.log; exit 1; }; } && \
        echo ">>> px4_msgs built" && rm -f /tmp/ws-build.log; \
    fi

# Record what was actually baked in, so smoke-test.sh can detect version drift
# between this Dockerfile and any build args a config might pass.
ENV BUILT_PX4_VERSION=${PX4_VERSION}
ENV BUILT_PX4_MSGS_BRANCH=${PX4_MSGS_BRANCH}
ENV BUILT_XRCE_AGENT_VERSION=${XRCE_AGENT_VERSION}

# ---------------------------------------------------------------------------
# 9. Display + shell setup
# ---------------------------------------------------------------------------
COPY --chown=${USERNAME}:${USERNAME} scripts/ /opt/drone-vip/scripts/
RUN sudo chmod +x /opt/drone-vip/scripts/*.sh

# Qt/X scratch dir. XDG_RUNTIME_DIR points here.
RUN mkdir -p /tmp/runtime-ros && chmod 0700 /tmp/runtime-ros

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
 && echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc \
 && printf '%s\n' \
      '# Source your own colcon overlay. Globbed because the mount path is' \
      '# /workspaces/<repo-name>, and the image does not know the repo name.' \
      'for _ws in /workspaces/*/install/setup.bash; do' \
      '    [ -f "$_ws" ] && source "$_ws"' \
      'done; unset _ws' >> ~/.bashrc \
 && echo 'export PX4_DIR=~/PX4-Autopilot' >> ~/.bashrc \
 && echo 'export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds' >> ~/.bashrc \
 && printf '%s\n' \
      '# Pick DISPLAY for THIS shell. Written by start-display.sh at container' \
      '# start. Sourced per-shell so the same image works whether the GUI is' \
      '# going to the host X server or to the built-in noVNC desktop.' \
      '[ -f /tmp/drone-display.env ] && source /tmp/drone-display.env' >> ~/.bashrc

CMD ["/bin/bash"]
