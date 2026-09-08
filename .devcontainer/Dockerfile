# UD Drone VIP development container
# Base: ROS 2 Jazzy (Ubuntu 24.04) desktop, maintained by Open Robotics Foundation
#
# Every version below is pinned on purpose. Do not float them.
#
# THESE ARG DEFAULTS ARE THE ONLY PLACE VERSIONS ARE SET. Do not add a
# build.args block to devcontainer.json: those values override these and
# the two silently drift apart.
#   ROS 2 Jazzy      LTS, supported to May 2029
#   Gazebo Harmonic  the version ROS 2 Jazzy officially pairs with
#   PX4 v1.16.1      latest stable release
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

FROM osrf/ros:jazzy-desktop-full

ARG PX4_VERSION=v1.16.1
ARG PX4_MSGS_BRANCH=release/1.16
ARG XRCE_AGENT_VERSION=v2.4.3
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

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
      tmux vim nano less htop iputils-ping net-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Gazebo Harmonic, pinned
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
      ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. Micro XRCE-DDS Agent (the PX4 <-> ROS 2 bridge)
# ---------------------------------------------------------------------------
RUN git clone -b ${XRCE_AGENT_VERSION} --depth 1 \
      https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
 && cmake -S /tmp/xrce -B /tmp/xrce/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/xrce/build -j"$(nproc)" \
 && cmake --install /tmp/xrce/build \
 && ldconfig /usr/local/lib/ \
 && rm -rf /tmp/xrce

# ---------------------------------------------------------------------------
# 4. Non-root user, so files you create in the repo are owned by you
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

# Qt writes here. Pointed at by XDG_RUNTIME_DIR in devcontainer.json.
RUN mkdir -p /tmp/runtime-ros && chmod 0700 /tmp/runtime-ros

# ---------------------------------------------------------------------------
# 5. PX4 source, pinned. NOT compiled here, on purpose.
#
# Compiling PX4 inside the image is a trap. The build emits far more than
# BuildKit's 1 MiB per-step log limit, and hitting that limit terminates the
# build with exit code 7 while hiding the output that would explain why
# (docker/buildx issue #484). A failure also throws away the whole image.
#
# Instead the source is cloned here (cached, fast) and compiled on first
# container start by .devcontainer/build-workspace.sh, where you get live
# progress, no log cap, and a retry that does not require an image rebuild.
# ---------------------------------------------------------------------------
RUN git clone --depth 1 -b ${PX4_VERSION} --recursive \
      https://github.com/PX4/PX4-Autopilot.git /home/${USERNAME}/PX4-Autopilot \
 && cd /home/${USERNAME}/PX4-Autopilot \
 && bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
 && sudo rm -rf /var/lib/apt/lists/*

ENV PX4_DIR=/home/${USERNAME}/PX4-Autopilot

# Record what was actually baked in, so smoke-test.sh can detect version drift
# between this Dockerfile and any build args a config might pass.
ENV BUILT_PX4_VERSION=${PX4_VERSION}
ENV BUILT_PX4_MSGS_BRANCH=${PX4_MSGS_BRANCH}
ENV BUILT_XRCE_AGENT_VERSION=${XRCE_AGENT_VERSION}

# ---------------------------------------------------------------------------
# 6. ROS 2 workspace source. Also compiled at first start, same reasoning.
# ---------------------------------------------------------------------------
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
 && git clone --depth 1 -b ${PX4_MSGS_BRANCH} \
      https://github.com/PX4/px4_msgs.git /home/${USERNAME}/ros2_ws/src/px4_msgs

# ---------------------------------------------------------------------------
# 7. Shell setup
# ---------------------------------------------------------------------------
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
 && echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc \
 && printf '%s\n' \
      '# Source your own colcon overlay. Globbed because the mount path is' \
      '# /workspaces/<repo-name>, and the image does not know the repo name.' \
      'for _ws in /workspaces/*/install/setup.bash; do' \
      '    [ -f "$_ws" ] && source "$_ws"' \
      'done; unset _ws' >> ~/.bashrc \
 && echo 'export PX4_DIR=~/PX4-Autopilot' >> ~/.bashrc \
 && echo 'export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds' >> ~/.bashrc

CMD ["/bin/bash"]
