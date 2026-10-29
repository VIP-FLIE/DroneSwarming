ARG PX4_BASE_IMAGE
FROM ${PX4_BASE_IMAGE}

ARG UBUNTU_VERSION
ARG ROS_DISTRO
ARG CC=gcc
ARG CXX=g++
ARG PX4_VERSION
ARG PX4_MSGS_VERSION
ARG RUFF_VERSION

ENV ROS_DISTRO=${ROS_DISTRO} \
    CC=${CC} \
    CXX=${CXX} \
    PX4_DIR=/opt/PX4-Autopilot \
    TERM=xterm-256color \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    GZ_SIM_RESOURCE_PATH="/workspace/worlds:/workspace/worlds/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

SHELL ["/bin/bash", "-o", "pipefail", "-x", "-c"]

RUN apt-get update \
    && apt-get install --no-install-recommends -y clangd g++ \
    && rm -rf /var/lib/apt/lists/*

RUN source /etc/os-release \
    && test "$VERSION_ID" = "$UBUNTU_VERSION" \
    && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && command -v MicroXRCEAgent \
    && git clone --depth 1 --branch "$PX4_VERSION" --recursive --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" \
    && make -C "$PX4_DIR" px4_sitl_default \
    && test -x "$PX4_DIR/build/px4_sitl_default/bin/px4"

WORKDIR /opt/px4_ros2

RUN mkdir -p src \
    && git clone --depth 1 --branch "$PX4_MSGS_VERSION" \
        https://github.com/PX4/px4_msgs.git src/px4_msgs \
    && source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --symlink-install

WORKDIR /workspace

RUN apt-get update \
    && apt-get install --no-install-recommends -y python3-venv \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/ruff \
    && /opt/ruff/bin/python -m pip install --no-cache-dir "ruff==${RUFF_VERSION}" \
    && install -m 0755 /opt/ruff/bin/ruff /usr/local/bin/ruff

COPY src/ src/

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && source /opt/px4_ros2/install/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

RUN chown -R ubuntu:ubuntu /workspace/build /workspace/install /workspace/log

RUN printf 'source "/opt/ros/%s/setup.bash"\n' "$ROS_DISTRO" >> /etc/bash.bashrc

RUN cat >> /etc/bash.bashrc <<'EOF'
source /opt/px4_ros2/install/setup.bash
source /workspace/install/setup.bash
EOF

RUN cat >> /root/.bashrc <<'EOF'
PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
EOF

RUN install -d -m 0700 "$XDG_RUNTIME_DIR"
