FROM px4io/px4-dev-ros2-gazebo:sha-bf9b1610ca27af976f89ea59a45003dd8cbb5fe2-jazzy@sha256:ed83ec2bb14c283dc10a5acd2675831c0fcca44be37ecc895e65ad4cd610f05a

ARG PX4_VERSION=v1.17.0
ARG PX4_MSGS_VERSION=v1.17.0

ENV ROS_DISTRO=jazzy \
    PX4_DIR=/opt/PX4-Autopilot \
    TERM=xterm-256color \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    GZ_SIM_RESOURCE_PATH="/workspace/worlds:/workspace/worlds/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

SHELL ["/bin/bash", "-o", "pipefail", "-x", "-c"]

RUN apt-get update \
    && apt-get install --no-install-recommends -y clangd \
    && rm -rf /var/lib/apt/lists/*

RUN source /etc/os-release \
    && test "$VERSION_ID" = "24.04" \
    && source /opt/ros/jazzy/setup.bash \
    && test "$ROS_DISTRO" = "jazzy" \
    && command -v MicroXRCEAgent \
    && git clone --depth 1 --branch "$PX4_VERSION" --recursive --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" \
    && make -C "$PX4_DIR" px4_sitl_default \
    && test -x "$PX4_DIR/build/px4_sitl_default/bin/px4"

WORKDIR /opt/px4_ros2

RUN mkdir -p src \
    && git clone --depth 1 --branch "$PX4_MSGS_VERSION" \
        https://github.com/PX4/px4_msgs.git src/px4_msgs \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install

WORKDIR /workspace

COPY src/ src/

RUN source /opt/ros/jazzy/setup.bash \
    && source /opt/px4_ros2/install/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

RUN chown -R ubuntu:ubuntu /workspace/build /workspace/install /workspace/log

RUN cat >> /etc/bash.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
source /workspace/install/setup.bash
EOF

RUN cat >> /root/.bashrc <<'EOF'
PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
EOF

RUN install -d -m 0700 "$XDG_RUNTIME_DIR"
