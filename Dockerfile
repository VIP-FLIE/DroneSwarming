FROM px4io/px4-sitl-gazebo-ros2:v1.18.0-beta1-660-g61907962c9@sha256:2c01c89215a5f541d3a819c1bd2115ff27c9423679f3738dbb4096271d77084b

ENV ROS_DISTRO=jazzy \
    TERM=xterm-256color \
    GZ_SIM_RESOURCE_PATH="/workspace/worlds:/workspace/worlds/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

WORKDIR /workspace

RUN . /etc/os-release \
    && test "$VERSION_ID" = "24.04" \
    && . /opt/ros/jazzy/setup.bash \
    && test "$ROS_DISTRO" = "jazzy" \
    && command -v px4-gazebo \
    && command -v MicroXRCEAgent

COPY src/ src/

RUN /usr/local/bin/ros2-entrypoint.sh colcon build --symlink-install

RUN cat >> /etc/bash.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
source /workspace/install/setup.bash
EOF

RUN cat >> /root/.bashrc <<'EOF'
PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
EOF
