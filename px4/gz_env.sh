# Runtime environment for PX4 Gazebo Harmonic.

PX4_GZ_MODELS="${PX4_GZ_MODELS:-/opt/PX4-Autopilot/Tools/simulation/gz/models}"
PX4_GZ_WORLDS=/opt/PX4-Autopilot/Tools/simulation/gz/worlds
PX4_GZ_PLUGINS=/opt/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins
PX4_GZ_SERVER_CONFIG=/opt/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config

export PX4_GZ_MODELS PX4_GZ_WORLDS PX4_GZ_PLUGINS PX4_GZ_SERVER_CONFIG
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+${GZ_SIM_RESOURCE_PATH}:}${PX4_GZ_MODELS}:${PX4_GZ_WORLDS}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+${GZ_SIM_SYSTEM_PLUGIN_PATH}:}${PX4_GZ_PLUGINS}"
export GZ_SIM_SERVER_CONFIG_PATH="$PX4_GZ_SERVER_CONFIG"
