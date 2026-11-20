# Gazebo worlds and models

Docker Compose defaults to `PX4_GZ_WORLD=vip_windy` and `PX4_GZ_MODEL=vip_x500`. The world uses a flat 400 m ground plane with a grass-colored grid texture, the PX4 lawn sky and clouds, and a steady 5 m/s wind along world +X. It contains no aerial obstacles. The project X500 is copied from PX4 v1.17.0 and retains its frame mass, rotor geometry, motor dynamics, and sensor configuration. Its main body link has Gazebo wind enabled.

PX4 Gazebo world plugins are configured in [`px4/server.config`](../px4/server.config). That configuration retains `SceneBroadcaster` and includes `WindEffects`. The world SDF contains no world-level plugin elements, so PX4's server configuration remains active. Compose mounts the server configuration and [`px4/gz_env.sh`](../px4/gz_env.sh) into the existing PX4 installation at runtime. Apply Compose mount changes with:

```bash
./scripts/sim up -d simulation
```

World, model, server, and PX4 parameter configuration changes use bind mounts or launch-time settings and do not require rebuilding the image.

Set either environment variable to override a default. PX4-supported stock models with a matching Gazebo airframe can be selected by name. `PX4_GZ_MODEL=x500` selects PX4's original X500 files:

```bash
PX4_GZ_WORLD=default ./scripts/px4-sitl
PX4_GZ_MODEL=x500 ./scripts/px4-sitl
PX4_GZ_WORLD=default PX4_GZ_MODEL=x500 ./scripts/tmux-session start
```

Store project world files here as `<name>.sdf`. The name must start with a letter or digit and contain only letters, digits, underscores, and hyphens. Store project models in `models/<model-name>/`. The custom drone files are in `models/vip_x500/` and `models/vip_x500_base/`. `models/px4-vip-x500/x500` is a symlink because PX4's existing `gz_x500` autostart target looks for an `x500/` model directory.

The launcher sets `PX4_PARAM_SIM_BAT_DRAIN=900` by default. PX4's SITL battery simulator linearly reduces charge over 900 armed seconds and resets the simulated charge while disarmed. Set `PX4_PARAM_SIM_BAT_DRAIN` before launch to change the duration. The simulator does not calculate battery current from motor load.

Docker Compose mounts this directory at `/workspace/worlds`. The image adds `/workspace/worlds` and `/workspace/worlds/models` to `GZ_SIM_RESOURCE_PATH` for Gazebo resource lookup. Restart PX4 and Gazebo after changing the world or model selection.
