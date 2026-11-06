# Gazebo worlds

The Compose services set `PX4_GZ_WORLD=lawn` by default. The value is available in sourced tmux
panes and interactive container shells before PX4 starts. PX4's stock `lawn` world has one flat
green ground plane, a visible grid, and a sky with clouds. It has no obstacle models.

Set `PX4_GZ_WORLD` to use another world. The override works with the host wrapper, a sourced
container shell, and every tmux mode:

```bash
PX4_GZ_WORLD=default ./scripts/px4-sitl
PX4_GZ_WORLD=default ./scripts/tmux-session start
PX4_GZ_WORLD=default ./scripts/tmux-session test
```

Store project world files here as `<name>.sdf`. Use a name that does not match a stock PX4 world.
The name must start with a letter or digit and contain only letters, digits, underscores, and
hyphens. The PX4 wrapper links a selected project world into PX4's world directory inside the
container. Store local models referenced by the world in `models/<model-name>/`.

Docker Compose mounts this directory at `/workspace/worlds`. The image adds `/workspace/worlds`
and `/workspace/worlds/models` to `GZ_SIM_RESOURCE_PATH` for Gazebo resource lookup. Restart PX4
and Gazebo after changing the world selection.
