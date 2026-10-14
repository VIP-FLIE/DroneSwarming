# Gazebo worlds

PX4 provides the stock Gazebo worlds in the simulation image. Both launch files use `default` unless `world:=<name>` selects another world.

Store project world files in this directory as `<name>.sdf`. Store local models referenced by those worlds in `models/<model-name>/`.

Docker Compose mounts this directory at `/workspace/worlds`. The image adds `/workspace/worlds` and `/workspace/worlds/models` to `GZ_SIM_RESOURCE_PATH`, which Gazebo uses to find world and model resources. Select a project world with `world:=<name>`.

Gazebo documents `GZ_SIM_RESOURCE_PATH` in its [resource lookup guide](https://gazebosim.org/api/sim/8/resources.html). PX4 documents that worlds and models on this path do not need to be copied into PX4's own directories in its [Gazebo simulation guide](https://docs.px4.io/v1.18/en/sim_gazebo_gz/).
