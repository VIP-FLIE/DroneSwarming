# Zed LSP

This guide runs Zed and its language servers in the project's development container. The container provides ROS 2, PX4, `clangd`, and the project compilation database.

## Requirements

- Docker Engine or another Docker-compatible engine available on `PATH`
- Zed 0.218 or later with Dev Containers support

The host does not need ROS 2, PX4, or `clangd` installed. Zed's [Dev Containers guide](https://zed.dev/docs/dev-containers) lists the current container requirements.

## Open the development container

Open the repository root in Zed. Choose **Open in Container** when prompted. To reconnect later, run **Project: Open Remote** from the Command Palette and select the dev container.

Zed reads [`.devcontainer/devcontainer.json`](../../.devcontainer/devcontainer.json), which selects the `development` service in [`compose.yaml`](../../compose.yaml). The configured post-create command builds the workspace and generates `/workspace/build/compile_commands.json`.

## Use the language servers

Zed includes C and C++ support and uses `clangd`. Its Dev Container session runs language servers inside the container, where the project installs `clangd` and ROS/PX4 headers. The project [`.clangd`](../../.clangd) file points `clangd` to the compilation database.

Use completion, diagnostics, hover information, references, and definition navigation in C and C++ files. Run **dev: open language server logs** from the Command Palette to inspect server output. Run **editor: restart language server** after regenerating the compilation database.

Rebuild the database from Zed's integrated terminal after changing package build metadata or adding a package:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

The [clangd compilation database guide](https://clangd.llvm.org/installation.html#compile_commandsjson) describes the database used for C++ analysis.

After editing `.devcontainer/devcontainer.json`, stop the existing container and reopen the project in the container. Zed does not rebuild or reload the container after configuration changes.
