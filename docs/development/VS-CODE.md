# VS Code LSP

This guide runs VS Code and its language servers in the project's development container. The container provides ROS 2, PX4, `clangd`, Python, and the project build environment.

## Requirements

- Docker Engine with the Docker Compose plugin
- Visual Studio Code
- The [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

The host does not need ROS 2, PX4, or `clangd` installed.

## Open the development container

1. Open the repository root in VS Code.
2. Install the recommended extensions when VS Code prompts.
3. Run **Dev Containers: Reopen in Container** from the Command Palette.

The repository's [`.devcontainer/devcontainer.json`](../../.devcontainer/devcontainer.json) selects the `development` service in [`compose.yaml`](../../compose.yaml). Its `postCreateCommand` builds the workspace and creates `/workspace/build/compile_commands.json`.

Install or enable these extensions in the container:

- [`llvm-vs-code-extensions.vscode-clangd`](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) for C and C++
- `ms-python.python` and `ms-python.vscode-pylance` for Python
- `ms-iot.vscode-ros` for ROS project integration

The repository recommends these extensions in [`.vscode/extensions.json`](../../.vscode/extensions.json). The [`clangd` package](../../Dockerfile) runs inside the container. The repository disables the C/C++ extension's IntelliSense engine to prevent duplicate C++ language services.

## Use the language servers

Open a C or C++ file for diagnostics, completion, hover information, references, and definition navigation. Open a Python launch file for Pylance diagnostics and navigation. The repository's [VS Code settings](../../.vscode/settings.json) select `/usr/bin/python3` and add the ROS 2 Jazzy and `px4_msgs` Python packages to Pylance's search paths.

Use the **Output** panel and select **clangd** to inspect C/C++ language-server messages. Run **clangd: Restart language server** after regenerating the compilation database.

Rebuild the database from the VS Code terminal after changing package build metadata or adding a package:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ros2/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

The [`.clangd`](../../.clangd) file selects `build/` as the compilation database directory. The [`clangd` guide](https://clangd.llvm.org/installation.html#compile_commandsjson) describes how this database informs C++ analysis.

Run **Dev Containers: Rebuild Container** after changing the Dockerfile or development-container configuration.
