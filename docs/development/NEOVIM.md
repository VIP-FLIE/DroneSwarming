# Neovim LSP

This guide connects native Neovim to the project’s containerized C/C++ language server. The host editor communicates with `clangd` through [`scripts/clangd`](../../scripts/clangd). The container provides ROS 2, PX4, `clangd`, and the compilation database.

## Requirements

- Linux with Docker Engine and the Docker Compose plugin
- Neovim 0.11.3 or later
- [`nvim-lspconfig`](https://github.com/neovim/nvim-lspconfig), installed with a Neovim plugin manager

The host does not need ROS 2, PX4, or `clangd` installed.

## Configure Neovim

Add this configuration after `nvim-lspconfig` loads:

```lua
vim.lsp.config("clangd", {
  cmd = { vim.fn.getcwd() .. "/scripts/clangd" },
})

vim.lsp.enable("clangd")
```

Start Neovim from the repository root so the command resolves to the project wrapper:

```bash
cd /path/to/DroneSwarming
nvim .
```

This configuration uses the Neovim 0.11 LSP API. The `nvim-lspconfig` project provides the `clangd` server defaults. See its [Neovim version requirements and setup instructions](https://github.com/neovim/nvim-lspconfig#readme).

## Initialize the container build

The Compose service stores `build/`, `install/`, and `log/` in named volumes. The first start creates an empty `build/` volume, so build the workspace once to create `build/compile_commands.json`:

```bash
docker compose up --build --detach development
docker compose exec development bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/px4_ros2/install/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
```

The wrapper starts the `development` service when it is stopped. It maps the host repository path to `/workspace` for `clangd`. The project [`.clangd`](../../.clangd) file points `clangd` to the generated compilation database.

Repeat the `colcon build` command after changing package build metadata or adding a package. Restart Neovim after the build to reload the compilation database.

Use these commands in a C or C++ buffer:

```vim
:lua vim.lsp.buf.definition()
:lua vim.lsp.buf.hover()
:lua vim.lsp.buf.references()
:lua vim.lsp.buf.rename()
```

Use `:checkhealth vim.lsp` to inspect Neovim's LSP setup. `clangd` provides diagnostics and completion for C and C++ files. This project wrapper configures C/C++ only; it does not start a Python language server for native Neovim.

See the [clangd compilation database guide](https://clangd.llvm.org/installation.html#compile_commandsjson) for compilation database details.
