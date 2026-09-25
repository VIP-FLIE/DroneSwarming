# tutorial_package

The smallest complete C++ ROS 2 package. One node, `your_node`, that logs one
line and waits. No drone, no PX4, no simulator.

Copy it to start a C++ package of your own.

## 1. What it does

`ros2 run tutorial_package your_node` starts a node named `your_node`. It logs
`Hello from your_node.` once, then does nothing until Ctrl-C.

It publishes nothing and subscribes to nothing. No drone moves.

## 2. What you learn from it

| Concept | Where |
|---|---|
| The three files every C++ ROS 2 package needs | `package.xml`, `CMakeLists.txt`, `src/your_node.cpp` |
| The names that must match across those files | Section 4, "Names that must match" |
| How colcon finds and builds a package | `package.xml` |
| How `ros2 run` finds an executable | `install(TARGETS ...)` in `CMakeLists.txt` |
| A node as a class derived from `rclcpp::Node` | `YourNode` |
| The `init`, `spin`, `shutdown` lifecycle every C++ node follows | `main()` |
| Logging with `RCLCPP_INFO` | The constructor |

Every file carries comments explaining each line. Read them in the order
`package.xml`, `CMakeLists.txt`, `src/your_node.cpp`.

## 3. How to run it

### Build

From the repo root, the directory that contains `src/`:

```bash
colcon build --symlink-install --packages-select tutorial_package
source install/setup.bash
```

C++ has to be rebuilt after every edit.

### Run

One terminal. No simulator, agent or `drone-gcs` needed.

```bash
ros2 run tutorial_package your_node
```

Output:

```
[INFO] [1790351086.575358431] [your_node]: Hello from your_node.
```

The number is the timestamp. `[your_node]` is the node name from the
constructor. The node keeps running with nothing more to print. Ctrl-C to stop
it.

While it runs, a second terminal shows it:

```bash
ros2 node list          # /your_node
```

### In the simulator

This package does not talk to PX4, so the simulator changes nothing. Section 4,
"Making it talk to the drone", lists the edits that turn it into a node that
reads the drone's position.

### Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'tutorial_package' not found` | Not built, or the overlay is not sourced in this terminal | Build, then `source install/setup.bash` |
| `No executable found` | The target is missing from `install(TARGETS ...)` | Add it and rebuild |
| Old behaviour after an edit | C++ was not rebuilt | Rebuild |
| `build/`, `install/` and `log/` appear inside `src/` | colcon was run from inside `src/` | Delete them and run colcon from the repo root |

[docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md) has the full table.

## 4. How it works

### Files

| File | Role |
|---|---|
| `package.xml` | Tells colcon the package name, its build type, and what it depends on |
| `CMakeLists.txt` | Build rules: what to compile, what to link, where to install it |
| `src/your_node.cpp` | The node |

### `package.xml`

| Element | Value | Why |
|---|---|---|
| `<name>` | `tutorial_package` | Must match the directory and `project()` in `CMakeLists.txt` |
| `<buildtool_depend>` | `ament_cmake` | The C++ build system. Needed at build time only |
| `<depend>` | `rclcpp` | The C++ ROS 2 client library. `<depend>` covers build time and run time |
| `<build_type>` | `ament_cmake` | Marks this as a C++ package. The alternative is `ament_python`. A package is one or the other |
| `<maintainer>` | `you@example.com` | A placeholder. Replace it in your copy |

colcon reads the dependencies to decide build order. A dependency used in the
code but missing here can build on your machine and fail on a clean checkout.

### `CMakeLists.txt`

| Line | Why |
|---|---|
| `project(tutorial_package)` | Must match `<name>` in `package.xml` |
| `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)` | Writes `compile_commands.json`, which the editor reads for IntelliSense. Without it the editor shows errors on includes that build fine |
| `add_compile_options(-Wall -Wextra -Wpedantic)` | Compiler warnings on |
| `find_package(ament_cmake REQUIRED)` | Always required |
| `find_package(rclcpp REQUIRED)` | One `find_package` per `<depend>` |
| `add_executable(your_node src/your_node.cpp)` | Names the executable. `your_node` is what `ros2 run` takes |
| `target_compile_features(your_node PRIVATE cxx_std_17)` | `rclcpp` needs C++17 |
| `target_link_libraries(your_node PRIVATE rclcpp::rclcpp)` | Links `rclcpp`. The imported target carries its own include paths |
| `install(TARGETS your_node DESTINATION lib/${PROJECT_NAME})` | `ros2 run` looks in `lib/<package>`. Without this line the build succeeds and `ros2 run` reports `No executable found` |
| `ament_package()` | Registers the package. Always the last line |

### `src/your_node.cpp`

| Part | What it does |
|---|---|
| `#include <rclcpp/rclcpp.hpp>` | The whole C++ client library |
| `class YourNode : public rclcpp::Node` | A node is a class derived from `rclcpp::Node` |
| `Node("your_node")` | The node name. It appears in `ros2 node list` and in every log line |
| `RCLCPP_INFO(get_logger(), ...)` | Logs at INFO level. `RCLCPP_WARN`, `RCLCPP_ERROR` and `RCLCPP_DEBUG` are the other levels |
| `rclcpp::init(argc, argv)` | Reads ROS arguments such as `--ros-args -p name:=value` and starts the middleware. Must come first |
| `rclcpp::spin(std::make_shared<YourNode>())` | Constructs the node, then runs its callbacks until Ctrl-C. This node has no callbacks, so it waits |
| `rclcpp::shutdown()` | Cleans up the middleware after `spin` returns |

The log line is printed from the constructor, which runs once inside
`make_shared`, before `spin` starts.

### Names that must match

| Where | Current value |
|---|---|
| Directory | `src/tutorial_package/` |
| `package.xml` | `<name>tutorial_package</name>` |
| `CMakeLists.txt` | `project(tutorial_package)` |

If these disagree, colcon cannot find or build the package. Names are lowercase
letters, digits and underscores.

The executable name (`your_node`) and the node name (`"your_node"` in the
constructor) are separate and do not have to match each other or the package.

### Using it as a template

```bash
cp -r src/tutorial_package src/my_package
```

Then, in the copy:

1. `package.xml`: change `<name>`, `<description>` and `<maintainer>`.
2. `CMakeLists.txt`: change `project()`.
3. To rename the executable, change `your_node` in `add_executable`,
   `target_compile_features`, `target_link_libraries` and `install(TARGETS ...)`.
4. Rename the source file and the class to suit, and update `add_executable` to
   match.

```bash
colcon build --symlink-install --packages-select my_package
source install/setup.bash
ros2 run my_package your_node
```

`ros2 pkg create` generates the same structure from scratch.
[docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md) covers that route, for both C++
and Python.

### Making it talk to the drone

Three edits turn it into a node that reads the drone's position from PX4.

`package.xml`, below the `rclcpp` line:

```xml
<depend>px4_msgs</depend>
```

`CMakeLists.txt`, the first line next to the other `find_package` lines, the
second replacing the existing `target_link_libraries`:

```cmake
find_package(px4_msgs REQUIRED)
target_link_libraries(your_node PRIVATE rclcpp::rclcpp ${px4_msgs_TARGETS})
```

`src/your_node.cpp`: subscribe to `/fmu/out/vehicle_local_position` with PX4's
QoS. The complete node is step 3 of the C++ section in
[docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md). The QoS is the part that
matters: PX4 publishes best effort with transient local durability, and a
default subscription never receives anything from it.

Then run it against the simulator with the usual stack from
[QUICKSTART.md](../../QUICKSTART.md): `MicroXRCEAgent`, `drone-gcs`,
`drone-sim`, and your node in a fourth terminal.

### Design decisions

| Decision | Reason |
|---|---|
| No PX4 dependency | The package builds and runs with nothing else running. Build problems and PX4 problems stay separate |
| `target_link_libraries` with imported targets | The current ROS 2 style, and what every other C++ package in this repo uses. `ros2 pkg create` generates `ament_target_dependencies`, which also works. Do not mix the two on one target |
| A class derived from `rclcpp::Node` | The shape every other node here uses, so the template grows into them without restructuring |
| Every line commented | It is meant to be read before it is copied |

### Where to go next

| To | Read |
|---|---|
| Create a package properly, C++ or Python | [docs/NEW-PACKAGE.md](../../docs/NEW-PACKAGE.md) |
| A complete C++ flight node | [`../cpp_single_hop`](../cpp_single_hop/README.md) |
| How your node plugs into PX4 | [docs/HOW-IT-WORKS.md](../../docs/HOW-IT-WORKS.md) |
