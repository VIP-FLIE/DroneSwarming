# Adding your own package

`drone_vip` and `drone_swarm` are ROS 2 packages. A package is a directory under
`src/` with its own dependencies and its own build rules, and it contains one or
more nodes. A node is one running program, which is what `ros2 run` starts.

Two build types, and a package is one or the other:

| Build type | Language | Example here |
|---|---|---|
| `ament_python` | Python | `src/drone_vip` |
| `ament_cmake` | C++ | `src/drone_swarm` |

A package cannot hold both. A project that needs both gets two packages.

## New package, or a new node in an existing one

Add a node to an existing package when it shares that package's dependencies and
belongs to the same job. It is one new file plus one line of registration.

Create a package when the thing has its own dependencies, or when it is a
separate piece of work that somebody else will own. A package can be built and
tested on its own with `colcon build --packages-select`.

## Naming

Lowercase letters, digits and underscores. No dashes and no capitals. For a
Python package the directory name becomes an importable Python module, so it has
to be a valid identifier. The same name appears in four places and they all have
to match: the directory, `package.xml`, `setup.py` or `CMakeLists.txt`, and the
marker file in `resource/`.

Everything below runs from the repo root, the directory that contains `src/`.

---

# Python package

## 1. Create it

```bash
cd src
ros2 pkg create --build-type ament_python --license MIT \
  --dependencies rclpy px4_msgs \
  --node-name watcher my_package
cd ..
```

| Path | What it is |
|---|---|
| `my_package/package.xml` | Name, version, maintainer and dependencies. colcon reads this to work out build order |
| `my_package/setup.py` | What gets installed, and the `console_scripts` entry points that `ros2 run` uses |
| `my_package/setup.cfg` | Points setuptools at `lib/my_package`, which is the directory `ros2 run` searches |
| `my_package/resource/my_package` | Empty marker file. It is what registers the package in the ament index |
| `my_package/my_package/__init__.py` | Makes the directory an importable Python package |
| `my_package/my_package/watcher.py` | Your node |
| `my_package/test/` | Three lint tests the template ships with |

## 2. Fix the metadata

The generator fills the maintainer in from your git config and leaves the
description as `TODO`. Edit both files, since they each carry their own copy:

- `package.xml`: `<description>`, `<maintainer>`, `<version>`
- `setup.py`: `description`, `maintainer`, `maintainer_email`, `version`

## 3. Write the node

`my_package/my_package/watcher.py`:

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition

from drone_vip.px4_qos import PX4_QOS, topic_prefix


class Watcher(Node):
    def __init__(self):
        super().__init__('watcher')

        self.declare_parameter('px4_ns', '')
        prefix = topic_prefix(self.get_parameter('px4_ns').value)

        self.create_subscription(
            VehicleLocalPosition,
            f'{prefix}/fmu/out/vehicle_local_position',
            self.on_position,
            PX4_QOS)

    def on_position(self, msg):
        self.get_logger().info(f'altitude {-msg.z:.1f} m')


def main(args=None):
    rclpy.init(args=args)
    node = Watcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

Three things in there are specific to this project:

- `PX4_QOS` on anything touching `/fmu/`. A default subscription never fires.
  [QUICKSTART.md](../QUICKSTART.md) has the detail.
- `topic_prefix` with a `px4_ns` parameter, so the same node works on drone 1 and
  drone 5. Instance 0 is bare `/fmu/...` and instance N is `/px4_N/fmu/...`.
- `main` takes `args=None` and calls `rclpy.init`, because the entry point below
  calls it with no arguments.

Importing `drone_vip.px4_qos` means `drone_vip` is a dependency. Add it to
`package.xml`:

```xml
<depend>drone_vip</depend>
```

`src/drone_vip/drone_vip/listener.py` is the same shape with more in it.

## 4. Register the entry point

`--node-name` already added an entry for the node it generated. This is what you
do for every node you add after that, in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'watcher = my_package.watcher:main',
    ],
},
```

The name on the left is what you type: `ros2 run my_package watcher`. The right
side is `module.path:function`. A node that is not listed here builds fine and
then reports `No executable found`.

## 5. Build and run

```bash
colcon build --symlink-install --packages-select my_package
source install/setup.bash
ros2 run my_package watcher
ros2 run my_package watcher --ros-args -p px4_ns:=px4_1
```

`--symlink-install` installs links rather than copies, so editing the body of a
node takes effect on the next `ros2 run`. Rebuild when you add a file, rename
something, or change `setup.py`.

---

# C++ package

## 1. Create it

```bash
cd src
ros2 pkg create --build-type ament_cmake --license MIT \
  --dependencies rclcpp px4_msgs \
  --node-name watcher my_package
cd ..
```

| Path | What it is |
|---|---|
| `my_package/package.xml` | Same role as in a Python package |
| `my_package/CMakeLists.txt` | The build rules. This is where most of the work goes |
| `my_package/src/watcher.cpp` | Your node |
| `my_package/include/my_package/` | Headers, if you split the code up. Delete it if you do not |

## 2. Write CMakeLists.txt

Replace the generated file with this:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(watcher src/watcher.cpp)
target_compile_features(watcher PRIVATE cxx_std_17)
target_link_libraries(watcher PRIVATE rclcpp::rclcpp ${px4_msgs_TARGETS})

install(TARGETS watcher DESTINATION lib/${PROJECT_NAME})

ament_package()
```

Line by line:

| Line | Why |
|---|---|
| `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)` | Writes the compile database colcon merges into `build/compile_commands.json`. `.vscode/c_cpp_properties.json` reads it, so editor IntelliSense gets the real compile flags |
| `find_package` | One per dependency. Without it the headers are not on the include path |
| `add_executable` | Every node you want to run needs one, with its source files |
| `target_compile_features` | `rclcpp` needs C++17 |
| `target_link_libraries` | `rclcpp::rclcpp` and `${px4_msgs_TARGETS}` are imported targets. They carry their own include directories, so nothing else has to be listed |
| `install(TARGETS ...)` | `ros2 run` searches `lib/<package>`. Without this line the node builds and `ros2 run` reports `No executable found` |
| `ament_package()` | Last line, always |

The generated template uses `ament_target_dependencies(watcher "rclcpp" "px4_msgs")`
instead of `target_link_libraries`. It works, and this repo uses the target form
because it is the current ROS 2 style and matches `src/drone_swarm`. Pick one,
and do not mix them in the same target.

Adding a second node to the same package means a second `add_executable`,
`target_compile_features`, `target_link_libraries` and an entry in
`install(TARGETS ...)`.

## 3. Write the node

`my_package/src/watcher.cpp`:

```cpp
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

class Watcher : public rclcpp::Node
{
public:
  Watcher()
  : Node("watcher")
  {
    const std::string px4_ns = declare_parameter<std::string>("px4_ns", "");
    const std::string prefix = px4_ns.empty() ? "" : "/" + px4_ns;

    rclcpp::QoS qos{rclcpp::KeepLast(1)};
    qos.best_effort();
    qos.transient_local();

    position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      prefix + "/fmu/out/vehicle_local_position", qos,
      [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        RCLCPP_INFO(get_logger(), "altitude %.1f m", -msg->z);
      });
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Watcher>());
  rclcpp::shutdown();
  return 0;
}
```

The header path is the message name in snake_case: `VehicleLocalPosition` is
`px4_msgs/msg/vehicle_local_position.hpp`. The QoS is the same profile the
Python side gets from `PX4_QOS`, and `/fmu/in/vehicle_command` is the exception
that needs `reliable()` instead of `best_effort()`. `src/drone_swarm/src/hop.cpp`
has both, with the reasoning next to them.

## 4. Build and run

```bash
colcon build --symlink-install --packages-select my_package
source install/setup.bash
ros2 run my_package watcher
ros2 run my_package watcher --ros-args -p px4_ns:=px4_1
```

C++ has no shortcut equivalent to editing a Python file in place. Every change
needs a rebuild.

---

# Dependencies

A dependency has to be declared in `package.xml` and then used by the language's
own mechanism. `package.xml` alone installs nothing; it tells colcon what to
build first and rosdep what to install.

| Dependency | `package.xml` | Python | C++ |
|---|---|---|---|
| ROS 2 client library | `<depend>rclpy</depend>` or `<depend>rclcpp</depend>` | `import rclpy` | `find_package(rclcpp REQUIRED)` and link `rclcpp::rclcpp` |
| PX4 messages | `<depend>px4_msgs</depend>` | `from px4_msgs.msg import ...` | `find_package(px4_msgs REQUIRED)` and link `${px4_msgs_TARGETS}` |
| The shared QoS profile | `<depend>drone_vip</depend>` | `from drone_vip.px4_qos import PX4_QOS` | Not applicable, C++ defines its own in the node |
| A plain pip package | `<exec_depend>python3-numpy</exec_depend>` | `import numpy` | Not applicable |

`px4_msgs` lives in `~/ros2_ws`, which is a separate workspace built before this
one. It is already on your path in both setups.

# Building

```bash
colcon build --symlink-install                          # everything in src/
colcon build --symlink-install --packages-select my_package
colcon build --symlink-install --packages-up-to my_package   # it and its dependencies
source install/setup.bash
```

Always run these from the repo root, the directory that contains `src/`. colcon
searches downward from wherever you start it, so running it inside `src/` builds
a workspace in the wrong place.

Every new terminal needs `source install/setup.bash` before `ros2 run` can see
your package. In the dev container new terminals do it for you. Natively,
`scripts/native/env.sh` does it, and
[NATIVE-SETUP.md](NATIVE-SETUP.md) covers the one line that goes in `~/.bashrc`.

To keep colcon out of a directory, put an empty `COLCON_IGNORE` file in it.

# Checklist

- [ ] Directory under `src/`, named in lowercase with underscores
- [ ] `package.xml` lists every dependency you import or link
- [ ] Python: node registered in `entry_points` in `setup.py`
- [ ] C++: `add_executable`, `target_link_libraries`, and the target named in `install(TARGETS ...)`
- [ ] `PX4_QOS` or the equivalent on every `/fmu/` topic
- [ ] `px4_ns` parameter, so the node survives meeting a second drone
- [ ] `colcon build` from the repo root, then `source install/setup.bash`
- [ ] `ros2 pkg executables my_package` lists what you expect

# Problems

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'my_package' not found` | Not built, or the overlay is not sourced in this terminal | `colcon build`, then `source install/setup.bash` |
| `No executable found` | Python: no `console_scripts` entry. C++: the target is missing from `install(TARGETS ...)` | Add it and rebuild |
| Node runs the old code after an edit | Built without `--symlink-install`, or it is a C++ node | Rebuild. `--symlink-install` only helps Python |
| `ModuleNotFoundError: No module named 'my_package'` | The Python file is not inside `my_package/my_package/`, or `__init__.py` is missing | Move the file, add the empty `__init__.py` |
| `ModuleNotFoundError: No module named 'px4_msgs'` | `~/ros2_ws` overlay not sourced | Source `install/setup.bash` from `~/ros2_ws`, or open a fresh terminal |
| C++ `fatal error: rclcpp/rclcpp.hpp: No such file` | Missing `find_package` or the target is not linked | Both lines in `CMakeLists.txt` |
| Editor shows red squiggles on `#include <px4_msgs/...>` while the build succeeds | No compile database yet | Build once, then reload the editor window. `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)` has to be in the package's `CMakeLists.txt` |
| Package builds but `ros2 pkg list` does not show it | `resource/<name>` marker missing or misnamed | It must match the package name exactly |
| Callback never fires, `ros2 topic echo` shows data | QoS mismatch | `PX4_QOS`, see [QUICKSTART.md](../QUICKSTART.md) |
| colcon builds packages you did not expect | A nested workspace under `src/` | Put an empty `COLCON_IGNORE` in it |
| Import fails with a dash in the name | A Python module name cannot contain a dash | Rename the package, underscores only |

# Reference

- [HOW-IT-WORKS.md](HOW-IT-WORKS.md), the pipeline your node plugs into
- [QUICKSTART.md](../QUICKSTART.md), QoS, offboard mode and multi-vehicle
- `src/drone_vip/drone_vip/listener.py`, the smallest complete Python node here
- `src/drone_swarm/src/hop.cpp`, the same for C++, with the three PX4 rules marked
- [Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), the upstream tutorial
- [px4_msgs release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg), every message you can subscribe to
