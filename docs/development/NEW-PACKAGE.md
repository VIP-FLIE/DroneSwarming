# Adding your own package

A package is a directory under `src/` with its own build rules. Contains one or more nodes. A node is one running program, started with `ros2 run`.

Package names: lowercase, underscores, no dashes. The directory name becomes a Python module name.

All commands from the repo root.

## Python

### 1. Create

```bash
cd src
ros2 pkg create --build-type ament_python your_package
cd ..
```

Generates:

| Path | Purpose |
|---|---|
| `your_package/package.xml` | Name, version, dependencies. colcon reads for build order |
| `your_package/setup.py` | What gets installed, `console_scripts` entries `ros2 run` uses |
| `your_package/setup.cfg` | Points setuptools at source directory |
| `your_package/resource/your_package` | Empty marker. Registers in ament index |
| `your_package/your_package/__init__.py` | Makes directory importable |

### 2. Add dependencies

In `package.xml`, inside `<package>`:

```xml
<depend>rclpy</depend>
<depend>px4_msgs</depend>
```

### 3. Write the node

`your_package/your_package/your_node.py`:

```python
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition


class YourNode(Node):
    def __init__(self) -> None:
        super().__init__('your_node')

        self.declare_parameter('px4_ns', '')
        ns: str = self.get_parameter('px4_ns').value.strip('/')
        prefix: str = f'/{ns}' if ns else ''

        self.create_subscription(
            VehicleLocalPosition,
            f'{prefix}/fmu/out/vehicle_local_position',
            self.on_position,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ))

    def on_position(self, msg: VehicleLocalPosition) -> None:
        self.get_logger().info(f'altitude {-msg.z:.1f} m')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YourNode()
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

All Python code uses type annotations. `from __future__ import annotations` enables forward references so `list[str] | None` works in Python 3.10+.

QoS is not optional. PX4 publishes `/fmu/out/` with BEST_EFFORT reliability. ROS 2 subscribers default to RELIABLE. Mismatch creates the subscription without error; callback never fires. `ros2 topic echo` negotiates QoS.

`px4_ns` handles multi-vehicle. Instance 0: bare `/fmu/...`. Instance N: `/px4_N/fmu/...`.

### 4. Register entry point

In `setup.py`, under `entry_points`:

```python
entry_points={
    'console_scripts': [
        'your_node = your_package.your_node:main',
    ],
},
```

Left side is what you type: `ros2 run your_package your_node`. Right side: `module.path:function`. A node not listed here builds and reports `No executable found`.

### 5. Build and run

```bash
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 run your_package your_node
ros2 run your_package your_node --ros-args -p px4_ns:=px4_1
```

`--symlink-install` installs links. Editing a Python node takes effect on next `ros2 run`. Rebuild after adding files or changing `setup.py`.

## C++

### 1. Create

```bash
cd src
ros2 pkg create --build-type ament_cmake your_package
cd ..
```

Generates:

| Path | Purpose |
|---|---|
| `your_package/package.xml` | Same role as Python |
| `your_package/CMakeLists.txt` | Build rules |
| `your_package/src/` | Source files |
| `your_package/include/your_package/` | Headers. Delete if unused |

### 2. Add dependencies

In `package.xml`:

```xml
<depend>rclcpp</depend>
<depend>px4_msgs</depend>
```

### 3. CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(your_package)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(your_node src/your_node.cpp)
target_compile_features(your_node PRIVATE cxx_std_17)
target_link_libraries(your_node PRIVATE rclcpp::rclcpp ${px4_msgs_TARGETS})

install(TARGETS your_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

| Line | Why |
|---|---|
| `find_package` | One per dependency. Without it, headers not on include path |
| `add_executable` | One per node, with source files |
| `target_compile_features` | `rclcpp` needs C++17 |
| `target_link_libraries` | `rclcpp::rclcpp` and `${px4_msgs_TARGETS}` carry include directories |
| `install(TARGETS ...)` | `ros2 run` searches `lib/<package>`. Without it: builds, reports `No executable found` |
| `ament_package()` | Last line |

Adding a second node: second `add_executable`, `target_compile_features`, `target_link_libraries`, and entry in `install(TARGETS ...)`.

### 4. Write the node

`your_package/src/your_node.cpp`:

```cpp
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

class YourNode : public rclcpp::Node
{
public:
  YourNode()
  : Node("your_node")
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
  rclcpp::spin(std::make_shared<YourNode>());
  rclcpp::shutdown();
  return 0;
}
```

Header path is message name in snake_case: `VehicleLocalPosition` → `px4_msgs/msg/vehicle_local_position.hpp`. QoS matches `PX4_QOS` on the Python side. `/fmu/in/vehicle_command` needs `reliable()` instead of `best_effort()`.

### 5. Build and run

```bash
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 run your_package your_node
ros2 run your_package your_node --ros-args -p px4_ns:=px4_1
```

C++ needs a rebuild after every change. `--symlink-install` does not give C++ an edit-in-place shortcut.

## Reference

- [`src/py_single_square/py_single_square/listener.py`](../../src/py_single_square/py_single_square/listener.py), smallest complete Python node
- [`src/cpp_single_hop/src/hop.cpp`](../../src/cpp_single_hop/src/hop.cpp), same for C++
- [../simulation/HOW-IT-WORKS.md](../simulation/HOW-IT-WORKS.md), the pipeline
- [../swarm/SWARM.md](../swarm/SWARM.md), launch files and multi-drone examples