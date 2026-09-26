# Adding your own package

> **Convention:** Names like `your_package`, `your_node`, `YourNode`, and `YourMessage`
> are placeholders. Replace each one with the name you choose when you follow along.
> Everything else is real code that stays as written.

Everything under `src/` is a ROS 2 package. A package is a directory with its
own dependencies and its own build rules, and it contains one or more nodes. A
node is one running program, which is what `ros2 run` starts.

Two build types, and a package is one or the other:

| Build type | Language | Examples here |
|---|---|---|
| `ament_python` | Python | `src/py_single_square` |
| `ament_cmake` | C++ | `src/cpp_single_hop`, `src/swarm_naive`, `src/swarm_comms` |

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
  --node-name your_node your_package
cd ..
```

| Path | What it is |
|---|---|
| `your_package/package.xml` | Name, version, maintainer and dependencies. colcon reads this to work out build order |
| `your_package/setup.py` | What gets installed, and the `console_scripts` entry points that `ros2 run` uses |
| `your_package/setup.cfg` | Points setuptools at `lib/your_package`, which is the directory `ros2 run` searches |
| `your_package/resource/your_package` | Empty marker file. It is what registers the package in the ament index |
| `your_package/your_package/__init__.py` | Makes the directory an importable Python package |
| `your_package/your_package/your_node.py` | Your node |
| `your_package/test/` | Three lint tests the template ships with |

## 2. Fix the metadata

The generator fills the maintainer in from your git config and leaves the
description as `TODO`. Edit both files, since they each carry their own copy:

- `package.xml`: `<description>`, `<maintainer>`, `<version>`
- `setup.py`: `description`, `maintainer`, `maintainer_email`, `version`

## 3. Write the node

`your_package/your_package/your_node.py`:

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition

from py_single_square.px4_qos import PX4_QOS, topic_prefix


class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')

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

Three things in there are specific to this project:

- `PX4_QOS` on anything touching `/fmu/`. A default subscription never fires.
  [QUICKSTART.md](../simulation/QUICKSTART.md) has the detail.
- `topic_prefix` with a `px4_ns` parameter, so the same node works on drone 1 and
  drone 5. Instance 0 is bare `/fmu/...` and instance N is `/px4_N/fmu/...`.
- `main` takes `args=None` and calls `rclpy.init`, because the entry point below
  calls it with no arguments.

Importing `py_single_square.px4_qos` means `py_single_square` is a dependency. Add it to
`package.xml`:

```xml
<depend>py_single_square</depend>
```

[`src/py_single_square/py_single_square/listener.py`](../../src/py_single_square/py_single_square/listener.py)
is the same shape with more in it.

## 4. Register the entry point

`--node-name` already added an entry for the node it generated. This is what you
do for every node you add after that, in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'your_node = your_package.your_node:main',
    ],
},
```

The name on the left is what you type: `ros2 run your_package your_node`. The right
side is `module.path:function`. A node that is not listed here builds fine and
then reports `No executable found`.

## 5. Build and run

```bash
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 run your_package your_node
ros2 run your_package your_node --ros-args -p px4_ns:=px4_1
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
  --node-name your_node your_package
cd ..
```

| Path | What it is |
|---|---|
| `your_package/package.xml` | Same role as in a Python package |
| `your_package/CMakeLists.txt` | The build rules. This is where most of the work goes |
| `your_package/src/your_node.cpp` | Your node |
| `your_package/include/your_package/` | Headers, if you split the code up. Delete it if you do not |

## 2. Write CMakeLists.txt

Replace the generated file with this:

```cmake
cmake_minimum_required(VERSION 3.8)
project(your_package)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

The generated template uses `ament_target_dependencies(your_node "rclcpp" "px4_msgs")`
instead of `target_link_libraries`. It works, and this repo uses the target form
because it is the current ROS 2 style and matches `src/cpp_single_hop`. Pick one,
and do not mix them in the same target.

Adding a second node to the same package means a second `add_executable`,
`target_compile_features`, `target_link_libraries` and an entry in
`install(TARGETS ...)`.

## 3. Write the node

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

The header path is the message name in snake_case: `VehicleLocalPosition` is
`px4_msgs/msg/vehicle_local_position.hpp`. The QoS is the same profile the
Python side gets from `PX4_QOS`, and `/fmu/in/vehicle_command` is the exception
that needs `reliable()` instead of `best_effort()`.
[`src/cpp_single_hop/src/hop.cpp`](../../src/cpp_single_hop/src/hop.cpp)
has both, with the reasoning next to them.

## 4. Build and run

```bash
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 run your_package your_node
ros2 run your_package your_node --ros-args -p px4_ns:=px4_1
```

C++ needs a rebuild after every change. `--symlink-install` does not give the
C++ side an edit-in-place shortcut.

---

# Launch files

One `ros2 launch` instead of several terminals. A launch file starts several
processes at once, passes each one its parameters, and can stagger them in time.
It is how anything with more than one node gets run.

Launch files go in `launch/` and are ordinary Python. They have to be installed
or `ros2 launch` cannot find them:

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

```python
# setup.py, for an ament_python package
data_files=[
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
],
```

Missing that line or block is why a launch file that exists reports `file not
found`.

`launch/your_package.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='your_node',
            name='your_node_1',
            output='screen',
            parameters=[{'px4_ns': 'px4_1'}],
        ),
    ])
```

```bash
colcon build --symlink-install --packages-select your_package
source install/setup.bash
ros2 launch your_package your_package.launch.py
```

| Thing | What it is for |
|---|---|
| `DeclareLaunchArgument` | A value you can pass on the command line: `ros2 launch pkg file.launch.py count:=5` |
| `LaunchConfiguration('count')` | Reads one back. It is a substitution, not a value, so it has nothing in it until launch is running |
| `OpaqueFunction` | Runs your Python once launch has started, where `LaunchConfiguration(...).perform(context)` gives you a real value. This is what you need to loop over an argument |
| `TimerAction` | Starts actions after a delay. Staggering process startup |
| `ExecuteProcess` | Runs something that is not a ROS node, such as PX4 itself |
| `--show-args` | Prints every argument and its description without launching anything |

[`src/swarm_naive/launch/swarm.launch.py`](../../src/swarm_naive/launch/swarm.launch.py)
uses all of those to start N PX4 instances and N nodes, and is commented line by
line. [SWARM.md](../swarm/SWARM.md) explains it.

---

# Your own message type

`px4_msgs` covers talking to PX4. A message of your own is for anything your
nodes say to each other.

`msg/YourMessage.msg`, one field per line, `type name`:

```
uint64 stamp_us
uint8 slot
bool ready
float32 north_m
```

`CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YourMessage.msg"
)

add_executable(your_node src/your_node.cpp)

# Only needed when the node lives in the SAME package as the message.
rosidl_get_typesupport_target(my_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(your_node PRIVATE rclcpp::rclcpp ${my_typesupport})
```

`package.xml`:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Then `#include <your_package/msg/your_message.hpp>`, which is the message name in
snake_case under the package name.

A message used from the package that generates it links through the
`rosidl_get_typesupport_target` target. That target cannot be found with
`find_package`, because it does not exist yet while the package is still being
built. Leave it out and the code compiles and then fails to link, with undefined
references to the message's type support.

Most tutorials put messages in a package of their own, and for a large project
that is right: everybody who wants to talk to you then depends on a small
interface package instead of on all of your code. One message used by one node
does not earn a second package.

Worked example: [`msg/SwarmState.msg`](../../src/swarm_comms/msg/SwarmState.msg)
in `src/swarm_comms`, which is what the drones say to each other.

```bash
ros2 interface show swarm_comms/msg/SwarmState
ros2 topic echo /swarm/state
```

---

# Dependencies

A dependency has to be declared in `package.xml` and then used by the language's
own mechanism. `package.xml` alone installs nothing; it tells colcon what to
build first and rosdep what to install.

| Dependency | `package.xml` | Python | C++ |
|---|---|---|---|
| ROS 2 client library | `<depend>rclpy</depend>` or `<depend>rclcpp</depend>` | `import rclpy` | `find_package(rclcpp REQUIRED)` and link `rclcpp::rclcpp` |
| PX4 messages | `<depend>px4_msgs</depend>` | `from px4_msgs.msg import ...` | `find_package(px4_msgs REQUIRED)` and link `${px4_msgs_TARGETS}` |
| The shared QoS profile | `<depend>py_single_square</depend>` | `from py_single_square.px4_qos import PX4_QOS` | Not applicable, C++ defines its own in the node |
| A plain pip package | `<exec_depend>python3-numpy</exec_depend>` | `import numpy` | Not applicable |
| A message of your own | `<buildtool_depend>rosidl_default_generators</buildtool_depend>`, `<exec_depend>rosidl_default_runtime</exec_depend>`, `<member_of_group>rosidl_interface_packages</member_of_group>` | `from your_package.msg import YourMessage` | `rosidl_generate_interfaces(...)` and link the target from `rosidl_get_typesupport_target` |
| Launch files | `<exec_depend>launch</exec_depend>`, `<exec_depend>launch_ros</exec_depend>` | Install `launch/` in `setup.py` | `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})` |

`px4_msgs` lives in `~/ros2_ws`, which is a separate workspace built before this
one. It is already on your path in both setups.

# Building

```bash
colcon build --symlink-install                          # everything in src/
colcon build --symlink-install --packages-select your_package
colcon build --symlink-install --packages-up-to your_package   # it and its dependencies
source install/setup.bash
```

Always run these from the repo root, the directory that contains `src/`. colcon
searches downward from wherever you start it, so running it inside `src/` builds
a workspace in the wrong place.

Every new terminal needs `source install/setup.bash` before `ros2 run` can see
your package. In the dev container new terminals do it for you. Natively,
`scripts/native/env.sh` does it, and
[NATIVE-SETUP.md](../setup/NATIVE-SETUP.md) covers the one line that goes in
`~/.bashrc`.

To keep colcon out of a directory, put an empty `COLCON_IGNORE` file in it.

# Checklist

- [ ] Directory under `src/`, named in lowercase with underscores
- [ ] `package.xml` lists every dependency you import or link
- [ ] Python: node registered in `entry_points` in `setup.py`
- [ ] C++: `add_executable`, `target_link_libraries`, and the target named in `install(TARGETS ...)`
- [ ] `PX4_QOS` or the equivalent on every `/fmu/` topic
- [ ] `px4_ns` parameter, so the node survives meeting a second drone
- [ ] Launch files, if any, installed into `share/your_package`
- [ ] `colcon build` from the repo root, then `source install/setup.bash`
- [ ] `ros2 pkg executables your_package` lists what you expect

# Problems

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'your_package' not found` | Not built, or the overlay is not sourced in this terminal | `colcon build`, then `source install/setup.bash` |
| `No executable found` | Python: no `console_scripts` entry. C++: the target is missing from `install(TARGETS ...)` | Add it and rebuild |
| Node runs the old code after an edit | Built without `--symlink-install`, or it is a C++ node | Rebuild. `--symlink-install` only helps Python |
| `ModuleNotFoundError: No module named 'your_package'` | The Python file is not inside `your_package/your_package/`, or `__init__.py` is missing | Move the file, add the empty `__init__.py` |
| `ModuleNotFoundError: No module named 'px4_msgs'` | `~/ros2_ws` overlay not sourced | Source `install/setup.bash` from `~/ros2_ws`, or open a fresh terminal |
| C++ `fatal error: rclcpp/rclcpp.hpp: No such file` | Missing `find_package` or the target is not linked | Both lines in `CMakeLists.txt` |
| Editor shows red squiggles on `#include <px4_msgs/...>` while the build succeeds | No compile database yet | Build once, then reload the editor window. `set(CMAKE_EXPORT_COMPILE_COMMANDS ON)` has to be in the package's `CMakeLists.txt` |
| Package builds but `ros2 pkg list` does not show it | `resource/<name>` marker missing or misnamed | It must match the package name exactly |
| Callback never fires, `ros2 topic echo` shows data | QoS mismatch | `PX4_QOS`, see [QUICKSTART.md](../simulation/QUICKSTART.md) |
| colcon builds packages you did not expect | A nested workspace under `src/` | Put an empty `COLCON_IGNORE` in it |
| Import fails with a dash in the name | A Python module name cannot contain a dash | Rename the package, underscores only |
| `ros2 launch` cannot find a launch file that exists | Not installed | `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})`, or the `data_files` entry in `setup.py` |
| Undefined references to your own message's type support | Message used from the package that generates it | Link the target from `rosidl_get_typesupport_target` |
| `ros2 interface show` does not list your message | `<member_of_group>rosidl_interface_packages</member_of_group>` missing | Add it to `package.xml` and rebuild |

# Reference

- [HOW-IT-WORKS.md](../simulation/HOW-IT-WORKS.md), the pipeline your node plugs into
- [QUICKSTART.md](../simulation/QUICKSTART.md), QoS, offboard mode and multi-vehicle
- [`src/py_single_square/py_single_square/listener.py`](../../src/py_single_square/py_single_square/listener.py), the smallest complete Python node here
- [`src/cpp_single_hop/src/hop.cpp`](../../src/cpp_single_hop/src/hop.cpp), the same for C++, with the three PX4 rules marked
- [SWARM.md](../swarm/SWARM.md), and [`src/swarm_naive`](../../src/swarm_naive/) and [`src/swarm_comms`](../../src/swarm_comms/), for launch files and a message of your own in use
- [Creating custom msg files](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html), the upstream tutorial
- [Creating launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html), the upstream tutorial
- [Creating a package](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), the upstream tutorial
- [px4_msgs release/1.16](https://github.com/PX4/px4_msgs/tree/release/1.16/msg), every message you can subscribe to