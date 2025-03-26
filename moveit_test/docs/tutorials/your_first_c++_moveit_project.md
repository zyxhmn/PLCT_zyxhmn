# 您的第一个 C++ MoveIt 项目

## 步骤

### 1 创建包

创建并导航到目录。

```bash
mkdir -p ws_moveit/src 
cd ws_moveit/src
```

使用 ROS 2 命令行工具创建一个新包：

```bash
ros2 pkg create \
 --build-type ament_cmake \
 --dependencies moveit_ros_planning_interface rclcpp \
 --node-name hello_moveit hello_moveit
```

请注意，我们添加了 `moveit_ros_planning_interface` 和 `rclcpp` 作为依赖项。这将在 `package.xml` 和 `CMakeLists.txt` 文件中创建必要的更改，以便我们可以依赖这两个包。`ws_moveit/src/hello_moveit/src/hello_moveit.cpp` 在您喜欢的编辑器中打开为您创建的新源文件。

### 2 创建 ROS 节点和执行器

第一段代码有点样板，但您应该习惯于 ROS 2 教程中看到这一点。

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
```

