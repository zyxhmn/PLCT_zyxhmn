# 使用MoveIt任务构造器实现抓取与放置

本教程将引导您创建一个使用MoveIt任务构造器（MoveIt Task Constructor，简称MTC）规划抓取放置操作的软件包。MTC通过将复杂运动规划任务拆解为多个子任务（称为"阶段"）来实现规划。若您想直接运行完整示例，可参考Docker指南启动包含完整教程的容器。

------

## 1 基本概念

MTC的核心思想是将复杂运动规划问题分解为多个简单子问题。顶层规划任务称为"任务"（Task），子问题通过"阶段"（Stage）定义。阶段可以任意组合排列，其顺序由结果传递方向决定。根据结果流向，阶段分为三类：

- **生成器阶段**：独立计算结果，向前后双向传递（如基于几何位姿的IK采样器）
- **传播器阶段**：接收相邻阶段结果，处理后单向传递（如基于起始/目标状态的笛卡尔路径计算）
- **连接器阶段**：尝试桥接相邻阶段的状态差异（如自由空间运动规划）

阶段层级类型：

- **封装器**：修饰或过滤单个子阶段的结果（如IK封装器）
- **串行容器**：按顺序执行子阶段，仅考虑端到端解决方案（如连贯的抓取动作序列）
- **并行容器**：组合多个子阶段，择优选择或并行执行（如尝试不同规划器）

[![../../../_images/mtc_stage_types.png](https://moveit.picknik.ai/main/_images/mtc_stage_types.png)](https://moveit.picknik.ai/main/_images/mtc_stage_types.png)

更多概念详见[MTC概念页](https://moveit.picknik.ai/main/doc/concepts/concepts.html)。

------

## 2 准备工作

确保已完成[入门指南](https://moveit.picknik.cn/main/doc/getting_started/getting_started.html)的步骤。

1. 克隆MTC源码（替换<branch>为对应ROS版本）：

```bash
git clone -b <branch> https://github.com/moveit/moveit_task_constructor.git	
```

1. 安装依赖：

```bash
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

1. 编译工作空间：

```
cd ..
colcon build --mixin release
```

## 3 示例演示

启动基础环境：

```
ros2 launch moveit_task_constructor_demo demo.launch.py
```

运行不同示例：

```
# 笛卡尔运动
ros2 launch moveit_task_constructor_demo run.launch.py exe:=cartesian
# 模块化示例
ros2 launch moveit_task_constructor_demo run.launch.py exe:=modular 
# 抓取放置完整示例
ros2 launch moveit_task_constructor_demo run.launch.py exe:=pick_place_demo
```

在RViz右侧面板可查看阶段结构、解决方案并可视化轨迹。

[![../../../_images/mtc_show_stages1.gif](https://moveit.picknik.ai/main/_images/mtc_show_stages1.gif)](https://moveit.picknik.ai/main/_images/mtc_show_stages1.gif)

------

## 4 创建MTC项目

### 4.1 新建软件包

```
ros2 pkg create --build-type ament_cmake \
--dependencies moveit_task_constructor_core rclcpp \
--node-name mtc_node mtc_tutorial
```

### 4.2 代码实现

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
```



**头文件包含**

```
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/task.h>
// ...其他必要头文件
```

**类定义**

```
class MTCTaskNode {
public:
  // 构造函数、接口获取、任务执行等方法
private:
  mtc::Task createTask(); // 任务构建方法
  // ...其他成员变量
};
```

**场景设置**

```
void MTCTaskNode::setupPlanningScene() {
  // 创建圆柱体碰撞物体
  moveit_msgs::msg::CollisionObject object;
  object.primitives[0].dimensions = {0.1, 0.02}; // 直径0.1m，高度0.02m
  pose.position.x = 0.5; // X轴位置0.5米
  // ...应用场景
}
```

**任务执行流程**

```
void MTCTaskNode::doTask() {
  task_ = createTask();
  if(task_.plan(5)) { // 最多尝试5次规划
    task_.execute(*task_.solutions().front()); // 执行最优解
  }
}
```

**任务构建**

```
mtc::Task MTCTaskNode::createTask() {
  mtc::Task task;
  task.setProperty("group", "panda_arm"); // 设置机械臂组
  // 添加当前状态阶段
  auto stage_current = std::make_unique<mtc::stages::CurrentState>("current");
  // ...添加更多阶段
  return task;
}
```

------

## 5 运行示例

### 5.1 启动文件配置

创建`pick_place_demo.launch.py`：

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mtc_tutorial",
            executable="mtc_node",
            output="screen"
        )
    ])
```

### 5.2 编译运行

```
cd ~/ws_moveit
colcon build --mixin release
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```

------

## 6 添加抓取阶段

在现有代码基础上扩展：

**连接阶段**

```
auto stage_connect = std::make_unique<mtc::stages::Connect>(
  "approach", 
  { {"panda_arm", sampling_planner} }
);
```

**抓取容器**

```
auto grasp = std::make_unique<mtc::SerialContainer>("抓取动作");
// 包含接近、抓取、抬升等子阶段
```

**接近物体**

```
auto approach = std::make_unique<mtc::stages::MoveRelative>("接近物体", cartesian_planner);
geometry_msgs::msg::Vector3Stamped direction;
direction.vector.z = -0.5; // Z轴负方向移动
approach->setDirection(direction);
```

**生成抓取姿态**

```
auto generate_grasp = std::make_unique<mtc::stages::GenerateGraspPose>();
generate_grasp->setPreGraspPose("open"); // 预抓取姿态
```

**IK求解**

```
auto compute_ik = std::make_unique<mtc::stages::ComputeIK>();
compute_ik->setMaxIKSolutions(8); // 最大IK解数量
```

------

