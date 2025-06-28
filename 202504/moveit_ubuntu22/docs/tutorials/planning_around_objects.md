# 围绕对象进行规划

## 1 添加规划场景接口

在hello_moveit.cpp中添加

```BASH
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
```

## 2 改变目标姿势

首先，通过以下更改更新目标姿势，使机器人规划到不同的位置：

```cpp
// Set a target Pose with updated values !!!
auto const target_pose = [] {
  geometry_msgs::msg::Pose msg;
  msg.orientation.y = 0.8;
  msg.orientation.w = 0.6;
  msg.position.x = 0.1;
  msg.position.y = 0.4;
  msg.position.z = 0.4;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);
```

## 3 创建碰撞对象

在下一个代码块中，我们创建一个碰撞对象。首先要注意的是，它被放置在机器人的坐标系中。如果我们有一个感知系统来报告场景中障碍物的位置，那么这就是它会构建的消息类型。因为这只是一个示例，所以我们是手动创建的。在这个代码块的末尾要注意的一件事是，我们将此消息上的操作设置为 `ADD` 。这会导致对象被添加到碰撞场景中。将此代码块放在上一步设置目标姿势和创建计划之间。

```cpp
// Create collision object for the robot to avoid
auto const collision_object = [frame_id =
                                 move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}();
```

## 4 将对象添加到规划场景

最后，我们需要将此对象添加到碰撞场景中。为此，我们使用一个名为 `PlanningSceneInterface` 的对象，该对象使用 ROS 接口将规划场景的更改传达给 `MoveGroup` 。此代码块应直接跟在创建碰撞对象的代码块后面。

```cpp
// Add the collision object to the scene
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
planning_scene_interface.applyCollisionObject(collision_object);
```

## 5 运行程序并观察变化

就像我们在上一个教程中所做的那样，使用 `demo.launch.py` 脚本启动 RViz 并运行我们的程序。

```
ros2 launch moveit2_tutorials demo.launch.py
```

源代码见