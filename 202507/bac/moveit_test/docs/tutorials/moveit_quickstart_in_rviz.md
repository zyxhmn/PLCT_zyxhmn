# RViz 中的 MoveIt 快速入门

## 第 1 步：启动 demo 并配置插件

Launch the demo: 启动演示：

```
ros2 launch moveit2_tutorials demo.launch.py
```

![rviz1](202507/bac/moveit_test/docs/tutorials/imgs/moveit_quickstart_in_rviz/1.png)

如果您是第一次执行此作，您应该会在 RViz 中看到一个空世界，并且必须添加 Motion Planning 插件：

1. 在 RViz Displays （RViz 显示） 选项卡中，按 *Add （添加* ）：

2. 从 moveit_ros_visualization 文件夹中，选择“MotionPlanning”作为 DisplayType。按“确定”。

3. 您现在应该在 RViz 中看到 Kinova 机器人.

现在，您可以开始为您的机器人配置插件。单击“显示”中“MotionPlanning”。

- 确保 **Robot Description** 字段设置为 `robot_description`。
- 确保 **Planning Scene Topic** 字段设置为 `/monitored_planning_scene`。单击 topic name 以显示 topic-name 下拉列表。
- 确保 **Planned Path** 下的 **Trajectory Topic** 设置为 `/display_planned_path`。
- 在 **Planning Request （规划请求** ） 中，将 **Planning Group （规划组** ） 更改为 `manipulator`。您还可以在左下角的 MotionPlanning 面板中看到这一点。

## 第 2 步：玩转可视化机器人 

有四种不同的重叠可视化：

1. /`monitored_planning_scene` 规划环境中的机器人配置（默认处于活动状态）。
2. 机器人的计划路径（默认处于活动状态）。
3. 绿色：运动规划的开始状态（默认处于禁用状态）。
4. 橙色：运动规划的目标状态（默认处于活动状态）。

可以使用复选框打开和关闭每个可视化的显示状态：

1. 使用 **Scene Robot** 树菜单中的 **Show Robot Visual** 复选框规划场景机器人。
2. 使用 **Planned Path** 树菜单中的 **Show Robot Visual** 复选框的计划路径。
3. 使用 **Planning Request** 树菜单中的 **Query Start State** 复选框的开始状态。
4. 使用 **Planning Request** 树菜单中的 **Query Goal State** 复选框的目标状态。

## 第 3 步：与 Kinova Gen 3 交互 

在接下来的步骤中，我们只需要场景机器人、开始状态和目标状态：

1. 选中 **Planned Path** 树菜单中的 **Show Robot Visual** 复选框
2. 取消选中 **Scene Robot** 树菜单中的 **Show Robot Visual** 复选框
3. 选中 **Planning Request** 树菜单中的 **Query Goal State** 复选框。
4. 选中 **Planning Request** 树菜单中的 **Query Start State** 复选框。

现在应该有两个交互式标记。一个与橙色手臂相对应的标记将用于设置运动规划的“目标状态”，另一个对应于绿色手臂的标记用于设置运动规划的“开始状态”。如果您没有看到交互式标记，请按 RViz 顶部菜单中的 **Interact**（注意：某些工具可能被隐藏，按顶部菜单中的 **“+”** 添加 **Interact** 工具，如下所示）。





### 保存您的配置

RViz 允许您将配置保存在 `File->Save Config` 下。您应该先执行此作，然后再继续学习下一个教程。如果您选择以新名称保存配置，则可以使用 `File->Save Config As` 并使用以下方法引用您的配置文件：

```bash
ros2 launch moveit2_tutorials demo.launch.py rviz_config:=your_rviz_config.rviz
```

异常：选择`File->Save Config As` 时rviz窗口为灰色且无法操作，使用esc可退出。

