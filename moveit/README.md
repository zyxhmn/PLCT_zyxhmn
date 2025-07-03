# Moveit

安装colcon:

```bash
pip3 install -U pytest colcon-common-extensions
```

编译moveit tutorials

```bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src
git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
```

检查moveit2_tutorials.repos，查看部分依赖是否安装

| 依赖项                  | 是否安装 | 版本  | 最新版本                                                     |
| ----------------------- | -------- | ----- | ------------------------------------------------------------ |
| launch_param_builder    | 是       | 0.1.1 | 0.1.1                                                        |
| moveit2                 | 是       | 2.5.4 | 2.5.9                                                        |
| moveit_resources        | 是       | 2.0.6 | 2.0.7                                                        |
| moveit_task_constructor | 否       | null  | [7566349](https://github.com/moveit/moveit_task_constructor/commit/756634951326ae17ae099882f7110c6f1d0a98c0) |
| moveit_visual_tools     | 是       | 4.1.0 | 4.1.2                                                        |
| rosparam_shortcuts      | 否       | null  | [0.5.0](https://github.com/PickNikRobotics/rosparam_shortcuts/tree/ros2) |
| srdfdom                 | 是       | 2.0.4 | 2.0.8                                                        |

拉取未安装的依赖

```bash
git clone https://github.com/ros-planning/moveit_task_constructor.git -b humble
cd moveit_task_constructor
git checkout 615e8ef248ec604658e6685851457cd567673cdb
cd ..
git clone https://github.com/PickNikRobotics/rosparam_shortcuts -b ros2
```

| 缺失软件包                  | 仓库地址                                   | 分支 |
| --------------------------- | ------------------------------------------ | ---- |
| ros-humble-py-binding-tools | https://github.com/moveit/py_binding_tools | ros2 |

使用下述命令构建软件包

```bash
colcon build  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-DBOOST_TIMER_ENABLE_DEPRECATED"
```

构建软件包，完成后将`source ~/moveit2_ws/install/setup.bash`添加至`.bashrc`文件中。

## Moveit Quickstart in RViz

### step 1 启动官方demo并且配置rviz插件

启动rviz

````bash
ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_moveit_config_demo_empty.rviz
````

第一次启动时，将在rviz中看到一个空世界，在`RViz Displays`选项卡中点击`add`,在`moveit_ros_visualization`文件夹中选择`MotionPlanning`。

现在可以在rviz中看到Panda机器人

![rviz_start](./imgs/rviz_start.png)

