# 机器人仿真
使用tiago机器人在gazebo仿真环境中进行导航抓取
仓库 https://github.com/zyxhmn/ROSArmVision 
noetic与humble分支都使用tiago
# moveit tutorials问题修复
具体情况为使用`ros2 launch moveit2_tutorial demo.launch.py` 时，move group报错，[日志](./move_group_5258_1755946637394.log)表现为插件无法加载;
```bash
[ERROR] [move_group.move_group]: Exception while loading move_group capability 'move_group/ApplyPlanningSceneService': 
MultiLibraryClassLoader: Could not create object of class type move_group::ApplyPlanningSceneService as no factory exists for it. 
Make sure that the library exists and was explicitly loaded through MultiLibraryClassLoader::loadLibrary()

```
通过CMakeLists.txt 中 显式链接了 moveit_move_group_default_capabilities 目标后重新构建解决问题。
详细对比编译后的插件，发现新so文件与旧so文件相比多了
0x000000000000000f (RPATH)  Library rpath: [/opt/ros/humble/lib:/opt/ros/humble/lib64]
