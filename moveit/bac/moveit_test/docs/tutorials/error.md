

# error

## error1

```bash
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
[haltija@localhost ws_moveit]$ rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
rosdep detected OS: [centos] aliasing it to: [rhel]
executing command [sudo -H dnf --assumeyes --setopt=strict=0 install ros-humble-py-binding-tools boost-python3-devel ros-humble-ros2-control urdfdom-headers-devel]
Last metadata expiration check: 0:40:26 ago on 2025年03月26日 星期三 22时51分52秒.
No match for argument: ros-humble-py-binding-tools
No match for argument: boost-python3-devel
No match for argument: urdfdom-headers-devel
Dependencies resolved.

 Problem: package ros-humble-ros2-control-2.25.2-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-ros2controlcli, but none of the providers can be installed
  - 冲突的请求
  - nothing provides python3-pygraphviz needed by ros-humble-ros2controlcli-2.25.2-1.oe2403.x86_64 from openEulerROS-humble
================================================================================================
 Package                        Arch        Version              Repository                Size
================================================================================================
Skipping packages with broken dependencies:
 ros-humble-ros2-control        x86_64      2.25.2-1.oe2403      openEulerROS-humble      9.6 k
 ros-humble-ros2controlcli      x86_64      2.25.2-1.oe2403      openEulerROS-humble       32 k

Transaction Summary
================================================================================================
Skip  2 Packages

Nothing to do.
Complete!
ERROR: the following rosdeps failed to install
  dnf: Failed to detect successful installation of [ros-humble-py-binding-tools]
  dnf: Failed to detect successful installation of [boost-python%{python3_pkgversion}-devel]
  dnf: Failed to detect successful installation of [ros-humble-ros2-control]
  dnf: Failed to detect successful installation of [urdfdom-headers-devel]

```

rosdep问题。

## error2

```bash
/home/haltija/ws_moveit/src/moveit2/moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/butterworth_filter.h:44:10: 致命错误：moveit_core/moveit_butterworth_parameters.hpp：No such file or directory
   44 | #include <moveit_core/moveit_butterworth_parameters.hpp>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
编译中断。
gmake[2]: *** [online_signal_smoothing/CMakeFiles/moveit_butterworth_filter.dir/build.make:76：online_signal_smoothing/CMakeFiles/moveit_butterworth_filter.dir/src/butterworth_filter.cpp.o] 错误 1
gmake[1]: *** [CMakeFiles/Makefile2:1283：online_signal_smoothing/CMakeFiles/moveit_butterworth_filter.dir/all] 错误 2
gmake[1]: *** 正在等待未完成的任务....
gmake: *** [Makefile:146：all] 错误 2
---
Failed   <<< moveit_core [20.5s, exited with code 2]
Aborted  <<< generate_parameter_library_example [32.6s]       
```

缺少generate_parameter_library

```bash
git clone https://github.com/PickNikRobotics/generate_parameter_library.git -b main
```
## error3

```bash

gmake[2]: *** 没有规则可制作目标“/usr/lib64/libboost_system.so”，由“libmoveit_warehouse.so.2.5.8” 需求。 停止。
gmake[2]: *** 正在等待未完成的任务....
gmake[1]: *** [CMakeFiles/Makefile2:149：CMakeFiles/moveit_warehouse.dir/all] 错误 2
gmake: *** [Makefile:146：all] 错误 2
---
```

缺少 Boost Thread 库的开发文件,使用dnf安装显示以及安装：

```bash
Package boost-system-1.83.0-4.oe2403sp1.x86_64 is already installed.
Package boost-filesystem-1.83.0-4.oe2403sp1.x86_64 is already installed.
```

仍然失败,本地编译安装boost 1.76

```bash
# 在用户目录下创建工作空间
mkdir -p ~/custom_boost/boost_1_73_0
cd ~/custom_boost
wget https://sourceforge.net/projects/boost/files/boost/1.76.0/boost_1_73_0.tar.gz
tar -xzf boost_1_76_0.tar.gz
cd boost_1_76_0
```

```bash
./bootstrap.sh --prefix=/home/xxx/custom_boost/install
./b2 install --prefix=/home/xxx/custom_boost/install
```

安装后：

```bash
# 设置 Boost 1.76 的路径
export BOOST_ROOT=~/custom_boost/install
export LD_LIBRARY_PATH=$BOOST_ROOT/lib:$LD_LIBRARY_PATH
export CPLUS_INCLUDE_PATH=$BOOST_ROOT/include:$CPLUS_INCLUDE_PATH

# 验证路径
echo $BOOST_ROOT       # 应输出 /home/yourname/custom_boost/install
ls $BOOST_ROOT/lib     
```

编译时指定boost

```bash
cd ~/ws_moveit
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOST_ROOT=$BOOST_ROOT \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DBoost_USE_STATIC_LIBS=OFF
```

## error4

```bash
gmake[2]: *** [test/unit_tests/CMakeFiles/unittest_trajectory_functions.dir/build.make:367：test/unit_tests/unittest_trajectory_functions] 错误 1
gmake[1]: *** [CMakeFiles/Makefile2:643：test/unit_tests/CMakeFiles/unittest_trajectory_functions.dir/all] 错误 2
gmake[1]: *** 正在等待未完成的任务....
gmake: *** [Makefile:146：all] 错误 2
---
```

单元测试有问题，暂时选择加入跳过所有测试的构建选项：

```bash
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOST_ROOT=$BOOST_ROOT \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DBoost_USE_STATIC_LIBS=OFF \
  -DBUILD_TESTING=OFF \
  -DCATKIN_ENABLE_TESTING=OFF
```

编译完成后暂未发现问题。

## error5

```bash
CMake Error at CMakeLists.txt:15 (find_package):
  By not providing "Findpy_binding_tools.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "py_binding_tools", but CMake did not find one.

  Could not find a package configuration file provided by "py_binding_tools"
  with any of the following names:

    py_binding_toolsConfig.cmake
    py_binding_tools-config.cmake

  Add the installation prefix of "py_binding_tools" to CMAKE_PREFIX_PATH or
  set "py_binding_tools_DIR" to a directory containing one of the above
  files.  If "py_binding_tools" provides a separate development package or
  SDK, be sure it has been installed.
```

缺少py_binding_tools，软件包管理器未提供，选择编译安装：

```bash
cd src
git clone https://github.com/moveit/py_binding_tools.git -b ros2
cd ..
colcon build --symlink-install --packages-select py_binding_tools
source ~/ws_moveit/install/setup.bash
```

接着使用下面的命令进行编译：

```bash
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOST_ROOT=$BOOST_ROOT \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DBoost_USE_STATIC_LIBS=OFF \
  -DBUILD_TESTING=OFF \
  -DCATKIN_ENABLE_TESTING=OFF
```

## error6

```bash
/home/haltija/ws_moveit/src/moveit_task_constructor/visualization/motion_planning_tasks/properties/property_factory.cpp: 在静态成员函数‘static rviz_common::properties::Property* moveit_rviz_plugin::PropertyFactory::createDefault(const std::string&, const std::string&, const std::string&, const std::string&, rviz_common::properties::Property*)’中:
/home/haltija/ws_moveit/src/moveit_task_constructor/visualization/motion_planning_tasks/properties/property_factory.cpp:162:65: 错误：‘rviz’不是一个类型名
  162 |                 rviz_common::properties::Property* result = new rviz::StringProperty(
      |                                                                 ^~~~
/home/haltija/ws_moveit/src/moveit_task_constructor/visualization/motion_planning_tasks/properties/property_factory.cpp:152:86: 警告：unused parameter ‘name’ [-Wunused-parameter]
  152 | rviz_common::properties::Property* PropertyFactory::createDefault(const std::string& name, const std::string& /*type*/,
      |                                                                   ~~~~~~~~~~~~~~~~~~~^~~~
gmake[2]: *** [motion_planning_tasks/properties/CMakeFiles/motion_planning_tasks_properties.dir/build.make:90：motion_planning_tasks/properties/CMakeFiles/motion_planning_tasks_properties.dir/property_factory.cpp.o] 错误 1
gmake[1]: *** [CMakeFiles/Makefile2:347：motion_planning_tasks/properties/CMakeFiles/motion_planning_tasks_properties.dir/all] 错误 2
gmake[1]: *** 正在等待未完成的任务....
gmake: *** [Makefile:146：all] 错误 2
---
Failed   <<< moveit_task_constructor_visualization [25.5s, exited with code 2]

```

~~命名空间问题，不会修改，暂时先跳过编译此软件包~~

```bash
colcon build --symlink-install --packages-skip moveit_task_constructor_visualization \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOST_ROOT=$BOOST_ROOT \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DBoost_USE_STATIC_LIBS=OFF \
  -DBUILD_TESTING=OFF \
  -DCATKIN_ENABLE_TESTING=OFF
```

编译完成其他包后发现，跳过后rviz内会报错。

### 解决方案

修改文件：
`src/moveit_task_constructor/visualization/motion_planning_tasks/properties/property_factory.cpp`

源代码：

```cpp
rviz_common::properties::Property* result = new rviz::StringProperty(...);
```

修改为：

```cpp
rviz_common::properties::Property* result = new rviz_common::properties::StringProperty(...);
```



## error7

编译moveit_hybrid_planning时耗时过长且无进展，官方教程说明所有包编译时间在20-30分钟，但是一个包已经编译35分钟。原因未知，跳过。

```bash
colcon build --symlink-install --packages-skip moveit_task_constructor_visualization moveit_hybrid_planning \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBOOST_ROOT=$BOOST_ROOT \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DBoost_USE_STATIC_LIBS=OFF \
  -DBUILD_TESTING=OFF \
  -DCATKIN_ENABLE_TESTING=OFF
```

编译完成后暂时未发现有什么问题。

## error8

使用命令`ros2 launch moveit2_tutorials demo.launch.py`启动demo时，rviz内有以下报错：

```bash
The class required for this display, 'moveit_rviz_plugin/PlanningScene', could not be loaded.
Error:
Failed to load library /home/haltija/ws_moveit/install/moveit_ros_visualization/lib/libmoveit_planning_scene_rviz_plugin.so. Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: Could not load library dlopen error: libyaml-cpp.so.0.8: cannot open shared object file: No such file or directory, at /home/lkp/rpmbuild/BUILD/ros-humble-rcutils-5.1.3/src/shared_library.c:99     
```

运行`moveit_planning_scene_rviz_plugin` 插件时无法找到 `libyaml-cpp.so.0.8` 动态库。

包管理器提供的yaml-cpp版本为7.0.x，不满足要求，手动构建并安装于用户目录下：

```bash
wget https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
tar -xzf yaml-cpp-0.8.0.tar.gz  #文件名注意下载的文件
cd yaml-cpp-yaml-cpp-0.8.0
mkdir build
cd build
# 指定安装到用户目录（避免污染系统路径）
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/home/haltija/custom_install/yaml-cpp-0.8/install \
  -DYAML_BUILD_SHARED_LIBS=ON
make -j$(nproc)
make install
```

将以下语句添加到`~/.bashrc`中：

```bash
export YAML_CPP_ROOT=/home/haltija/custom_install/yaml-cpp-0.8/install
export LD_LIBRARY_PATH=$YAML_CPP_ROOT/lib64:$LD_LIBRARY_PATH
export CPLUS_INCLUDE_PATH=$YAML_CPP_ROOT/include:$CPLUS_INCLUDE_PATH
```

