# moveit_test

## 测试平台

- cpu ：Ryzen 5800H 
- 内存 ： 32GB ddr4
- 系统 ：openEuler 24.03

## 安装

### ros

参考[教程](https://openeuler-ros-docs.readthedocs.io/en/latest/tutorials/guide.html)安装ros2

### moveit

参考[教程](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon)

#### 使用mixin安装colcon的ROS2构建系统

```bash
pip3 install -U pytest colcon-common-extensions
pip3 install colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

 使用`export PATH=$PATH:/usr/local/bin:/home/haltija/.local/bin`添加环境变量。

#### 安装cstool

```bash
pip install vcstool
```

#### 创建colcon工作区并下载教程

```bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b humble https://github.com/moveit/moveit2_tutorials
```

接下来，我们将下载 MoveIt 其余部分的源代码:

```bash
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
```

安装rosdep及依赖

```bash
pip install rosdep
rosdep init
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

运行`rosdep update`报错，报错内容见[error](./error/error1.md)，在.bashrc中添加`export ROS_OS_OVERRIDE=centos:8`