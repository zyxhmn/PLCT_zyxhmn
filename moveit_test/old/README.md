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

运行`rosdep update`报错，报错内容见[error1](./error/error1.md)。原因为rosdep无法识别操作系统。

在.bashrc中添加`export ROS_OS_OVERRIDE=centos:8` 后，运行 `rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y`报错，报错内容见[error2](./error/error2.md)。

1. **软件包未找到**：
   - `boost-python3-devel`、`ros-humble-py-binding-tools` 和 `urdfdom-headers-devel` 在当前的仓库中无法找到匹配的版本。
2. **依赖冲突**：
   - `ros-humble-ros2-control` 依赖于 `ros-humble-ros2controlcli`，但后者无法安装，因为它需要 `python3-pygraphviz`，而该依赖项在系统中不存在。

使用`rosdep check -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO`命令查看所缺少的依赖项。输出内容如下：

```bash
[haltija@localhost src]$ rosdep check -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO
rosdep detected OS: [centos] aliasing it to: [rhel]
System dependencies have not been satisfied:
dnf     boost-python%{python3_pkgversion}-devel
dnf     ros-humble-ros2-control
dnf     ros-humble-py-binding-tools
dnf     urdfdom-headers-devel
```

