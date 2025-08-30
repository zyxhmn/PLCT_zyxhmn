# 安装

## ROS Humble

参考[教程](https://openeuler-ros-docs.readthedocs.io/en/latest/installation/install-ros-humble.html)安装ros Humble。

使用`dnf install "ros-humble-*" --skip-broken --exclude=ros-humble-generate-parameter-library-example`命令安装ros-humble相关的所有软件包。

## moveit

安装ros humble同时已经安装了moveit相关的包。

参考[教程](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)编译安装。

```bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b humble https://github.com/moveit/moveit2_tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y # 用不了
cd ~/ws_moveit
colcon build --mixin release
```

如果编译过程中出现报错参考[error](202507/bac/moveit_test/docs/tutorials/error.md)。

成功后`source ~/ws_moveit/install/setup.bash`或：

```bash
echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc
```

