# 入门

## 安装 ROS 2 和 colcon

ros humble 安装参考[官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

安装完成后`echo 'source /opt/ros/jazzy/setup.bash' > ~/.bashrc`。

安装 [rosdep](http://wiki.ros.org/rosdep) 来安装系统依赖项：

```
sudo apt install python3-rosdep
```

安装 ROS 2 后，请确保您拥有最新的软件包：

```
sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
```

安装 [vcstool](https://index.ros.org/d/python3-vcstool/) ：

```
sudo apt install python3-vcstool
```

## 创建 Colcon 工作区并下载教程

```
mkdir -p ~/ws_moveit/src
```

### 下载 MoveIt 源代码和教程 

进入您的 Colcon 工作区并拉取 MoveIt 教程源，其中 `<branch>` 可以是 ROS Humble 的 `humble` ，也可以是最新版本教程的 `main` ：

```
cd ~/ws_moveit/src
git clone -b <branch> https://github.com/moveit/moveit2_tutorials
```

接下来我们将下载 MoveIt 其余部分的源代码：

```
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
```

### 构建您的 Colcon 工作区 

以下步骤将从 Debian 安装您工作区中尚未存在的任何软件包依赖项。此步骤将安装 MoveIt 及其所有依赖项：

```
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

下一个命令将配置您的 Colcon 工作区：

```
cd ~/ws_moveit
colcon build --mixin release
```

此处可能耗时较长。完成后`echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc`