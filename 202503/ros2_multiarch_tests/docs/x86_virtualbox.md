# x86架构下openEuler24.03 安装ros2 humble 测试

## 基本信息

### 测试平台

操作系统： Arch Linux 
内核版本： 6.13.7-arch1-1 (64 位)
处理器： 16 × AMD Ryzen 7 5800H with Radeon Graphics
内存： 31.2 GiB 内存
虚拟机 Virtualbox

### 软件信息

- os:openEuler 24.03 LTS SP1
- 下载地址：https://www.openeuler.org/zh/download/

##  安装

### 系统安装

参考[VMWare安装openEuler24.03操作系统](https://openeuler-ros-docs.readthedocs.io/en/latest/installation/vm-install-oe.html)安装，安装过程与vmware类似。

### ros安装

参考[安装ROS Humble](https://openeuler-ros-docs.readthedocs.io/en/latest/installation/install-ros-humble.html)

## 测试结果

### ros包工具

|        功能        |                      用例                      | 测试结果 |
| :----------------: | :--------------------------------------------: | :------: |
|       创建包       | ros2 pkg create --build-type ament_cmake mypkg |   通过   |
| 查看包中可执行文件 |         ros2 pkg executables turtlesim         |   通过   |
|     列出所有包     |                 ros2 pkg list                  |   通过   |
|   查看包安装路径   |           ros2 pkg prefix turtlesim            |   通过   |
|    包的xml文件     |             ros2 pkg xml turtlesim             |   通过   |

### 运行工具

|   功能   |              用例              | 测试结果 |
| :------: | :----------------------------: | :------: |
| 启动节点 | ros2 run demo_nodes_cpp talker |   通过   |

### 话题工具

|     功能     |                         用例                          | 测试结果 |
| :----------: | :---------------------------------------------------: | :------: |
| 列出所有话题 |                    ros2 topic list                    |   通过   |
| 查看话题信息 |                ros2 topic info /rosout                |   通过   |
| 查看话题类型 |                ros2 topic type /rosout                |   通过   |
| 查找话题类型 |        ros2 topic find rcl_interfaces/msg/Log         |   通过   |
| 话题发布频率 | ros2 run demo_nodes_cpp talker ros2 topic hz /chatte  |   通过   |
| 查看话题带宽 | ros2 run demo_nodes_cpp talker ros2 topic bw /chatter |   通过   |
|   监听话题   |               ros2 topic echo /chatter                |   通过   |

### 参数工具

|          功能          |                      用例                      | 测试结果 |
| :--------------------: | :--------------------------------------------: | :------: |
| 列出当前节点的所有参数 | ros2 run demo_nodes_cpp talker ros2 param list |   通过   |

### 服务工具

|            功能            |       用例        | 测试结果 |
| :------------------------: | :---------------: | :------: |
| 列出当前节点提供的所有服务 | ros2 service list |   通过   |

### 节点工具

|           功能           |          用例          | 测试结果 |
| :----------------------: | :--------------------: | :------: |
| 列出当前系统中所有的节点 |     ros2 node list     |   通过   |
|    获取节点的详细信息    | ros2 node info /talker |   通过   |

### 包记录工具

|        功能        |                 用例                 | 测试结果 |
| :----------------: | :----------------------------------: | :------: |
| 记录所有话题的数据 |          ros2 bag record -a          |   通过   |
|  查看包的详细信息  | ros2 bag info [上一步存储的路径].db3 |   通过   |
|   播放记录的数据   | ros2 bag play [上一步存储的路径].db3 |   通过   |

### 启动工具

|              功能              |                         用例                         | 测试结果 |
| :----------------------------: | :--------------------------------------------------: | :------: |
| 启动一个包含多个节点的启动文件 | ros2 launch demo_nodes_cpp talker_listener.launch.py |   通过   |

### 接口工具

|         功能         |                        用例                        | 测试结果 |
| :------------------: | :------------------------------------------------: | :------: |
| 列出系统中的所有接口 |                ros2 interface list                 |   通过   |
|   查看  包中的接口   |         ros2 interface package action_msgs         |   通过   |
|   显示接口详细内容   | ros2 interface show geometry_msgs/msg/TwistStamped |   通过   |

### ROS通信组件

|      功能       |                             用例                             | 测试结果 |
| :-------------: | :----------------------------------------------------------: | :------: |
|    话题通信     | ros2 run demo_nodes_py talker ros2 run demo_nodes_py listener |   通过   |
|    服务通信     | ros2 run demo_nodes_cpp add_two_ints_server ros2 run demo_nodes_cpp add_two_ints_client |   通过   |
| 测试ROS坐标转换 | ros2 run tf2_ros static_transform_publisher 1 1 1 0 0 0 /base_link /odom ros2 run tf2_ros tf2_echo base_link odom |   通过   |

### turtlesim

|           功能           |          用例          | 测试结果 |
| :----------------------: | :--------------------: | :------: |
| 列出当前系统中所有的节点 |     ros2 node list     |   通过   |
|    获取节点的详细信息    | ros2 node info /talker |   通过   |

