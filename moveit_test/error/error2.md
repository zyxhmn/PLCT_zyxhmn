# rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

```bash
[haltija@localhost src]$ rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
rosdep detected OS: [centos] aliasing it to: [rhel]
executing command [sudo -H dnf --assumeyes --setopt=strict=0 install boost-python3-devel urdfdom-headers-devel ros-humble-py-binding-tools freeglut-devel ros-humble-ros2-control]
[sudo] haltija 的密码：
Last metadata expiration check: 0:42:49 ago on 2025年03月22日 星期六 17时30分48秒.
No match for argument: boost-python3-devel
No match for argument: urdfdom-headers-devel
No match for argument: ros-humble-py-binding-tools
Dependencies resolved.

 Problem: package ros-humble-ros2-control-2.25.2-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-ros2controlcli, but none of the providers can be installed
  - 冲突的请求
  - nothing provides python3-pygraphviz needed by ros-humble-ros2controlcli-2.25.2-1.oe2403.x86_64 from openEulerROS-humble
==============================================================================================================
 Package                           Architecture   Version                   Repository                   Size
==============================================================================================================
Installing:
 freeglut-devel                    x86_64         3.4.0-2.oe2403sp1         everything                   19 k
Skipping packages with broken dependencies:
 ros-humble-ros2-control           x86_64         2.25.2-1.oe2403           openEulerROS-humble         9.6 k
 ros-humble-ros2controlcli         x86_64         2.25.2-1.oe2403           openEulerROS-humble          32 k

Transaction Summary
==============================================================================================================
Install  1 Package
Skip     2 Packages

Total download size: 19 k
Installed size: 56 k
Downloading Packages:
freeglut-devel-3.4.0-2.oe2403sp1.x86_64.rpm                                   3.6 kB/s |  19 kB     00:05    
--------------------------------------------------------------------------------------------------------------
Total                                                                         1.7 kB/s |  19 kB     00:11     
Running transaction check
Transaction check succeeded.
Running transaction test
Transaction test succeeded.
Running transaction
  Preparing        :                                                                                      1/1 
  Installing       : freeglut-devel-3.4.0-2.oe2403sp1.x86_64                                              1/1 
  Running scriptlet: freeglut-devel-3.4.0-2.oe2403sp1.x86_64                                              1/1 
  Verifying        : freeglut-devel-3.4.0-2.oe2403sp1.x86_64                                              1/1 

Installed:
  freeglut-devel-3.4.0-2.oe2403sp1.x86_64                                                                     
Skipped:
  ros-humble-ros2-control-2.25.2-1.oe2403.x86_64       ros-humble-ros2controlcli-2.25.2-1.oe2403.x86_64      

Complete!
ERROR: the following rosdeps failed to install
  dnf: Failed to detect successful installation of [boost-python%{python3_pkgversion}-devel]
  dnf: Failed to detect successful installation of [urdfdom-headers-devel]
  dnf: Failed to detect successful installation of [ros-humble-py-binding-tools]
  dnf: Failed to detect successful installation of [ros-humble-ros2-control]

```

