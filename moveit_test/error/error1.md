# rosdep update

```bash
[haltija@localhost src]$ rosdep init
ERROR: default sources list file already exists:
        /etc/ros/rosdep/sources.list.d/20-default.list
Please delete if you wish to re-initialize
[haltija@localhost src]$ sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
[haltija@localhost src]$ sudo rosdep init
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run

        rosdep update

[haltija@localhost src]$ rosdep update
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Skip end-of-life distro "bouncy"
Skip end-of-life distro "crystal"
Skip end-of-life distro "dashing"
Skip end-of-life distro "eloquent"
Skip end-of-life distro "foxy"
Skip end-of-life distro "galactic"
Skip end-of-life distro "groovy"
Add distro "humble"

ERROR: Rosdep experienced an error: Could not detect OS, tried ['zorin', 'windows', 'nixos', 'clearlinux', 'ubuntu', 'slackware', 'rocky', 'rhel', 'raspbian', 'qnx', 'pop', 'osx', 'sailfishos', 'tizen', 'conda', 'oracle', 'opensuse', 'opensuse', 'opensuse', 'opensuse', 'opensuse', 'openembedded', 'neon', 'mx', 'mint', 'linaro', 'gentoo', 'funtoo', 'freebsd', 'fedora', 'elementary', 'elementary', 'debian', 'cygwin', 'euleros', 'centos', 'manjaro', 'buildroot', 'arch', 'amazon', 'alpine', 'almalinux']
Please go to the rosdep page [1] and file a bug report with the stack trace below.
[1] : http://www.ros.org/wiki/rosdep

rosdep version: 0.25.1

Traceback (most recent call last):
  File "/usr/local/lib/python3.11/site-packages/rosdep2/main.py", line 144, in rosdep_main
    exit_code = _rosdep_main(args)
                ^^^^^^^^^^^^^^^^^^
  File "/usr/local/lib/python3.11/site-packages/rosdep2/main.py", line 448, in _rosdep_main
    return _no_args_handler(command, parser, options, args)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/local/lib/python3.11/site-packages/rosdep2/main.py", line 457, in _no_args_handler
    return command_handlers[command](options)
           ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/local/lib/python3.11/site-packages/rosdep2/main.py", line 670, in command_update
    update_sources_list(success_handler=update_success_handler,
  File "/usr/local/lib/python3.11/site-packages/rosdep2/sources_list.py", line 514, in update_sources_list
    rosdep_data = get_gbprepo_as_rosdep_data(dist_name)
                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/local/lib/python3.11/site-packages/rosdep2/gbpdistro_support.py", line 148, in get_gbprepo_as_rosdep_data
    ctx = create_default_installer_context()
          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/local/lib/python3.11/site-packages/rosdep2/__init__.py", line 89, in create_default_installer_context
    m.register_platforms(context)
  File "/usr/local/lib/python3.11/site-packages/rosdep2/platforms/arch.py", line 52, in register_platforms
    register_manjaro(context)
  File "/usr/local/lib/python3.11/site-packages/rosdep2/platforms/arch.py", line 57, in register_manjaro
    (os_name, os_version) = context.get_os_name_and_version()
                            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/local/lib/python3.11/site-packages/rosdep2/installers.py", line 113, in get_os_name_and_version
    os_name = self.os_detect.get_name()
              ^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/usr/lib/python3.11/site-packages/rospkg/os_detect.py", line 636, in get_name
    self.detect_os()
  File "/usr/lib/python3.11/site-packages/rospkg/os_detect.py", line 606, in detect_os
    raise OsNotDetected("Could not detect OS, tried %s" % attempted)
rospkg.os_detect.OsNotDetected: Could not detect OS, tried ['zorin', 'windows', 'nixos', 'clearlinux', 'ubuntu', 'slackware', 'rocky', 'rhel', 'raspbian', 'qnx', 'pop', 'osx', 'sailfishos', 'tizen', 'conda', 'oracle', 'opensuse', 'opensuse', 'opensuse', 'opensuse', 'opensuse', 'openembedded', 'neon', 'mx', 'mint', 'linaro', 'gentoo', 'funtoo', 'freebsd', 'fedora', 'elementary', 'elementary', 'debian', 'cygwin', 'euleros', 'centos', 'manjaro', 'buildroot', 'arch', 'amazon', 'alpine', 'almalinux']

```





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

