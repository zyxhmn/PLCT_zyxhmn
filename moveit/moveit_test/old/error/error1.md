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



