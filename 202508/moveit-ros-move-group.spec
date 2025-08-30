%global ros_distro humble
%global __strip /bin/true
%global _lto_cflags %{nil}   # 禁用 LTO

%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/%{ros_distro}/.*$
%global __requires_exclude_from ^/opt/ros/%{ros_distro}/.*$

%define RosPkgName      moveit-ros-move-group

Name:           ros-%{ros_distro}-%{RosPkgName}
Version:        2.5.9
Release:        1%{?dist}%{?release_suffix}
Summary:        The move_group node for MoveIt

Url:            http://moveit.ros.org
License:        BSD
Source0:        %{name}_%{version}.orig.tar.gz

Requires: ros-%{ros_distro}-moveit-common
Requires: ros-%{ros_distro}-moveit-core
Requires: ros-%{ros_distro}-moveit-ros-planning
Requires: ros-%{ros_distro}-moveit-ros-occupancy-map-monitor
Requires: ros-%{ros_distro}-rclcpp
Requires: ros-%{ros_distro}-rclcpp-action
Requires: ros-%{ros_distro}-tf2
Requires: ros-%{ros_distro}-tf2-geometry-msgs
Requires: ros-%{ros_distro}-tf2-ros
Requires: ros-%{ros_distro}-pluginlib
Requires: ros-%{ros_distro}-std-srvs
Requires: ros-%{ros_distro}-moveit-kinematics
Requires: ros-%{ros_distro}-ros-workspace

BuildRequires: ros-%{ros_distro}-moveit-common
BuildRequires: ros-%{ros_distro}-moveit-core
BuildRequires: ros-%{ros_distro}-moveit-ros-planning
BuildRequires: ros-%{ros_distro}-moveit-ros-occupancy-map-monitor
BuildRequires: ros-%{ros_distro}-rclcpp
BuildRequires: ros-%{ros_distro}-rclcpp-action
BuildRequires: ros-%{ros_distro}-tf2
BuildRequires: ros-%{ros_distro}-tf2-geometry-msgs
BuildRequires: ros-%{ros_distro}-tf2-ros
BuildRequires: ros-%{ros_distro}-tf2-kdl
BuildRequires: ros-%{ros_distro}-pluginlib
BuildRequires: ros-%{ros_distro}-std-srvs
BuildRequires: ros-%{ros_distro}-ament-cmake
BuildRequires: ros-%{ros_distro}-ros-workspace
BuildRequires: ros-%{ros_distro}-generate-parameter-library

%if 0%{?with_tests}
BuildRequires: ros-%{ros_distro}-moveit-resources-fanuc-moveit-config
BuildRequires: ros-%{ros_distro}-ament-lint-auto
BuildRequires: ros-%{ros_distro}-ament-lint-common
%endif

Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
The move_group node for MoveIt

%prep
%autosetup -p1

%build
# Needed to bootstrap since the ros_workspace package does not yet exist.
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages
export CCACHE_DISABLE=1
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++

# 确保编译选项不被优化
%global _lto_cflags %{nil}
export CFLAGS="%{optflags} -fPIC -fno-function-sections -fno-data-sections"
export CXXFLAGS="%{optflags} -fPIC -fno-function-sections -fno-data-sections -DBOOST_TIMER_ENABLE_DEPRECATED"
export LDFLAGS="-Wl,--no-as-needed -Wl,--no-gc-sections -Wl,--disable-new-dtags"

# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}

%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/%{ros_distro}" \
    -DAMENT_PREFIX_PATH="/opt/ros/%{ros_distro}" \
    -DCMAKE_PREFIX_PATH="/opt/ros/%{ros_distro}" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_SKIP_RPATH=OFF \
    -DCMAKE_SKIP_BUILD_RPATH=OFF \
    -DCMAKE_INSTALL_RPATH="/opt/ros/%{ros_distro}/lib" \
    ..

%make_build -j4

%install
# Needed to bootstrap since the ros_workspace package does not yet exist.
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Needed to bootstrap since the ros_workspace package does not yet exist.
export PYTHONPATH=/opt/ros/%{ros_distro}/lib/python%{python3_version}/site-packages

# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/%{ros_distro}/setup.sh" ]; then . "/opt/ros/%{ros_distro}/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/%{ros_distro}

%changelog
* Mon Jul 21 2025 Zhou Yu <zhouyu.ros@isrc.iscas.ac.cn> - 2.5.9-1
- Update to version 2.5.9

* Thu May 04 2023 Michael Görner me@v4hn.de - 2.5.4-1
- Autogenerated by ros-porting-tools
