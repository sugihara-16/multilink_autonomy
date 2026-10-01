#!/bin/bash
set -eo pipefail
# Interactive terminals now inherit ROS2; isolate ROS1 build/check processes.
if [[ ${ROS_VERSION:-} == 2 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" LANG="${LANG:-C.UTF-8}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc "$(readlink -f "${BASH_SOURCE[0]}")" "$@"
fi
uas_native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export ROS_DISTRO=noetic ROS_VERSION=1 ROS_PYTHON_VERSION=3
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
cd /root/ros1_noetic
patchfile=${uas_native_dir}/rosconsole-log4cxx.patch
if patch -d src/rosconsole -p1 --batch --forward --dry-run --silent < "$patchfile"; then
  patch -d src/rosconsole -p1 --batch < "$patchfile"
else
  patch -d src/rosconsole -p1 -R --batch --dry-run --silent < "$patchfile"
fi
patchfile=${uas_native_dir}/pcl-ros-cxx17.patch
if patch -d src/perception_pcl/pcl_ros -p1 --batch --forward --dry-run --silent < "$patchfile"; then
  patch -d src/perception_pcl/pcl_ros -p1 --batch < "$patchfile"
else
  patch -d src/perception_pcl/pcl_ros -p1 -R --batch --dry-run --silent < "$patchfile"
fi
rosdep update --include-eol-distros --rosdistro noetic
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys="python3-catkin-pkg python3-rospkg python3-rosdistro"
./src/catkin/bin/catkin_make_isolated --install -j4 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3
