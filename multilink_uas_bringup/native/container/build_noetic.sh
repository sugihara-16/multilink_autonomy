#!/usr/bin/env bash
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=noetic ROS_VERSION=1 ROS_PYTHON_VERSION=3 DEBIAN_FRONTEND=noninteractive
cd /root/ros1_noetic
for spec in 'rosconsole rosconsole-log4cxx.patch' 'perception_pcl/pcl_ros pcl-ros-cxx17.patch'; do
 read -r target patchname <<< "$spec"
 if patch -d "src/$target" -p1 --batch --forward --dry-run --silent <"/root/uas_native/$patchname"; then
  patch -d "src/$target" -p1 --batch <"/root/uas_native/$patchname"
 else
  patch -d "src/$target" -p1 -R --batch --dry-run --silent <"/root/uas_native/$patchname"
 fi
done
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys='python3-catkin-pkg python3-rospkg python3-rosdistro'
./src/catkin/bin/catkin_make_isolated --install -j6 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3
