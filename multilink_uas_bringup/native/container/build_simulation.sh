#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic CMAKE_BUILD_PARALLEL_LEVEL=4 DEBIAN_FRONTEND=noninteractive
cd /root/ros2/aerial_robot_base_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys='gazebo_ros gazebo_ros_control gazebo_dev micro_ros_agent gz-gui8 gz-sim8 gz-math7 gz-plugin2 gz-transport13'
colcon build --symlink-install --packages-up-to dragon gz_ros2_control --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cd /root/unified_autonomy_stack/workspaces/ws_sim
ln -sfn ../../robot_bringup src/robot_bringup
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source /root/ros2/aerial_robot_base_ws/install/local_setup.bash
source /root/unified_autonomy_stack/workspaces/ws_sim/install/local_setup.bash
cd /root/multilink_autonomy_ws
colcon build --symlink-install --packages-select multilink_uas_bringup --cmake-args -DBUILD_TESTING=OFF
python3 /root/uas_native/active_perception/build_performance.py --jobs 4
python3 /root/uas_native/active_perception/build_serial_stop.py
python3 /root/uas_native/container/build_gazebo_runtime.py
