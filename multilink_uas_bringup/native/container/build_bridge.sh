#!/usr/bin/env bash
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /root/ros1_noetic/install_isolated/setup.bash
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=4
cd /root/unified_autonomy_stack/workspaces/ws_ros1_bridge
bridge_patch=/root/uas_native/container/ros1-bridge-array-size.patch
if ! git -C src/ros1_bridge apply --reverse --check "$bridge_patch" 2>/dev/null; then
 git -C src/ros1_bridge apply --check "$bridge_patch"
 git -C src/ros1_bridge apply "$bridge_patch"
fi
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
