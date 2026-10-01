#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=3
uas=/root/unified_autonomy_stack
for ws in ws_sim ws_cbf ws_gbplanner; do
  target="$uas/workspaces/$ws/src/robot_bringup"
  if [ -d "$target" ] && [ ! -L "$target" ]; then
    [ ! -L "$target/robot_bringup" ] || unlink "$target/robot_bringup"
    rmdir "$target"
  fi
  ln -sfnT ../../robot_bringup "$target"
done
for ws in ws_sim ws_cbf; do
  cd "$uas/workspaces/$ws"
  colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
 done
cd "$uas/workspaces/ws_rl"
colcon build --packages-select ros2_numpy --symlink-install --cmake-args -DBUILD_TESTING=OFF
