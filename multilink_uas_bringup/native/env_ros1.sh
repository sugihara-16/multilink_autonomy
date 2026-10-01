#!/bin/bash
# Source in a clean terminal; do not mix with ROS2's environment.
if [[ ${ROS_VERSION:-1} == 2 ]]; then
  echo 'ROS2 is already loaded. Use a clean env -i shell for ROS1 (see README).' >&2
  return 1
fi
source /root/ros1_noetic/install_isolated/setup.bash
source /root/unified_autonomy_stack/workspaces/ws_gbplanner/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11374 ROS_IP=127.0.0.1
export PATH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/bin:$PATH"
