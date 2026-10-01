#!/bin/bash
# Use the system ROS1 toolchain; no interactive virtual environment activation.
set -eo pipefail
if [[ ${ROS_VERSION:-} == 2 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/bin:/usr/bin:/bin \
    bash --noprofile --norc "$(readlink -f "${BASH_SOURCE[0]}")" "$@"
fi
native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$native_dir/../../../../slam_ros1_ws"
source /root/ros1_noetic/install_isolated/setup.bash
/root/.venvs/uas/bin/catkin build kindr lightweight_filtering -j2 -p1 --no-status
/root/.venvs/uas/bin/catkin build rovio --no-deps -j1 -p1 --no-status \
  --cmake-args -DROVIO_NCAM=3 -DMAKE_SCENE=OFF --make-args rovio_node
