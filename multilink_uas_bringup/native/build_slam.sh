#!/bin/bash
set -eo pipefail
if [[ ${ROS_VERSION:-} == 2 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" LANG="${LANG:-C.UTF-8}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc "$(readlink -f "${BASH_SOURCE[0]}")" "$@"
fi
native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$native_dir/../../../../slam_ros1_ws"
source /root/ros1_noetic/install_isolated/setup.bash
/root/.venvs/uas/bin/catkin config --extend /root/ros1_noetic/install_isolated --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DCONFIG_UTILS_BUILD_TESTS=OFF -DCONFIG_UTILS_BUILD_DEMOS=OFF -DBUILD_TESTING=OFF
/root/.venvs/uas/bin/catkin build mimosa -j2 -p1 --no-status
