#!/bin/bash
set -eo pipefail
# Interactive terminals now inherit ROS2; isolate ROS1 build/check processes.
if [[ ${ROS_VERSION:-} == 2 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" LANG="${LANG:-C.UTF-8}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc "$(readlink -f "${BASH_SOURCE[0]}")" "$@"
fi
uas_native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source /root/ros1_noetic/install_isolated/setup.bash
cd /root/unified_autonomy_stack/workspaces/ws_gbplanner
patchfile=${uas_native_dir}/grid-map-onetbb.patch
if patch -d src/mapping/grid_map -p1 --batch --forward --dry-run --silent < "$patchfile"; then
  patch -d src/mapping/grid_map -p1 --batch < "$patchfile"
else
  patch -d src/mapping/grid_map -p1 -R --batch --dry-run --silent < "$patchfile"
fi
/root/.venvs/uas/bin/catkin config --extend /root/ros1_noetic/install_isolated --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF -DCATKIN_ENABLE_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
/root/.venvs/uas/bin/catkin build gbplanner pci_general robot_bringup --no-status -j4 -p1
