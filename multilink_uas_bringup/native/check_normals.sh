#!/bin/bash
set -eo pipefail
# Interactive terminals now inherit ROS2; isolate ROS1 build/check processes.
if [[ ${ROS_VERSION:-} == 2 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" LANG="${LANG:-C.UTF-8}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc "$(readlink -f "${BASH_SOURCE[0]}")" "$@"
fi
cd "$(dirname "$0")"
source ./env_ros1.sh
c++ -std=c++17 check_normals.cpp -o check_normals $(pkg-config --cflags --libs grid_map_filters)
exec ./check_normals
