#!/usr/bin/env bash
set -e
source /root/uas_native/env_ros1.sh
source /root/multilink_autonomy_ws/comparison_ws/devel/setup.bash
source /root/multilink_autonomy_ws/fast_livo2_official_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11378
export LD_LIBRARY_PATH=/root/multilink_autonomy_ws/fast_livo2_official_ws/devel/lib:/root/multilink_autonomy_ws/comparison_ws/deps/lib:${LD_LIBRARY_PATH:-}
exec "$@"
