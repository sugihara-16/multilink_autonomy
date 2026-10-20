#!/usr/bin/env bash
set -e
source /root/uas_native/env_ros1.sh
source /root/multilink_autonomy_ws/slam_ros1_ws/devel/setup.bash
if [[ -f /root/multilink_autonomy_ws/comparison_ws/devel/setup.bash ]]; then
  source /root/multilink_autonomy_ws/comparison_ws/devel/setup.bash
fi
export ROS_MASTER_URI=http://127.0.0.1:11376
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export LD_LIBRARY_PATH=/root/multilink_autonomy_ws/comparison_ws/deps/lib:${LD_LIBRARY_PATH:-}
exec "$@"
