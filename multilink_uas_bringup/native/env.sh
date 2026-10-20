#!/bin/bash
# Interactive ROS2 default. ROS1 processes use a clean environment separately.
if [[ ${ROS_VERSION:-2} == 1 ]]; then
  echo 'ROS1 is already loaded; open a fresh terminal for ROS2.' >&2
  return 1
fi
export UAS_NATIVE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export MULTILINK_AUTONOMY_WS="$(cd "$UAS_NATIVE_ROOT/../../../.." && pwd -P)"
if [[ ${MULTILINK_ROS2_ENV_LOADED:-} == "$UAS_NATIVE_ROOT" && ${MULTILINK_ROS2_ENV_REVISION:-} == 2 ]]; then
  return 0
fi
source /opt/ros/humble/setup.bash
for ws in ws_sim ws_cbf ws_rl; do
 source /root/unified_autonomy_stack/workspaces/$ws/install/local_setup.bash
done
# ROS2 aerial robots and the ROS2 side of the ROS1 bridge.
# local_setup avoids sourcing their historical build-time underlays.
source /root/ros2/aerial_robot_base_ws/install/local_setup.bash
source /root/unified_autonomy_stack/workspaces/ws_ros1_bridge/install/local_setup.bash
export ROS_DOMAIN_ID=74 ROS_MASTER_URI=http://127.0.0.1:11374 ROS_IP=127.0.0.1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$UAS_NATIVE_ROOT/fastdds.xml"
export GZ_PARTITION=uas_native GZ_VERSION=harmonic
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA QT_X11_NO_MITSHM=1
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}
if [[ -f "$MULTILINK_AUTONOMY_WS/install/local_setup.bash" ]]; then
  source "$MULTILINK_AUTONOMY_WS/install/local_setup.bash"
fi
export MULTILINK_ROS2_ENV_LOADED="$UAS_NATIVE_ROOT"
export MULTILINK_ROS2_ENV_REVISION=2
