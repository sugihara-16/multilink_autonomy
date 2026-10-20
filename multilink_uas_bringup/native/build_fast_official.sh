#!/usr/bin/env bash
# Run under a clean environment through official_fast_ros1.sh.
set -euo pipefail
ws=/root/multilink_autonomy_ws/fast_livo2_official_ws
comparison=/root/multilink_autonomy_ws/comparison_ws
# Sources must already be extracted from the pinned official Git archives.
# No patches are applied. C++17 is required by the installed ROS logging library.
cmake -S "$ws/src/rpg_vikit/vikit_common" -B "$ws/build_vikit_common" \
  -DCMAKE_PROJECT_INCLUDE=/root/uas_native/comparison/official_pcl_abi.cmake -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DSophus_DIR="$comparison/Sophus/build" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$comparison/deps/lib" -DCATKIN_DEVEL_PREFIX="$ws/devel"
cmake --build "$ws/build_vikit_common" -j2 --target vikit_common
cmake -S "$ws/src/rpg_vikit/vikit_ros" -B "$ws/build_vikit_ros" \
  -DCMAKE_PROJECT_INCLUDE=/root/uas_native/comparison/official_pcl_abi.cmake -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DSophus_DIR="$comparison/Sophus/build" \
  -Dvikit_common_DIR="$ws/devel/share/vikit_common/cmake" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$comparison/deps/lib" -DCATKIN_DEVEL_PREFIX="$ws/devel"
cmake --build "$ws/build_vikit_ros" -j2 --target vikit_ros
cmake -S "$ws/src/FAST-LIVO2" -B "$ws/build" \
  -DCMAKE_PROJECT_INCLUDE=/root/uas_native/comparison/official_pcl_abi.cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSophus_DIR="$comparison/Sophus/build" \
  -Dvikit_common_DIR="$ws/devel/share/vikit_common/cmake" \
  -Dvikit_ros_DIR="$ws/devel/share/vikit_ros/cmake" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$comparison/deps/lib" -DCATKIN_DEVEL_PREFIX="$ws/devel"
cmake --build "$ws/build" -j2
