#!/usr/bin/env bash
set -eo pipefail
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /root/ros1_noetic/install_isolated/setup.bash
ws=/root/multilink_autonomy_ws
native=/root/uas_native
catkin=/root/.venvs/uas/bin/catkin
cd "$ws/slam_ros1_ws"
"$catkin" config --extend /root/ros1_noetic/install_isolated --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DCONFIG_UTILS_BUILD_TESTS=OFF -DCONFIG_UTILS_BUILD_DEMOS=OFF -DBUILD_TESTING=OFF
"$catkin" build mimosa kindr lightweight_filtering -j4 -p1 --no-status
"$catkin" build rovio --no-deps -j2 -p1 --no-status --cmake-args -DROVIO_NCAM=3 -DMAKE_SCENE=OFF --make-args rovio_node
source "$ws/slam_ros1_ws/devel/setup.bash"
cd "$ws/comparison_ws"
"$catkin" config --extend "$ws/slam_ros1_ws/devel" --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
"$catkin" build mimosa -j4 -p1 --no-status
source "$ws/comparison_ws/devel/setup.bash"
cd "$native/active_perception"
python3 build.py fif rovio mimosa
python3 build_rovio_joint_geometry.py
python3 build_articulated_rovio.py
python3 build_mimosa_performance.py
python3 build_compact_mimosa.py --prefetch
python3 build_fixed_gravity.py
python3 build_sensor_bias_fusion.py
python3 build_velocity_fusion.py
python3 build_batched_fusion.py
python3 build_drpm_four_dof.py
python3 build_batched_drpm.py
python3 build_drpm_support.py
python3 build_drpm_view_information.py
python3 build_native_adapters.py
python3 build_reliable_bridge.py
python3 build_zmq_retry.py
python3 build_no_crash_dump.py
