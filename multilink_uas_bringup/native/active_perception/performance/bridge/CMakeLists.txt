cmake_minimum_required(VERSION 3.16)
project(uas_reliable_bridge LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros1_bridge REQUIRED)
find_ros1_package(roscpp REQUIRED)
add_executable(reliable_bridge ../reliable_bridge.cpp)
ament_target_dependencies(reliable_bridge rclcpp ros1_bridge)
target_include_directories(reliable_bridge PRIVATE ${ros1_roscpp_INCLUDE_DIRS})
# Link the same bundled ROS1 ABI as libros1_bridge, not the separate Noetic
# workspace libraries (which have different SONAMEs and logging globals).
list(GET ros1_bridge_LIBRARIES 0 BRIDGE_LIBRARY)
get_filename_component(BRIDGE_LIBDIR "${BRIDGE_LIBRARY}" DIRECTORY)
target_link_libraries(reliable_bridge
  "${BRIDGE_LIBDIR}/libroscpp.so.4d"
  "${BRIDGE_LIBDIR}/libxmlrpcpp.so.3d")
# ROS1 transitive dependencies live outside the system library search path.
# Keep the isolated binary runnable from the ROS2-only launcher environment.
target_link_options(reliable_bridge PRIVATE -Wl,--disable-new-dtags)
