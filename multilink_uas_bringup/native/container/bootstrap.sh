#!/usr/bin/env bash
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
mkdir -p /root/setup-logs /root/multilink_autonomy_ws/build
ln -sfn /root/multilink_autonomy_ws/src/multilink_autonomy/multilink_uas_bringup/native /root/uas_native
apt-get update
apt-get install -y curl gnupg wget git git-lfs ripgrep build-essential cmake ninja-build pkg-config python3-pip python3-venv python3-numpy python3-scipy python3-matplotlib python3-opencv python3-yaml python3-vcstool python3-rosdep python3-pil libeigen3-dev libboost-all-dev libpcl-dev libopencv-dev libgoogle-glog-dev libgflags-dev libyaml-cpp-dev libfmt-dev libtbb-dev liblog4cxx-dev libtinyxml2-dev libtinyxml-dev liburdfdom-dev libconsole-bridge-dev libbz2-dev liblz4-dev libxml2-dev xvfb mesa-utils xauth sudo rsync
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' >/etc/apt/sources.list.d/gazebo-stable.list
apt-get update
apt-get install -y gz-harmonic libgz-sim8-dev libgz-rendering8-dev libgz-sensors8-dev libgz-physics7-dev libgz-gui8-dev ros-humble-ros-gzharmonic ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-rviz2 ros-humble-cv-bridge ros-humble-tf-transformations ros-humble-gazebo-msgs ros-humble-joint-state-publisher ros-humble-robot-state-publisher
python3 -m pip install trimesh==4.4.9 pycollada==0.8
python3 -m venv --system-site-packages /root/.venvs/uas
/root/.venvs/uas/bin/pip install catkin-tools empy==3.3.4 rospkg catkin-pkg transformations
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi
rosdep update --include-eol-distros
# Existing guards check this filesystem; in Linux it is the same runtime volume.
mkdir -p /mnt
ln -sfn /root /mnt/c
