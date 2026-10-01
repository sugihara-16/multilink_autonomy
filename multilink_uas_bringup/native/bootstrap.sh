#!/bin/bash
set -eo pipefail
uas_native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export DEBIAN_FRONTEND=noninteractive
# Preserve WSL's shared Windows executable handler across distro startups.
ln -sf /dev/null /etc/systemd/system/systemd-binfmt.service
ln -sf /dev/null /etc/systemd/system/binfmt-support.service
apt-get update
apt-get install -y curl gnupg lsb-release locales ripgrep software-properties-common sudo git git-lfs build-essential cmake python3-pip python3-venv rsync tmux mesa-utils
locale-gen en_US.UTF-8
update-locale LANG=en_US.UTF-8
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update
apt-get install -y ros-humble-desktop ros-humble-ros-gzharmonic gz-harmonic python3-colcon-common-extensions python3-rosdep python3-vcstool ros-humble-mavros-msgs ros-humble-control-msgs ros-humble-tf-transformations ros-humble-tf2-tools ros-humble-xacro ros-humble-joint-state-publisher ros-humble-ros2-control ros-humble-ros2-controllers libgoogle-glog-dev libspdlog-dev libpcap-dev libgeographic-dev geographiclib-tools libgl1-mesa-dri libgl1-mesa-glx libgz-cmake3-dev libgz-common5-dev libgz-fuel-tools9-dev libgz-gui8-dev libgz-math7-dev libgz-msgs10-dev libgz-physics7-dev libgz-plugin2-dev libgz-rendering8-dev libgz-sensors8-dev libgz-tools2-dev libgz-transport13-dev libgz-utils2-dev libsdformat14-dev python3-dev libtool automake autoconf
mkdir -p ${uas_native_dir}/logs
chown -R root:root ${uas_native_dir}
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

# Applies on the next start of this distro; do not restart other WSL distros.
python3 - <<'CONF'
import configparser
p = configparser.ConfigParser()
p.read('/etc/wsl.conf')
if not p.has_section('boot'): p.add_section('boot')
if not p.has_section('user'): p.add_section('user')
p['boot']['systemd'] = 'false'
p['user']['default'] = 'root'
with open('/etc/wsl.conf', 'w') as f: p.write(f)
CONF
cp -n /etc/skel/.bashrc /etc/skel/.profile /etc/skel/.bash_logout /root/
chown root:root /root /root/.bashrc /root/.profile /root/.bash_logout
