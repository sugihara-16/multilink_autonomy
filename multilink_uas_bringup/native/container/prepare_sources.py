"""Apply recorded patches without touching the host aerial-robot workspace."""
import json, shutil, subprocess
from pathlib import Path
N=Path('/root/uas_native');W=Path('/root/multilink_autonomy_ws');A=Path('/root/ros2/aerial_robot_base_ws/src')
def patch(root,file):
    if subprocess.run(['git','-C',str(root),'apply','--reverse','--check',str(file)],capture_output=True).returncode==0:return
    subprocess.run(['git','-C',str(root),'apply','--check',str(file)],check=True)
    subprocess.run(['git','-C',str(root),'apply',str(file)],check=True)
for root,file in [(W/'slam_ros1_ws/src/mimosa','mimosa-imu-wait.patch'),(W/'slam_ros1_ws/src/rovio','rovio-three-camera.patch'),(W/'slam_ros1_ws/src/rovio/lightweight_filtering','rovio-lwf-cxx17.patch'),(A/'aerial_robot_base','dragon-bringup-options.patch'),(A/'aerial_robot_base','dragon-rotor-tf.patch'),(A/'gz_ros2_control','dragon-control-description-retry.patch')]:patch(root,N/file)
patch(A/'aerial_robot_nerve',N/'container/spinal-simulation-clock.patch')
patch(A/'aerial_robot_base',N/'container/dragon-simulation-clock.patch')
link=W/'slam_ros1_ws/src/lightweight_filtering'
if not link.exists():link.symlink_to('rovio/lightweight_filtering',target_is_directory=True)
# Only the comparison MIMOSA is needed here; FAST-LIVO2 is outside this trial.
C=W/'comparison_ws/src/mimosa';C.parent.mkdir(parents=True,exist_ok=True)
if not C.exists():
    shutil.copytree(W/'slam_ros1_ws/src/mimosa/mimosa',C)
    subprocess.run(['patch','-p1','-i',str(N/'comparison/mimosa-drpm.patch')],cwd=C,check=True)
for name in ['mimosa-odometry-covariance.patch','mimosa-drpm-noise.patch','mimosa-drpm-official.patch']:patch(C,N/'comparison'/name)
for name in ['drpm_kernel.hpp','drpm_pose_factor.hpp','drpm_registration.hpp','drpm.hpp']:shutil.copyfile(N/'comparison'/name,C/'include/mimosa/lidar'/name)
shutil.copytree(N/'comparison/official_drpm',C/'include/mimosa/lidar/official_drpm',dirs_exist_ok=True)
shutil.copyfile(N/'comparison/drpm_registration.cpp',C/'src/lidar/drpm_registration.cpp')
print('Recorded source patches applied')
