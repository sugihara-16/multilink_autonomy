"""One-command viewer for the generated 100 m scanned-texture tunnels."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    workspace = Path(os.environ.get('MULTILINK_AUTONOMY_WS', '/root/multilink_autonomy_ws'))
    seed = LaunchConfiguration('seed').perform(context)
    world = workspace / 'assets/ostopovice_100m_20261024' / ('layout' + seed) / 'world.sdf'
    launch = Path(get_package_share_directory('multilink_uas_bringup')) / 'launch/switching_tunnel.launch.py'
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)),
            launch_arguments={'world_file': str(world), 'camera_pose': '15 0 2 0 0 0.6'}.items())]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('seed', default_value='24101',
        choices=['24101','24102','24103']), OpaqueFunction(function=setup)])
