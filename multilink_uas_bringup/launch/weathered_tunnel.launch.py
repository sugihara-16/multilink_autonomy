"""One-command viewer for the procedural weathered tunnel."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    launch=Path(get_package_share_directory('multilink_uas_bringup'))/'launch/switching_tunnel.launch.py'
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)),launch_arguments={
        'world_file':LaunchConfiguration('world_file').perform(context),
        'camera_pose':'12 0 2 0 0 0.3'}.items())]


def generate_launch_description():
    workspace=Path(os.environ.get('MULTILINK_AUTONOMY_WS','/root/multilink_autonomy_ws'))
    return LaunchDescription([DeclareLaunchArgument('world_file',default_value=str(workspace/'assets/weathered_tunnel_20261024/world.sdf')),OpaqueFunction(function=setup)])
