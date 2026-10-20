"""DRPM+MIMOSA+single-camera ROVIO, foreground lifecycle, no automatic bag or GUI."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch.substitutions import LaunchConfiguration

def setup(context):
    native=Path(os.environ.get('UAS_NATIVE_ROOT',str(Path(get_package_share_directory('multilink_uas_bringup'))/'native')))
    value=lambda name:LaunchConfiguration(name).perform(context)
    cmd=['python3',str(native/'active_perception/run.py'),'--mode',value('mode'),
         '--spawn',*[value('spawn_'+a) for a in 'xyz']]
    if value('world_file'):cmd+=['--world',value('world_file')]
    if value('takeoff')=='true':cmd+=['--takeoff']
    return [ExecuteProcess(cmd=cmd,output='screen',sigterm_timeout='20',sigkill_timeout='25')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='active',choices=['active','fixed']),
        DeclareLaunchArgument('world_file',default_value=''),DeclareLaunchArgument('takeoff',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('spawn_x',default_value='40'),DeclareLaunchArgument('spawn_y',default_value='5'),
        DeclareLaunchArgument('spawn_z',default_value='.5'),OpaqueFunction(function=setup)])
