"""Launch the native supervisor in the foreground with isolated ROS environments."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    native = os.environ.get('UAS_NATIVE_ROOT', os.path.join(
        get_package_share_directory('multilink_uas_bringup'), 'native'))
    supervisor = ExecuteProcess(
        cmd=['/usr/bin/python3', os.path.join(native, 'manager.py'),
             'foreground', 'dragon_vio',
             '--land-after-check', LaunchConfiguration('land_after_check'),
             '--headless', LaunchConfiguration('headless'),
             '--rviz', LaunchConfiguration('rviz')],
        output='screen', sigterm_timeout='20', sigkill_timeout='25')
    return LaunchDescription([
        DeclareLaunchArgument('land_after_check', default_value='false', choices=['true', 'false'],
                              description='Land after 30 seconds of validated hover; default holds position'),
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false'],
                              description='Do not open the Gazebo GUI; GPU rendering stays enabled'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        RegisterEventHandler(OnProcessExit(target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='UAS supervisor exited'))])),
        supervisor,
    ])
