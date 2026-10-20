"""Manual UniPilot camera inspection; optional legacy DRAGON, rqt angle sliders."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    native = Path(__file__).resolve().parent
    value = lambda name: LaunchConfiguration(name).perform(context)
    output = (Path(value('output')).expanduser().resolve() if value('output')
              else native / 'logs' / (value('robot') + '-gimbal-inspect'))
    unipilot = value('robot') == 'unipilot'
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(native / ('unipilot_gimbal_sim.launch.py' if unipilot else 'dragon_sim.launch.py'))),
        launch_arguments={
            'vio': 'true', 'gimbal': 'true', 'output': str(output),
            **{key: value(key) for key in ('headless', 'world_file', 'spawn_x', 'spawn_y', 'spawn_z', 'takeoff_height', 'gui_software_rendering')},
        }.items()),
        ExecuteProcess(cmd=['python3', str(native / 'active_perception/manual_gimbal.py'), '--robot', value('robot')], output='screen')]
    if not unipilot:
        actions.append(ExecuteProcess(cmd=['python3', str(native / 'zab_controller_ready.py'), '--output', str(output)], output='screen'))
    if value('rqt') == 'true':
        runtime = Path('/tmp') / f'uas-rqt-runtime-{os.getuid()}'
        runtime.mkdir(mode=0o700, exist_ok=True)
        runtime.chmod(0o700)
        env = {'XDG_RUNTIME_DIR': str(runtime)}
        actions += [
            ExecuteProcess(cmd=['python3', str(native / 'active_perception/gimbal_rqt.py')], additional_env=env, output='screen'),
            ExecuteProcess(cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/rmf_unipilot/cam_gimbal/image' if unipilot else '/dragon/cam_link3/image'], additional_env=env, output='screen')]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='unipilot', choices=['unipilot', 'dragon']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rqt', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui_software_rendering', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('world_file', default_value='/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf'),
        DeclareLaunchArgument('spawn_x', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='0'),
        DeclareLaunchArgument('spawn_z', default_value='0.5'),
        DeclareLaunchArgument('takeoff_height', default_value='1.5'),
        DeclareLaunchArgument('output', default_value=''),
        OpaqueFunction(function=setup),
    ])
