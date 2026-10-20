"""ZaB with independent LIO/VIO and RViz; flight and recording are opt-in."""
import os,json,sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,SetEnvironmentVariable,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    value=lambda k:LaunchConfiguration(k).perform(context)
    native=Path(os.environ.get('UAS_NATIVE_ROOT',str(Path(get_package_share_directory('multilink_uas_bringup'))/'native'))).resolve()
    assets=Path(value('assets'));world=assets/'zab_harmonic.sdf'
    if not world.exists():raise RuntimeError(f'Run python3 {native}/setup_zab.py --destination {assets} first')
    sys.path.insert(0,str(native))
    from manager import state
    if state():raise RuntimeError('Another managed simulation is active; stop it before launching ZaB')
    resources=str(assets/'models')+':'+os.environ.get('GZ_SIM_RESOURCE_PATH','')
    actions=[SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',resources)]
    if value('robot')=='true':
        command=['/usr/bin/python3',str(native/'manager.py'),'foreground',('dragon_compare' if value('estimators')=='true' else 'dragon_record'),
                 '--headless','true','--rviz',value('rviz'),'--record',value('record'),
                 '--flight','none','--world-file',str(world),
                 '--spawn',*[value('spawn_'+a) for a in 'xyz']]
        if value('route'):
            route=Path(value('route')).resolve()
            plan=json.loads((route/'route.json').read_text())
            command[command.index('--flight')+1]='route'
            command[command.index('--spawn')+1:]=[str(v) for v in plan['spawn_xyz']]
            command += ['--route-dir',str(route),'--once']
        server=ExecuteProcess(cmd=command, output='screen',
                              sigterm_timeout='20',sigkill_timeout='25')
        actions += [server,RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='DRAGON supervisor exited'))]))]
    else:
        command=['gz','sim','-r','-s','--render-engine','ogre2',str(world)]
        proc=ExecuteProcess(cmd=command,output='screen',sigterm_timeout='10',sigkill_timeout='15')
        actions += [proc,RegisterEventHandler(OnProcessExit(target_action=proc,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
                    Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]
    if value('headless')=='false':
        actions.append(ExecuteProcess(cmd=['gz','sim','-g','--render-engine','ogre2'],
            additional_env={'LIBGL_ALWAYS_SOFTWARE':'1','LP_NUM_THREADS':'2'},output='screen'))
    return actions

def generate_launch_description():
    default=str(Path(os.environ.get('MULTILINK_AUTONOMY_WS','/root/multilink_autonomy_ws'))/'assets/zab')
    return LaunchDescription([DeclareLaunchArgument('assets',default_value=default),
        DeclareLaunchArgument('headless',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('estimators',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('record',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('route',default_value='',description='Validated one-shot route directory; empty means no flight'),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('robot',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('spawn_x',default_value='0'),DeclareLaunchArgument('spawn_y',default_value='0'),
        DeclareLaunchArgument('spawn_z',default_value='0.5'),OpaqueFunction(function=setup)])
