"""Finite articulated-camera validation with Gazebo; Ctrl+C stops all children."""
import os,sys,time
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess
from launch.substitutions import LaunchConfiguration

def setup(context):
    native=Path(os.environ.get('UAS_NATIVE_ROOT','/root/uas_native')).resolve()
    value=lambda n:LaunchConfiguration(n).perform(context)
    output=value('output') or str(native/'logs'/('dragon-articulated-'+time.strftime('%Y%m%d-%H%M%S')))
    args=[sys.executable,str(native/'active_perception/run_articulated_dragon.py'),'--output',output,
          '--mode',value('mode'),'--distance',value('distance'),'--speed',value('speed'),'--domain',value('domain'),'--port',value('port')]
    if value('world_file'):args.extend(['--world',value('world_file')])
    args.append('--headless' if value('headless').lower() in ('true','1') else '--no-headless')
    return [ExecuteProcess(cmd=args,output='screen',sigterm_timeout='25',sigkill_timeout='30')]

def generate_launch_description():
    defaults={'output':'','mode':'active','distance':'90','speed':'0.5','domain':'89','port':'11489','world_file':'','headless':'false'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
