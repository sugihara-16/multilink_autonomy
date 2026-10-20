"""UAS default UniPilot with a single gimbal camera, ZaB and manual hover."""
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    native=Path(__file__).resolve().parent
    sys.path.insert(0,str(native))
    from unipilot_gimbal import prepare
    value=lambda k:LaunchConfiguration(k).perform(context)
    world=Path(value('world_file'));world_name=ET.parse(world).getroot().find('world').get('name')
    rmf=Path(get_package_share_directory('rmf_gz'));cave=Path(get_package_share_directory('subt_cave_sim'))
    model=prepare(rmf/'resources/robots/rmf_unipilot/model.sdf',value('output'),paired=value('paired')=='true',sensor_profile=value('sensor_profile'),imu_sampling=value('imu_sampling'),diagnostic_sensors=value('diagnostic_sensors')=='true',diagnostic_imu_only=value('diagnostic_imu_only')=='true',camera_far=float(value('camera_far')) if value('camera_far') else None)
    simulation_world=world
    if value('serial_postupdates')=='true' or value('batch_render')=='true':
        # Official Gazebo 8.15 scheduling policy: same systems and physics,
        # avoid one thread/barrier per small PostUpdate callback at every ms.
        root=ET.parse(world).getroot();w=root.find('world')
        if value('serial_postupdates')=='true':
            policy=ET.SubElement(w,'gz:policies',{'xmlns:gz':'https://gazebosim.org'})
            ET.SubElement(policy,'parallel_postupdates').text='false'
        if value('batch_render')=='true':
            plugin=w.find("plugin[@name='gz::sim::systems::Sensors']")
            plugin.set('filename','/root/multilink_autonomy_ws/build/gz-batch-speed/libgz-sim8-sensors-system.so')
        simulation_world=Path(value('output'))/'world_serial.sdf'
        ET.ElementTree(root).write(simulation_world,encoding='unicode',xml_declaration=True)
    topics=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/model/rmf_unipilot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/model/rmf_unipilot/cmd/vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/model/rmf_unipilot/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
        '/rmf_unipilot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/rmf_unipilot/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/rmf_unipilot/cam_gimbal/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/active/gimbal/states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/active/gimbal/yaw@std_msgs/msg/Float64]gz.msgs.Double',
        '/active/gimbal/pitch@std_msgs/msg/Float64]gz.msgs.Double']
    if value('diagnostic_sensors')=='true':
        topics.append('/diagnostic/imu_fast@sensor_msgs/msg/Imu[gz.msgs.IMU')
        topics.extend(['/diagnostic/imu_clean@sensor_msgs/msg/Imu[gz.msgs.IMU','/diagnostic/image_clean@sensor_msgs/msg/Image[gz.msgs.Image','/diagnostic/image_wide@sensor_msgs/msg/Image[gz.msgs.Image','/diagnostic/image_far@sensor_msgs/msg/Image[gz.msgs.Image'])
    if value('bridge_clock')!='true':topics=[t for t in topics if not t.startswith('/clock@')]
    if value('paired')=='true':
        topics.append('/rmf_unipilot/cam_fixed/image@sensor_msgs/msg/Image[gz.msgs.Image')
    # At 1000 Hz, the bridge's default ten-message history is only 10 ms.
    # Preserve bursts after host scheduling delays without changing sensor time.
    bridge_entries=[]
    for spec in topics:
        topic,types=spec.split('@',1)
        delimiter='[' if '[' in types else ']'
        ros_type,gz_type=types.split(delimiter,1)
        depth=2000 if ros_type in ('sensor_msgs/msg/Imu','sensor_msgs/msg/JointState') else 10
        bridge_entries.append(dict(ros_topic_name=topic,gz_topic_name=topic,
            ros_type_name=ros_type,gz_type_name=gz_type,
            direction='GZ_TO_ROS' if delimiter=='[' else 'ROS_TO_GZ',
            publisher_queue=depth,subscriber_queue=depth))
    bridge_config=Path(value('output'))/'gazebo_bridge.yaml'
    bridge_config.write_text(yaml.safe_dump(bridge_entries))
    runtime=Path('/tmp')/f'uas-gazebo-runtime-{os.getuid()}'
    runtime.mkdir(mode=0o700,exist_ok=True);runtime.chmod(0o700)
    sim_share=Path(get_package_share_directory('aerial_robot_simulation'))
    server_env={'XDG_RUNTIME_DIR':str(runtime)}
    if value('batch_render')=='true':
        server_env.update(UAS_BATCH_SENSOR_RENDER='1',LD_LIBRARY_PATH='/root/multilink_autonomy_ws/build/gz-sensors-speed/lib:'+os.environ.get('LD_LIBRARY_PATH',''))
    server=ExecuteProcess(cmd=['gz','sim','-s',*(['-r'] if value('paused')!='true' else []),'--render-engine','ogre2',
                              *(['--headless-rendering'] if value('egl')=='true' else []),
                              '--render-engine-api-backend',value('graphics_api'),str(simulation_world)],output='screen',
                          additional_env=server_env)
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',':'.join([str(world.parent/'models'),str(rmf/'resources'),str(cave),str(cave/'models'),os.environ.get('GZ_SIM_RESOURCE_PATH','')])),
        server,
        RegisterEventHandler(OnProcessExit(target_action=server,on_exit=[Shutdown(reason='Gazebo simulation server exited')])),
        Node(package='ros_gz_sim',executable='create',arguments=['-world',world_name,'-name','rmf_unipilot','-file',str(model),
            '-x',value('spawn_x'),'-y',value('spawn_y'),'-z',value('spawn_z')],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(bridge_config)}],
            remappings=[('/model/rmf_unipilot/odometry','/rmf_unipilot/odom'),('/model/rmf_unipilot/cmd/vel','/rmf_unipilot/cmd/vel'),
                        ('/model/rmf_unipilot/enable','/rmf_unipilot/enable'),('/rmf_unipilot/lidar/scan/points','/rmf_unipilot/lidar/points')],output='screen'),
        ]
    if value('manual_hover')=='true':
        actions.append(ExecuteProcess(cmd=['python3',str(native/'unipilot_hover_manual.py'),'--ros-args','-p','takeoff_height:='+value('takeoff_height')],output='screen'))
    if value('headless')!='true':
        # Use the previously working DRAGON GUI setup and keep its lifecycle
        # separate: a closed/crashed viewer must not stop physics or rqt.
        gui_env={
                'XDG_RUNTIME_DIR':str(runtime),
                'GZ_GUI_PLUGIN_PATH':str(sim_share.parent.parent/'lib')+os.pathsep+os.environ.get('GZ_GUI_PLUGIN_PATH','')}
        if value('gui_software_rendering')=='true':
            # WSLg D3D12 can abort Qt context creation. Apply the workaround
            # only to the viewer; GPU sensor rendering stays on the server.
            gui_env.update(GALLIUM_DRIVER='llvmpipe',LIBGL_ALWAYS_SOFTWARE='1')
        elif os.environ.get('WSL_DISTRO_NAME'):
            # Match the working aerial_robot Gazebo launch's WSL GL profile;
            # the previous UniPilot launch omitted these compatibility settings.
            gui_env.update(GALLIUM_DRIVER='d3d12',LIBGL_ALWAYS_SOFTWARE='0',
                           QT_QPA_PLATFORM='xcb',MESA_GL_VERSION_OVERRIDE='3.3',
                           MESA_GLSL_VERSION_OVERRIDE='330')
        actions.append(ExecuteProcess(cmd=['gz','sim','-g','--render-engine','ogre2',
            '--gui-config',str(sim_share/'config/gazebo_gui.config')],output='screen',additional_env=gui_env))
    return actions


def generate_launch_description():
    defaults=dict(world_file='/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf',headless='true',
        spawn_x='0',spawn_y='0',spawn_z='.5',takeoff_height='1.5',output='/root/uas_native/logs/unipilot-gimbal',
        gui_software_rendering='false',manual_hover='true',paused='false',serial_postupdates='false',egl='false',graphics_api='opengl',paired='false',batch_render='false',bridge_clock='true',camera_far='',sensor_profile='native',imu_sampling='resolved',diagnostic_sensors='false',diagnostic_imu_only='false')
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],OpaqueFunction(function=setup)])
