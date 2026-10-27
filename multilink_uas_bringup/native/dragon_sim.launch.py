"""Attach one base-link LiDAR to the maintained ROS2 DRAGON model."""
from pathlib import Path
import hashlib
import json
import os
import xml.etree.ElementTree as ET
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('dragon'))
    cave = Path(get_package_share_directory('subt_cave_sim'))
    override = LaunchConfiguration('world_file').perform(context)
    world = Path(override) if override else cave/'worlds/darpa_cave_01.sdf'
    world_name = ET.parse(world).getroot().find('world').get('name')
    spawn = [float(LaunchConfiguration('spawn_'+a).perform(context)) for a in 'xyz']
    resources = [str(world.parent/'models'), str(cave), str(cave/'models'), get_package_share_directory('rmf_gz')+'/resources']
    resources += [str(cave/'models'/name/'materials/textures') for name in
                  ['Rescue Randy Sitting', 'Samsung J8 Black', 'JanSport Backpack Red']]
    resources.append(os.environ.get('GZ_SIM_RESOURCE_PATH', ''))
    output = Path(LaunchConfiguration('output').perform(context))
    output.mkdir(parents=True, exist_ok=True)
    (output/'world.json').write_text(json.dumps({
        'world_file': str(world), 'sha256': hashlib.sha256(world.read_bytes()).hexdigest(),
        'world_name': world_name, 'model_name': 'dragon', 'spawn_xyz': spawn,
        'source': 'Explicit world override' if override else 'Unmodified UAS subt_cave_sim/darpa_cave_01.sdf'}, indent=2)+'\n')
    root = ET.fromstring(xacro.process_file(str(share/'robots/quad/dragon.gazebo.xacro'),
        mappings={'robot_name': 'dragon', 'visual_detail': 'balanced', 'body_joint_velocity': '1.0'}).toxml())
    imu = root.find("./gazebo/sensor[@type='imu']")
    if imu is None:
        raise RuntimeError('DRAGON model has no FC IMU')
    # UAS cave already loads the IMU system. Keep the model's magnetometer
    # system (absent from the UAS world), but avoid a second IMU publisher.
    for sensor_node in root.findall('./gazebo/sensor'):
        for plugin in list(sensor_node.findall('plugin')):
            if plugin.get('name') == 'gz::sim::systems::Imu':
                sensor_node.remove(plugin)
    ET.SubElement(imu, 'topic').text = '/dragon/imu/raw'
    ET.SubElement(imu, 'gz_frame_id').text = 'dragon/fc'
    # Existing Livox visual/mass is retained. Optical center is 77 mm above link2.
    sensor = ET.fromstring('''<sensor name="base_lidar" type="gpu_lidar">
      <pose>0.132 0 0.077 0 0 0</pose><always_on>true</always_on><update_rate>10</update_rate>
      <topic>/dragon/lidar/scan</topic><gz_frame_id>dragon/lidar</gz_frame_id>
      <lidar><scan><horizontal><samples>360</samples><resolution>1</resolution>
      <min_angle>-3.14159265359</min_angle><max_angle>3.14159265359</max_angle></horizontal>
      <vertical><samples>32</samples><resolution>1</resolution><min_angle>-0.12217304764</min_angle>
      <max_angle>0.90757121104</max_angle></vertical></scan>
      <range><min>0.1</min><max>30</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><mean>0</mean><stddev>0.03</stddev></noise></lidar></sensor>''')
    ET.SubElement(root,'gazebo',reference='link2').append(sensor)
    vio = LaunchConfiguration('vio').perform(context) == 'true'
    gimbal = LaunchConfiguration('gimbal').perform(context) == 'true'
    articulated = LaunchConfiguration('articulated').perform(context) == 'true'
    end_gimbal = LaunchConfiguration('end_gimbal').perform(context) == 'true'
    if end_gimbal and not articulated:raise ValueError('end_gimbal requires articulated:=true')
    if articulated:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from dragon_articulated import configure
        configure(root, Path(get_package_share_directory('rmf_gz'))/'resources/robots/rmf_unipilot/model.sdf', output)
        if end_gimbal:
            from dragon_gimbal import configure as configure_gimbal
            configure_gimbal(root,output)
    if gimbal and not vio:
        raise ValueError('gimbal requires vio:=true')
    if vio and not articulated:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from dragon_cameras import add_cameras
        add_cameras(root, output, gimbal=gimbal)
    ET.indent(root)
    ET.ElementTree(root).write(output/'dragon_slam.urdf', encoding='unicode')
    cfg = yaml.safe_load((share/'config/Simulation.yaml').read_text())
    params = cfg['/**/sim_param_server']['ros__parameters']
    for name in params:
        if name.startswith('ground_truth_') and ('noise' in name or 'drift' in name):
            params[name] = 0.0
    (output/'Simulation.yaml').write_text(yaml.safe_dump(cfg))
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(resources)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/bringup_launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless'), 'rviz': 'false',
            'robot_description_file': str(output/'dragon_slam.urdf'),
            'sim_param_path': str(output/'Simulation.yaml'), 'world_sdf_file': str(world),
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            **{'spawn_'+a: str(v) for a,v in zip('xyz',spawn)}, 'render_engine': 'ogre2'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='dragon_sensors',
            arguments=['/dragon/imu/raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
                       '/dragon/lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'] +
                      ([f'/dragon/cam_link{i}/image@sensor_msgs/msg/Image[gz.msgs.Image' for i in ((3,) if gimbal else (1,3,4))] if vio and not articulated else []) +
                      (['/dragon/end_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/dragon/articulated/states@sensor_msgs/msg/JointState[gz.msgs.Model'] if articulated else []) +
                      (['/active/gimbal/states@sensor_msgs/msg/JointState[gz.msgs.Model',
                        '/active/gimbal/yaw@std_msgs/msg/Float64]gz.msgs.Double',
                        '/active/gimbal/pitch@std_msgs/msg/Float64]gz.msgs.Double'] if gimbal or end_gimbal else []),
            remappings=[('/dragon/lidar/scan/points','/dragon/lidar/points')],
            parameters=[{'use_sim_time': True}], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world_file', default_value=''),
        DeclareLaunchArgument('spawn_x', default_value='40'), DeclareLaunchArgument('spawn_y', default_value='5'),
        DeclareLaunchArgument('takeoff_height', default_value='1.5'),
        DeclareLaunchArgument('spawn_z', default_value='0.5'),
        DeclareLaunchArgument('gimbal', default_value='false'),
        DeclareLaunchArgument('articulated', default_value='false'),
        DeclareLaunchArgument('end_gimbal', default_value='false'),
        DeclareLaunchArgument('vio', default_value='false'), DeclareLaunchArgument('output'), OpaqueFunction(function=setup)])
