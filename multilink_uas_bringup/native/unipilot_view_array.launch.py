"""Extend the selected UniPilot launch with four massless fixed view sensors."""
import importlib.util
from pathlib import Path
import sys
import yaml
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NATIVE = Path(__file__).resolve().parent
sys.path.insert(0, str(NATIVE))
sys.path.insert(0, str(NATIVE/'active_perception'))
import unipilot_gimbal
from fixed_view_array import VIEWS, prepare_array

spec = importlib.util.spec_from_file_location('selected_unipilot_launch', NATIVE/'unipilot_gimbal_sim.launch.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
original_setup = base.setup
original_prepare = unipilot_gimbal.prepare


def setup(context):
    unipilot_gimbal.prepare = lambda *a, **k: prepare_array(original_prepare, *a, **k)
    try:
        actions = original_setup(context)
    finally:
        unipilot_gimbal.prepare = original_prepare
    output = Path(LaunchConfiguration('output').perform(context))
    entries = [dict(ros_topic_name=f'/view_array/{name}/raw_image',
                    gz_topic_name=f'/view_array/{name}/raw_image',
                    ros_type_name='sensor_msgs/msg/Image', gz_type_name='gz.msgs.Image',
                    direction='GZ_TO_ROS', publisher_queue=30, subscriber_queue=30)
               for name in VIEWS]
    config = output/'view_array_gazebo_bridge.yaml'
    config.write_text(yaml.safe_dump(entries))
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='view_array_gazebo_bridge', parameters=[{'config_file': str(config)}],
                        output='screen'))
    return actions


base.setup = setup


def generate_launch_description():
    return base.generate_launch_description()
