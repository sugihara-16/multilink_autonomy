#!/usr/bin/env python3
"""Expose camera angle commands as bounded ROS2 parameters for rqt sliders."""
import math
import argparse
from pathlib import Path
import sys

import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float64


class ManualGimbal(Node):
    def __init__(self, robot='unipilot'):
        super().__init__('camera_gimbal_manual')
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from unipilot_gimbal import GIMBAL_LIMITS_DEG
        self.limits = GIMBAL_LIMITS_DEG if robot == 'unipilot' else {'yaw': 120.0, 'pitch': 30.0}
        self.publishers_by_axis = {}
        for axis, limit in self.limits.items():
            self.declare_parameter(axis + '_deg', 0.0, ParameterDescriptor(
                description=f'Camera {axis} target relative to its mounting orientation, in degrees',
                floating_point_range=[FloatingPointRange(
                    from_value=-limit, to_value=limit, step=0.0)]))
            self.publishers_by_axis[axis] = self.create_publisher(
                Float64, '/active/gimbal/' + axis, 10)
        self.add_on_set_parameters_callback(self.validate)
        self.create_timer(0.1, self.publish_angles)
        self.get_logger().info('In rqt Reconfigure select /camera_gimbal_manual; yaw_deg and pitch_deg are angle targets.')

    def validate(self, parameters):
        for parameter in parameters:
            if parameter.name in ('yaw_deg', 'pitch_deg'):
                limit = self.limits[parameter.name.removesuffix('_deg')]
                if not isinstance(parameter.value, float) or not math.isfinite(parameter.value) or abs(parameter.value) > limit:
                    return SetParametersResult(successful=False, reason=f'{parameter.name} must be a finite double within +/-{limit} degrees')
        return SetParametersResult(successful=True)

    def publish_angles(self):
        for axis, publisher in self.publishers_by_axis.items():
            publisher.publish(Float64(data=math.radians(self.get_parameter(axis + '_deg').value)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['unipilot', 'dragon'], default='unipilot')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ManualGimbal(args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
