#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
import rclpy
from rclpy.executors import ExternalShutdownException
from dragon_hover import Hover

class ActiveHover(Hover):
    def save(self):
        super().save()
        path=self.output/'flight_slam_result.json';report=json.loads(path.read_text())
        report['estimator_inputs']=['LiDAR','IMU','single-camera ROVIO with measured gimbal rotation']
        report['registration']='adopted DRPM official update + radial noise'
        path.write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rclpy.init();n=ActiveHover(a.output,False)
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
