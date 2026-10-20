#!/usr/bin/env python3
"""Use the previously validated GT flight controller; no SLAM feedback."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
import rclpy
from rclpy.executors import ExternalShutdownException
from dragon_route_flight import RouteFlight

class Flight(RouteFlight):
    def __init__(self,out,route,end):
        super().__init__(out,route,once=True)
        self.plan["disk_reserve_gb"]=1  # Pose/decision logs only; no image or point-cloud recording.
        self.finish_at=min(end,self.duration) if end>0 else self.duration
    def control(self):
        if self.stage=='initialize':
            try:health=json.loads((self.out/'evaluation_status.json').read_text()).get('startup',{})
            except (FileNotFoundError,json.JSONDecodeError):return
            if health and health.get('checked') and not health.get('valid'):
                raise RuntimeError('Invalid preflight ROVIO: displacement exceeds 1 m before simulation 5 s')
            if health and not health.get('checked'):return
        if self.stage=='tracking' and self.route_time>=self.finish_at:
            self.transition('complete');self.status();return
        super().control()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--route',type=Path,required=True);p.add_argument('--end',type=float,default=0);a=p.parse_args()
    rclpy.init();n=Flight(a.output,a.route,a.end)
    try:
        while rclpy.ok() and n.stage!='complete':rclpy.spin_once(n,timeout_sec=.2)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if rclpy.ok():n.status()
        n.csvfile.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
