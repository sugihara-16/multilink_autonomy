"""Exercise flight-stage transitions without starting a simulator."""
import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
import rclpy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from articulated_hover_test import Test

class Preparation(unittest.TestCase):
    def test_matched_preparation_holds_second_cycle_and_skips_sweep(self):
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as folder:
                out=Path(folder);n=Test(out,distance=90.,speed=.5,gimbal=True,unipilot_matched=True)
                try:
                    seconds=[100.]
                    n.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=seconds[0]))
                    n.state=5;n.cog=Odometry();n.fc=Odometry()
                    n.fc.pose.pose.position.x=n.cog.pose.pose.position.x=5.
                    n.fc.pose.pose.position.z=n.cog.pose.pose.position.z=2.
                    n.joints=dict(zip(n.names,[0.,1.56]*3+[0.,0.]))
                    n.stage='settle';n.start=91.
                    (out/'evaluation_status.json').write_text(json.dumps(dict(startup=dict(checked=True,valid=True),counts=dict(rovio=40,mimosa=40,truth=40))))
                    for t,stage in [(100.,'initialize'),(101.,'approach'),(102.,'approach'),(114.,'preparation'),(140.,'preparation'),(166.,'active'),(167.,'active'),(354.,'landing')]:
                        seconds[0]=t;n.tick();self.assertEqual(n.stage,stage)
                        if t==140:self.assertEqual(json.loads((out/'route_status.json').read_text())['stage'],'excitation_hold')
                    n.state=0;seconds[0]=355.;n.tick()
                    self.assertTrue(n.done)
                    self.assertNotIn('articulation',[e['stage'] for e in n.events])
                    self.assertTrue(json.loads((out/'flight_result.json').read_text())['passed'])
                finally:n.destroy_node()
        finally:rclpy.shutdown()

if __name__=='__main__':unittest.main()
