#!/usr/bin/env python3
import csv,json,subprocess,sys,tempfile,unittest
from pathlib import Path
class SpeedAudit(unittest.TestCase):
 def test_failed_flight_keeps_final_tracking_hold(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);flight=root/'flight';flight.mkdir()
   (root/'campaign.json').write_text(json.dumps(dict(status='failed',wall_s=1,runs=dict(run1_prefix_active=dict(status='failed',path=str(flight))))))
   with (flight/'route_tracking.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['sim_t','route_t','tracking_error_m']);w.writerows([[1,.1,.01],[1.02,.2,.05],[1.04,.2,.19]])
   subprocess.run([sys.executable,str(Path(__file__).with_name('summarize_performance.py')),str(root)],check=True,stdout=subprocess.DEVNULL)
   result=json.loads((root/'performance_audit.json').read_text());run=result['runs']['run1_prefix_active']
   self.assertEqual(run['tracking_window_s'],[1,1.04]);self.assertEqual(run['tracking_error_max_m'],.19)
   self.assertFalse(result['all_routes_complete']);self.assertFalse(result['timestamp_coverage_complete']);self.assertFalse(result['fivefold_runtime_reached'])
if __name__=='__main__':unittest.main()
