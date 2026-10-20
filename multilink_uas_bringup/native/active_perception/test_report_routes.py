#!/usr/bin/env python3
"""Check evaluator on known rigid frame offset and a known linear drift."""
import json,tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from report_routes import evaluate

class Evaluation(unittest.TestCase):
    def test_initial_alignment_and_drift(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);t=np.arange(0,15.001,.05);r=np.maximum(0,t-5);pos=np.column_stack([r,np.sin(r/4),r*0]);q=Rotation.from_euler('z',r*.01)
            frame=Rotation.from_euler('xyz',[.2,-.3,.8]);offset=np.array([12.,-7.,3.])
            def save(name,xyz,quat):
                np.savetxt(p/(name+'.csv'),np.column_stack([t*1e9,xyz,quat]),delimiter=',',header='stamp_ns,x,y,z,qx,qy,qz,qw',comments='')
            save('truth',pos,q.as_quat());save('rovio',frame.inv().apply(pos-offset),(frame.inv()*q).as_quat())
            drift=np.column_stack([r*.1,t*0,t*0]);save('mimosa',frame.inv().apply(pos+drift-offset),(frame.inv()*q).as_quat())
            np.savetxt(p/'route_tracking.csv',np.column_stack([t,r,t*0]),delimiter=',',header='sim_t,route_t,tracking_error_m',comments='')
            np.savetxt(p/'encoders.csv',np.column_stack([t,t*0,t*0]),delimiter=',',header='t,yaw,pitch',comments='')
            (p/'mimosa').mkdir();np.savetxt(p/'mimosa/lidar_manager_odometry.log',t[::2,None])
            (p/'protocol.json').write_text(json.dumps(dict(route_end=10.,drpm_fusion=False,seed=1)))
            (p/'runtime.json').write_text(json.dumps(dict(status='route_complete')))
            (p/'decisions.jsonl').write_text(json.dumps(dict(reason='fixed_baseline',compute_ms=0))+'\n')
            r=evaluate(p)['estimators'];self.assertLess(r['rovio']['position_rmse_m'],1e-12)
            self.assertLess(r['rovio']['rotation_rmse_deg'],1e-12)
            expected=np.sqrt(np.mean((np.arange(.1,9.999,.1)-.1)**2))*.1
            self.assertAlmostEqual(r['mimosa']['position_rmse_m'],expected,places=10)
if __name__=='__main__':unittest.main()
