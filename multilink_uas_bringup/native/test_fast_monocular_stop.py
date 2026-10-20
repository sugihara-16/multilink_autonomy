"""Check that the early-stop rule cannot label a short transient as divergence."""
import csv,tempfile
from pathlib import Path
import numpy as np
import check_fast_monocular_failures as c
with tempfile.TemporaryDirectory() as tmp:
 c.LOG=Path(tmp);d=c.LOG/'trial';d.mkdir();gt=c.LOG/'fast_multicamera_all/trial';gt.mkdir(parents=True)
 def save(path,t,x):
  with path.open('w') as f:
   w=csv.writer(f);w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw']);w.writerows((int(a),float(b),0,0,0,0,0,1) for a,b in zip(t,x))
 ts=np.arange(0,20000000001,50000000,dtype=np.int64);save(gt/'truth_full.csv',ts,np.zeros(len(ts)));window=dict(start_ns=0,end_ns=20000000000)
 for end,level,expected in [(14950000000,101,False),(15000000000,101,True),(15000000000,100,False)]:
  t=ts[ts<=end];x=np.where(t>=10000000000,level,0);save(d/'odometry_full.csv',t,x);assert bool(c.detect_divergence(d,window))==expected
 t=ts[ts<=16000000000];x=np.where((t>=10000000000)&(t<14500000000),101,0);save(d/'odometry_full.csv',t,x);assert c.detect_divergence(d,window) is None
 with (d/'odometry_full.csv').open('a') as f:f.write('16050000000,101,')
 assert c.detect_divergence(d,window) is None
 print('PASS: 4.95 s transient rejected; 5 s sustained accepted; exact threshold rejected; recovered transient rejected; incomplete output row ignored')
