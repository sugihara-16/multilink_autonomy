from pathlib import Path
import shlex,subprocess
n=Path(__file__).resolve().parent;w=n.parent.parents[3];d=w/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir';o=w/'build/mimosa-speed'
f={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
link=shlex.split((d/'link.txt').read_text());libs=link[next(i for i,x in enumerate(link) if x.startswith('-Wl,-rpath')):]
for mode in ['original','optimized']:
 obj=o/(mode+'_test.o')
 if not obj.exists():
  subprocess.run(['c++',*f['CXX_DEFINES'],*([f'-I{o}/include'] if mode=='optimized' else []),*f['CXX_INCLUDES'],'-O3','-DNDEBUG','-fopenmp','-std=c++17','-c',str(n/'test_mimosa_performance.cpp'),'-o',str(obj)],check=True)
 subprocess.run(['c++','-fopenmp',str(obj),'-o',str(o/('test_'+mode)),str(w/'build/active_perception/libmimosa_lib.so'),str(w/'slam_ros1_ws/devel/lib/libgtsam_points.so'),str(w/'slam_ros1_ws/devel/lib/libconfig_utilities.so'),*libs],check=True)

import os,hashlib,json
from time import monotonic
result={};env=os.environ.copy();env['LD_LIBRARY_PATH']=f'{w}/build/active_perception:{w}/slam_ros1_ws/devel/lib:{w}/comparison_ws/deps/lib:/root/ros1_noetic/install_isolated/lib';env['OMP_WAIT_POLICY']='PASSIVE'
for mode in ['original','optimized']:
 data=o/(mode+'.bin');p=subprocess.run([str(o/('test_'+mode)),str(data)],env=env,capture_output=True,text=True,check=True)
 result[mode]=dict(stdout=p.stdout,sha256=hashlib.sha256(data.read_bytes()).hexdigest())
result['byte_identical']=result['original']['sha256']==result['optimized']['sha256']
(o/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
assert result['byte_identical'],result
print(json.dumps(result,indent=2))
