#!/usr/bin/env python3
"""Compare map snapshots/searches and ICP outputs against the original code."""
import argparse,hashlib,json,os,shlex,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--scratch',action='store_true');p.add_argument('--pruned',action='store_true');p.add_argument('--cached',action='store_true');p.add_argument('--neighbors',action='store_true');p.add_argument('--prefetch',action='store_true');a=p.parse_args()
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/('build/mimosa-compact-prefetch' if a.prefetch else 'build/mimosa-compact-neighbors' if a.neighbors else 'build/mimosa-compact-cached' if a.cached or a.neighbors else 'build/mimosa-compact-pruned' if a.pruned else 'build/mimosa-compact-scratch' if a.scratch else 'build/mimosa-compact')
a.neighbors=a.neighbors or a.prefetch
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
f={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
link=shlex.split((d/'link.txt').read_text());libs=link[next(i for i,x in enumerate(link) if x.startswith('-Wl,-rpath')):]
obj=out/'factor_test.o'
subprocess.run(['c++',*f['CXX_DEFINES'],*(['-DUAS_IVOX_AABB=1','-DUAS_IVOX_STATIC5=1'] if a.pruned or a.neighbors else []),*(['-DUAS_IVOX_NEIGHBOR_CACHE=1'] if a.neighbors else []),*(['-DUAS_IVOX_PREFETCH=1'] if a.prefetch else []),f'-I{out}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O3','-DNDEBUG','-fopenmp','-std=c++17','-c',str(HERE/('test_mimosa_cache.cpp' if a.cached or a.neighbors else 'test_mimosa_performance.cpp')),'-o',str(obj)],check=True)
subprocess.run(['c++','-fopenmp',str(obj),'-o',str(out/'test_factor'),str(out/'libmimosa_lib.so'),str(WS/'slam_ros1_ws/devel/lib/libgtsam_points.so'),str(WS/'slam_ros1_ws/devel/lib/libconfig_utilities.so'),*libs],check=True)
original_exe=WS/'build/mimosa-speed/test_original'
if a.cached or a.neighbors:
 original_exe=out/'test_original_cache';original_obj=out/'original_cache.o'
 subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],'-O3','-DNDEBUG','-fopenmp','-std=c++17','-c',str(HERE/'test_mimosa_cache.cpp'),'-o',str(original_obj)],check=True)
 subprocess.run(['c++','-fopenmp',str(original_obj),'-o',str(original_exe),str(WS/'build/active_perception/libmimosa_lib.so'),str(WS/'slam_ros1_ws/devel/lib/libgtsam_points.so'),str(WS/'slam_ros1_ws/devel/lib/libconfig_utilities.so'),*libs],check=True)
env=os.environ.copy();env['OMP_WAIT_POLICY']='PASSIVE'
common=f'{WS}/build/active_perception:{WS}/slam_ros1_ws/devel/lib:{WS}/comparison_ws/deps/lib:/root/ros1_noetic/install_isolated/lib'
result={}
for mode,exe,path in [('original',original_exe,common),('candidate',out/'test_factor',str(out)+':'+common)]:
 env['LD_LIBRARY_PATH']=path;data=out/(mode+'_factor.bin')
 r=subprocess.run([str(exe),str(data)],env=env,capture_output=True,text=True,check=True)
 result[mode]=dict(stdout=r.stdout,sha256=hashlib.sha256(data.read_bytes()).hexdigest())
result['byte_identical']=result['original']['sha256']==result['candidate']['sha256']
(out/'factor_validation.json').write_text(json.dumps(result,indent=2)+'\n');assert result['byte_identical'],result
print(json.dumps(result,indent=2))
