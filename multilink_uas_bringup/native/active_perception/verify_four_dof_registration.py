#!/usr/bin/env python3
"""Test the actual compact-prefetch ICP header, without changing any library."""
import hashlib,json,os,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-four-dof-check';out.mkdir(exist_ok=True)
selected=WS/'build/mimosa-compact-prefetch';d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
link=shlex.split((d/'link.txt').read_text());libs=link[next(i for i,x in enumerate(link) if x.startswith('-Wl,-rpath')):]
source=HERE/'performance/test_four_dof_registration.cpp';obj=out/'test.o';exe=out/'test'
subprocess.run(['c++',*flags['CXX_DEFINES'],'-DUAS_IVOX_AABB=1','-DUAS_IVOX_STATIC5=1','-DUAS_IVOX_NEIGHBOR_CACHE=1','-DUAS_IVOX_PREFETCH=1',f'-I{selected}/include',f'-I{HERE}',*flags['CXX_INCLUDES'],'-O1','-DNDEBUG','-fopenmp','-std=c++17','-c',str(source),'-o',str(obj)],check=True)
subprocess.run(['c++','-fopenmp',str(obj),'-o',str(exe),str(selected/'libmimosa_lib.so'),str(WS/'slam_ros1_ws/devel/lib/libgtsam_points.so'),str(WS/'slam_ros1_ws/devel/lib/libconfig_utilities.so'),*libs],check=True)
env=dict(os.environ);env['OMP_WAIT_POLICY']='PASSIVE';env['LD_LIBRARY_PATH']=f'{selected}:{WS}/build/active_perception:{WS}/slam_ros1_ws/devel/lib:{WS}/comparison_ws/deps/lib:/root/ros1_noetic/install_isolated/lib'
r=subprocess.run([str(exe)],env=env,capture_output=True,text=True,check=True);print(r.stdout)
paths=[source,selected/'include/mimosa/lidar/geometric_factor.hpp',selected/'libmimosa_lib.so',exe]
(out/'result.json').write_text(json.dumps(dict(result=r.stdout,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
