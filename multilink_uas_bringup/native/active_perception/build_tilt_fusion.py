#!/usr/bin/env python3
"""Separate experimental node: existing relative factors plus sparse VIO gravity direction."""
import hashlib,json,os,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-tilt';out.mkdir(exist_ok=True)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
for name,source,extra in [('test_tilt_odometry',HERE/'performance/test_tilt_odometry.cpp',[]),
                          ('mimosa_tilt_node',HERE/'performance/ordered_mimosa_node.cpp',['-DMIMOSA_TILT_VIO'])]:
    obj=out/(name+'.o')
    subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],*extra,'-O1','-c',str(source),'-o',str(obj)],check=True)
    cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/name)
    cmd=[str(obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in cmd]
    subprocess.run(cmd,cwd=d.parents[1],check=True)
    if name.startswith('test_'):
        env=dict(os.environ);env['LD_LIBRARY_PATH']=str(WS/'slam_ros1_ws/devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
        subprocess.run([str(out/name)],env=env,check=True)
sources=[HERE/'performance'/n for n in ['ordered_mimosa_node.cpp','ordered_imu_prefix.hpp','tilt_odometry_manager.hpp','test_tilt_odometry.cpp']]
(out/'manifest.json').write_text(json.dumps(dict(source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    binary_sha256=hashlib.sha256((out/'mimosa_tilt_node').read_bytes()).hexdigest(),
    scope='Opt-in ROVIO gravity-direction factor, official GTSAM Pose3AttitudeFactor; no yaw/position measurement, no ground truth. Relative-pose factors unchanged. Shared IMU and temporal correlations not exactly modelled.'),indent=2)+'\n')
