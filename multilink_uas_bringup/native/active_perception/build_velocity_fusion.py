#!/usr/bin/env python3
"""Build a separate sparse velocity/tilt node without changing current trials."""
import hashlib,json,os,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-velocity';out.mkdir(exist_ok=True)
original=HERE/'performance/ordered_mimosa_node.cpp';source=original.read_text()
old='#ifdef MIMOSA_GRAVITY_TRANSLATION';assert source.count(old)==1
source=source.replace(old,'#ifdef MIMOSA_BODY_VELOCITY\n#include "velocity_odometry_manager.hpp"\nusing VioManager=VelocityOdometryManager;\n#elif defined(MIMOSA_GRAVITY_TRANSLATION)')
node=out/'ordered_mimosa_node.cpp';node.write_text(source)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
for name,src,extra in [('test_body_velocity',HERE/'performance/test_body_velocity.cpp',[]),('mimosa_velocity_node',node,['-DMIMOSA_BODY_VELOCITY'])]:
    obj=out/(name+'.o')
    subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],f'-I{HERE}/performance',*extra,'-O1','-c',str(src),'-o',str(obj)],check=True)
    cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/name)
    cmd=[str(obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in cmd]
    subprocess.run(cmd,cwd=d.parents[1],check=True)
    if name.startswith('test_'):
        env=dict(os.environ);env['LD_LIBRARY_PATH']=str(WS/'slam_ros1_ws/devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
        subprocess.run([str(out/name)],env=env,check=True)
paths=[original,node,*[HERE/'performance'/s for s in ['ordered_imu_prefix.hpp','body_velocity_factor.hpp','velocity_odometry_manager.hpp','test_body_velocity.cpp']],out/'mimosa_velocity_node']
(out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
print('BUILT',out,flush=True)
