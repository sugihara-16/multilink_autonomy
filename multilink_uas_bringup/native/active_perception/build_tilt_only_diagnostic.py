#!/usr/bin/env python3
"""Build an isolated diagnostic node that omits only external velocity factors.

ROVIO gravity-direction factors, all state timestamps, and IMU/LiDAR processing
remain identical. This is NOT pure LIO: visual-inertial tilt is still present.
The experiment node and its currently loaded header/library are not modified.
"""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

HERE=Path(__file__).resolve().parent
WS=HERE.parent.parents[3]
out=WS/'build/mimosa-tilt-only-diagnostic'
out.mkdir(exist_ok=True)
original=HERE/'performance/velocity_odometry_manager.hpp'
source=original.read_text()
factor='      factors.add(BodyVelocityFactor(X(0),V(0),measured,gtsam::noiseModel::Gaussian::Covariance(C)));'
assert source.count(factor)==1
source=source.replace(factor,'      // Diagnostic ablation: omit the body-velocity factor only.')
anchor='    bool aligned=false,vio_aligned=false,estimated=true;'
assert source.count(anchor)==1
source=source.replace(anchor,'''    bool diagnostic=false;
    pnh.param("odometry/manager/diagnostic_disable_velocity",diagnostic,false);
    if(!diagnostic)throw std::invalid_argument("This diagnostic binary must explicitly disable velocity factors");
'''+anchor)
source=source.replace('directory+"velocity_factors.csv"','directory+"omitted_velocity_observations.csv"')
header=out/'velocity_odometry_manager.hpp';header.write_text(source)
node=out/'ordered_mimosa_node.cpp'
node.write_text((WS/'build/mimosa-velocity/ordered_mimosa_node.cpp').read_text())
directory=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir'
flags={line.split(' = ')[0]:shlex.split(line.split(' = ')[1])
       for line in (directory/'flags.make').read_text().splitlines() if ' = ' in line}
obj=out/'ordered_mimosa_node.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
                f'-I{HERE}/performance','-DMIMOSA_BODY_VELOCITY','-O1','-c',str(node),'-o',str(obj)],check=True)
binary=out/'mimosa_tilt_only_node'
link=shlex.split((directory/'link.txt').read_text());link[link.index('-o')+1]=str(binary)
link=[str(obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in link]
subprocess.run(link,cwd=directory.parents[1],check=True)
paths=[Path(__file__).resolve(),original,header,node,binary]
(out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,
    files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
print('BUILT',binary,flush=True)
