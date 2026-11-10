#!/usr/bin/env python3
"""Link existing verified batched fusion and four-DoF official-rule DRPM objects.
No factor weights, kernel, or sensor settings are modified by this build.
"""
import hashlib,json,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-batched-drpm';out.mkdir(exist_ok=True)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
objects={'/graph/manager.cpp.o':WS/'build/mimosa-batched-vision/graph_manager.o',
'/imu/manager.cpp.o':WS/'build/mimosa-sensor-bias/imu_manager.o',
'/lidar/geometric.cpp.o':WS/'build/mimosa-compact-prefetch/geometric.o',
'/lidar/incremental_voxel_map.cpp.o':WS/'build/mimosa-compact-prefetch/incremental_voxel_map.o',
'/lidar/drpm_registration.cpp.o':WS/'build/mimosa-drpm-four-dof/drpm_registration.o'}
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
cmd=[next((str(p) for suffix,p in objects.items() if s.endswith(suffix)),s) for s in cmd]
subprocess.run(cmd,cwd=d.parents[1],check=True)
paths=[Path(__file__).resolve(),*objects.values(),out/'libmimosa_lib.so']
(out/'manifest.json').write_text(json.dumps({'scope':__doc__,'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}},indent=2)+'\n')
print(out)
