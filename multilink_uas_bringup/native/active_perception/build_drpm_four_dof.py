#!/usr/bin/env python3
"""Isolated DRPM library honoring the existing four-DoF LiDAR restriction.

The main fusion library is untouched. This is a constrained extension using
the official Eq.24 update, not an unmodified official six-DoF implementation.
"""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

HERE=Path(__file__).resolve().parent
WS=HERE.parent.parents[3]
out=WS/'build/mimosa-drpm-four-dof'
out.mkdir(exist_ok=True)
header=HERE/'performance/drpm_four_dof_kernel.hpp'
test=HERE/'performance/test_drpm_four_dof.cpp'
test_binary=out/'test_drpm_four_dof'
kernel_root=WS/'comparison_ws/src/mimosa/include/mimosa/lidar'
test_inputs=[header,test,*[kernel_root/name for name in ['drpm_kernel.hpp','drpm.hpp','official_drpm/degeneracy.h']]]
if not test_binary.exists() or any(p.stat().st_mtime_ns>test_binary.stat().st_mtime_ns for p in test_inputs):
    subprocess.run(['c++','-std=c++17','-O1','-I/usr/include/eigen3',
        '-I'+str(WS/'comparison_ws/src/mimosa/include'),'-I'+str(HERE/'performance'),
        str(test),'-o',str(test_binary)],check=True)
subprocess.run([str(test_binary)],check=True)
source=WS/'build/active_perception/drpm_registration.cpp'
s=source.read_text()
anchor='#include "mimosa/lidar/drpm_kernel.hpp"'
assert s.count(anchor)==1
s=s.replace(anchor,anchor+'\n#include "drpm_four_dof_kernel.hpp"')
anchor=' std::string status="ok";DrpmKernelResult result;size_t correspondences=0;'
assert s.count(anchor)==1
s=s.replace(anchor,anchor+'''
 auto solve=[&](const Linearization& l){
   if(cfg.reg_4_dof) return drpmFourDofKernel(l.H,l.rhs,l.points,l.normals,l.weights,
     l.normal_covariances,l.point_covariances,cfg.lidar_point_noise_std_dev,
     cfg.drpm_snr_factor,cfg.drpm_registration_mode,T_W_L.rotation().unrotate(V3::UnitZ()));
   return drpmKernel(l.H,l.rhs,l.points,l.normals,l.weights,l.normal_covariances,l.point_covariances,
     cfg.lidar_point_noise_std_dev,cfg.drpm_snr_factor,cfg.drpm_registration_mode);
 };
''')
for indent in ['       ','     ']:
    anchor='result=drpmKernel(l.H,l.rhs,l.points,l.normals,l.weights,l.normal_covariances,l.point_covariances,\n'+indent+'cfg.lidar_point_noise_std_dev,cfg.drpm_snr_factor,cfg.drpm_registration_mode);'
    assert s.count(anchor)==1
    s=s.replace(anchor,'result=solve(l);')
generated=out/'drpm_registration.cpp'
generated.write_text(s)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
flags={line.split(' = ')[0]:shlex.split(line.split(' = ')[1])
       for line in (d/'flags.make').read_text().splitlines() if ' = ' in line}
obj=out/'drpm_registration.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
    '-I'+str(HERE/'performance'),'-I'+str(HERE),'-I'+str(WS/'build/active_perception'),
    '-c',str(generated),'-o',str(obj)],check=True)
cmd=shlex.split((d/'link.txt').read_text())
cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
objects={'/imu/manager.cpp.o':WS/'build/mimosa-sensor-bias/imu_manager.o',
         '/lidar/geometric.cpp.o':WS/'build/mimosa-compact-prefetch/geometric.o',
         '/lidar/incremental_voxel_map.cpp.o':WS/'build/mimosa-compact-prefetch/incremental_voxel_map.o',
         '/lidar/drpm_registration.cpp.o':obj}
cmd=[next((str(p) for suffix,p in objects.items() if item.endswith(suffix)),item) for item in cmd]
subprocess.run(cmd,cwd=d.parents[1],check=True)
paths=[source,generated,header,test,Path(__file__).resolve(),*objects.values(),out/'libmimosa_lib.so']
(out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,
    changes='Only drpm_registration.o differs from the sensor-bias library. reg_4_dof=false uses the old kernel unchanged. reg_4_dof=true uses the four-variable restriction. IMU, graph, ordinary ICP and map objects unchanged.',
    sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
print('BUILT',out,flush=True)
