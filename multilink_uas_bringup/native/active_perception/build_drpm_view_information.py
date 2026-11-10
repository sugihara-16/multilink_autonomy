#!/usr/bin/env python3
"""Keep six-DoF LiDAR geometry in view scoring, independently of factor DoF.

The factor still updates heading and translation only. Excluding roll/pitch
from that update is not evidence that LiDAR cannot observe roll/pitch.
This library changes telemetry only, not registration or graph information.
"""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parents[3]
out = WS / 'build/mimosa-batched-drpm-support-view'
out.mkdir(exist_ok=True)
source = (WS / 'build/mimosa-batched-drpm-support/drpm_registration.cpp').read_text()
replacements = {
    ' double last_translation=0,last_rotation=0;':
    ' Linearization final_linearization;\n double last_translation=0,last_rotation=0;',
    '   auto l=linearize(T_W_L,points,target,cfg);correspondences=l.points.size();\n   if(correspondences<6)throw std::runtime_error("insufficient_final_correspondences");':
    '   final_linearization=linearize(T_W_L,points,target,cfg);\n   const auto& l=final_linearization;correspondences=l.points.size();\n   if(correspondences<6)throw std::runtime_error("insufficient_final_correspondences");',
    ' publishActiveDrpm(stamp,T_W_L,result);': ''' // Match the ordinary-MIMOSA view diagnostic: full measured geometry.
 // The four-DoF factor below remains exactly unchanged.
 try {
   const auto& l=final_linearization;
   auto view=drpmKernel(l.H,l.rhs,l.points,l.normals,l.weights,
     l.normal_covariances,l.point_covariances,cfg.lidar_point_noise_std_dev,
     cfg.drpm_snr_factor,1);
   publishActiveDrpm(stamp,T_W_L,view);
 } catch(const std::runtime_error& e) {
   ROS_WARN_THROTTLE(5,"View-only LiDAR diagnosis: %s",e.what());
 }''',
}
for old, new in replacements.items():
    if source.count(old) != 1:
        raise RuntimeError('Unexpected source anchor: ' + old)
    source = source.replace(old, new)
generated = out / 'drpm_registration.cpp'
generated.write_text(source)
d = WS / 'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
flags = {line.split(' = ')[0]: shlex.split(line.split(' = ')[1])
         for line in (d / 'flags.make').read_text().splitlines() if ' = ' in line}
obj = out / 'drpm_registration.o'
subprocess.run(['c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
                '-I' + str(HERE / 'performance'), '-I' + str(HERE), '-c', str(generated), '-o', str(obj)], check=True)
objects = {
    '/graph/manager.cpp.o': WS / 'build/mimosa-batched-vision/graph_manager.o',
    '/imu/manager.cpp.o': WS / 'build/mimosa-sensor-bias/imu_manager.o',
    '/lidar/geometric.cpp.o': WS / 'build/mimosa-compact-prefetch/geometric.o',
    '/lidar/incremental_voxel_map.cpp.o': WS / 'build/mimosa-compact-prefetch/incremental_voxel_map.o',
    '/lidar/drpm_registration.cpp.o': obj,
}
cmd = shlex.split((d / 'link.txt').read_text())
cmd[cmd.index('-o') + 1] = str(out / 'libmimosa_lib.so')
cmd = [next((str(p) for suffix, p in objects.items() if word.endswith(suffix)), word) for word in cmd]
subprocess.run(cmd, cwd=d.parents[1], check=True)
paths = [Path(__file__).resolve(), generated, *objects.values(), out / 'libmimosa_lib.so']
(out / 'manifest.json').write_text(json.dumps(dict(scope=__doc__, sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}), indent=2) + '\n')
print(out)
