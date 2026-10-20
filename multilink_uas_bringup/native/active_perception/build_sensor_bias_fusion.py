#!/usr/bin/env python3
"""Isolated library with optional sensor-derived mean-reverting IMU bias."""
import hashlib,json,os,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-sensor-bias';out.mkdir(exist_ok=True)
source=WS/'build/mimosa-fixed-gravity/imu_manager.cpp';s=source.read_text()
anchor='#include "mimosa/imu/manager.hpp"';assert s.count(anchor)==1
s=s.replace(anchor,anchor+'\n#include "sensor_bias_model.hpp"')
anchor='  // Prepare logger';assert s.count(anchor)==1
s=s.replace(anchor,'  active_fusion::biasModel().load(pnh);\n'+anchor)
start=s.index('  double sigma_b_a =');end=s.index('\n\n  logger_->info("Added IMU factor',start)
old=s[start:end]
s=s[:start]+'''  auto& model=active_fusion::biasModel();
  if(model.enabled) {
    graph.add(MeanRevertingBiasFactor(B(key_0),B(key_1),model.tau,model.driving,preintegrator_->deltaTij()));
    model.record(ts_1,preintegrator_->deltaTij());
  } else {
'''+old+'\n  }'+s[end:]
generated=out/'imu_manager.cpp';generated.write_text(s)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
obj=out/'imu_manager.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],f'-I{HERE}/performance','-c',str(generated),'-o',str(obj)],check=True)
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
objects={'/imu/manager.cpp.o':obj,'/lidar/geometric.cpp.o':WS/'build/mimosa-compact-prefetch/geometric.o','/lidar/incremental_voxel_map.cpp.o':WS/'build/mimosa-compact-prefetch/incremental_voxel_map.o','/lidar/drpm_registration.cpp.o':WS/'build/active_perception/drpm_registration.o'}
cmd=[next((str(p) for suffix,p in objects.items() if item.endswith(suffix)),item) for item in cmd];subprocess.run(cmd,cwd=d.parents[1],check=True)
test=HERE/'performance/test_mean_reverting_bias.cpp';test_obj=out/'test_mean_reverting_bias.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-O1','-c',str(test),'-o',str(test_obj)],check=True)
node=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir'
link=shlex.split((node/'link.txt').read_text());link[link.index('-o')+1]=str(out/'test_mean_reverting_bias');link=[str(test_obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in link]
subprocess.run(link,cwd=node.parents[1],check=True)
env=dict(os.environ);env['LD_LIBRARY_PATH']=str(WS/'slam_ros1_ws/devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
subprocess.run([str(out/'test_mean_reverting_bias')],env=env,check=True)
paths=[source,generated,test,*[HERE/'performance'/s for s in ['mean_reverting_bias_factor.hpp','sensor_bias_model.hpp']],*objects.values(),out/'libmimosa_lib.so']
(out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
print('BUILT',out,flush=True)
