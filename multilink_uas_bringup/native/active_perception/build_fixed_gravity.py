#!/usr/bin/env python3
"""Honor MIMOSA's gravity selection in an isolated library; keep the speed baseline intact."""
import hashlib,json,os,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-fixed-gravity';out.mkdir(exist_ok=True)
base=WS/'comparison_ws';d=base/'build/mimosa/CMakeFiles/mimosa_lib.dir'
source=WS/'build/mimosa-speed/imu_manager.cpp';s=source.read_text()
replacements={
'''      nav_state = propagation_preintegrator_->predict(
        propagation_base_state_.navState(), propagation_base_state_.imuBias(),
        propagation_base_state_.gravity());''':'''      nav_state = config_.preintegration.use_estimated_gravity
        ? propagation_preintegrator_->predict(propagation_base_state_.navState(),
            propagation_base_state_.imuBias(),propagation_base_state_.gravity())
        : propagation_preintegrator_->predict(propagation_base_state_.navState(),
            propagation_base_state_.imuBias());''',
'''  graph.add(gtsam::ImuFactorWithGravity(
    X(key_0), V(key_0), X(key_1), V(key_1), B(key_0), G(0), *preintegrator_));''':'''  if (config_.preintegration.use_estimated_gravity) {
    graph.add(gtsam::ImuFactorWithGravity(
      X(key_0), V(key_0), X(key_1), V(key_1), B(key_0), G(0), *preintegrator_));
  } else {
    graph.add(gtsam::ImuFactor(
      X(key_0), V(key_0), X(key_1), V(key_1), B(key_0), *preintegrator_));
  }''',
'''    ns_1 = preintegrator_->predict(state_0.navState(), state_0.imuBias(), state_0.gravity());''':'''    ns_1 = config_.preintegration.use_estimated_gravity
      ? preintegrator_->predict(state_0.navState(),state_0.imuBias(),state_0.gravity())
      : preintegrator_->predict(state_0.navState(),state_0.imuBias());'''
}
for old,new in replacements.items():
    if s.count(old)!=1:raise ValueError('Unexpected IMU source: '+old)
    s=s.replace(old,new)
modified=out/'imu_manager.cpp';modified.write_text(s)
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
obj=out/'imu_manager.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',str(modified),'-o',str(obj)],check=True)
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
objects={'/imu/manager.cpp.o':obj,
    '/lidar/geometric.cpp.o':WS/'build/mimosa-compact-prefetch/geometric.o',
    '/lidar/incremental_voxel_map.cpp.o':WS/'build/mimosa-compact-prefetch/incremental_voxel_map.o',
    '/lidar/drpm_registration.cpp.o':WS/'build/active_perception/drpm_registration.o'}
cmd=[next((str(p) for suffix,p in objects.items() if item.endswith(suffix)),item) for item in cmd]
subprocess.run(cmd,cwd=d.parents[1],check=True)
test=HERE/'performance/test_fixed_gravity.cpp';test_obj=out/'test_fixed_gravity.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-O1','-c',str(test),'-o',str(test_obj)],check=True)
node=base/'build/mimosa/CMakeFiles/mimosa_node.dir'
link=shlex.split((node/'link.txt').read_text());link[link.index('-o')+1]=str(out/'test_fixed_gravity')
link=[str(test_obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in link]
subprocess.run(link,cwd=node.parents[1],check=True)
env=dict(os.environ);env['LD_LIBRARY_PATH']=str(WS/'slam_ros1_ws/devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
subprocess.run([str(out/'test_fixed_gravity')],env=env,check=True)
files=[source,modified,test,*objects.values(),out/'libmimosa_lib.so']
(out/'manifest.json').write_text(json.dumps(dict(scope='Only honor use_estimated_gravity in IMU factor creation and both prediction paths. False uses configured constant gravity; true retains original variable-gravity behavior. Same compact-prefetch LiDAR objects.',sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}),indent=2)+'\n')
