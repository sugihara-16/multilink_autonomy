#!/usr/bin/env python3
"""Generalize the DRPM adapter's hardcoded five-point plane support.
Uses the unmodified official normal estimator at N=5,20,50; the normal
uncertainty rejection threshold and nominal sigma are unchanged.
"""
from pathlib import Path
import hashlib,json,shlex,subprocess
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-batched-drpm-support';out.mkdir(exist_ok=True)
s=(WS/'build/mimosa-drpm-four-dof/drpm_registration.cpp').read_text()
replacements={
 'std::vector<size_t> ids(5);std::vector<double> distances(5);':'const int count=cfg.num_corres_points;std::vector<size_t> ids(count);std::vector<double> distances(count);',
 'target->knn_search(pw,5,ids,distances)':'target->knn_search(pw,count,ids,distances)',
 'Eigen::Matrix<double,3,5> q;':'Eigen::Matrix<double,3,Eigen::Dynamic> q(3,count);',
 'for(int j=0;j<5;++j)':'for(int j=0;j<count;++j)',
 'centered.transpose()*centered/5.0':'centered.transpose()*centered/double(count)',
 'std::vector<M3> cs(5);':'std::vector<M3> cs(count);',
 'cfg.num_corres_points!=5':'(cfg.num_corres_points!=5 && cfg.num_corres_points!=20 && cfg.num_corres_points!=50)',
 'DRPM comparison requires 5 neighbors and noise mode 0 or 2':'DRPM requires 5, 20 or 50 neighbors and noise mode 0 or 2',
 'std::tie(n,normal_variance,distance,cn)=degeneracy::EstimateNormal<5,double>(q,cfg.lidar_point_noise_std_dev,true);':'''if(count==5) std::tie(n,normal_variance,distance,cn)=degeneracy::EstimateNormal<5,double>(q,cfg.lidar_point_noise_std_dev,true);
   else if(count==20) std::tie(n,normal_variance,distance,cn)=degeneracy::EstimateNormal<20,double>(q,cfg.lidar_point_noise_std_dev,true);
   else std::tie(n,normal_variance,distance,cn)=degeneracy::EstimateNormal<50,double>(q,cfg.lidar_point_noise_std_dev,true);'''}
for a,b in replacements.items():
 if a not in s:raise ValueError(a)
 s=s.replace(a,b)
source=out/'drpm_registration.cpp';source.write_text(s)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
f={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
obj=out/'drpm_registration.o'
subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],*f['CXX_FLAGS'],'-I'+str(HERE/'performance'),'-I'+str(HERE),'-c',str(source),'-o',str(obj)],check=True)
objects={'/graph/manager.cpp.o':WS/'build/mimosa-batched-vision/graph_manager.o','/imu/manager.cpp.o':WS/'build/mimosa-sensor-bias/imu_manager.o','/lidar/geometric.cpp.o':WS/'build/mimosa-compact-prefetch/geometric.o','/lidar/incremental_voxel_map.cpp.o':WS/'build/mimosa-compact-prefetch/incremental_voxel_map.o','/lidar/drpm_registration.cpp.o':obj}
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
cmd=[next((str(p) for suffix,p in objects.items() if s.endswith(suffix)),s) for s in cmd];subprocess.run(cmd,cwd=d.parents[1],check=True)
paths=[Path(__file__).resolve(),source,*objects.values(),out/'libmimosa_lib.so']
(out/'manifest.json').write_text(json.dumps({'scope':__doc__,'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}},indent=2)+'\n')
print(out,flush=True)
