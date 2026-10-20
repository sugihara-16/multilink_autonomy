#!/usr/bin/env python3
"""Isolated synchronous vision/LiDAR transaction; existing trials are untouched.

After both managers initialize, coincident VIO runs first, LiDAR runs second,
the map uses the resulting joint pose, and graph odometry is emitted only after
both updates. Refresh cached optimized values after same-key updates as well.
Sensor samples, timestamps, factor weights and initialization order are retained.
"""
import hashlib,json,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-batched-vision';out.mkdir(exist_ok=True)
graph_original=WS/'comparison_ws/src/mimosa/src/graph/manager.cpp'
s=graph_original.read_text()
anchor='#include "mimosa/graph/manager.hpp"';assert s.count(anchor)==1
s=s.replace(anchor,anchor+'''
#include <functional>
namespace active_fusion_batch {
thread_local bool deferred=false;
thread_local std::function<void()> flush;
}
extern "C" void mimosa_begin_sensor_batch(){
  if(active_fusion_batch::deferred)throw std::runtime_error("Nested sensor batch");
  active_fusion_batch::deferred=true;
}
extern "C" void mimosa_end_sensor_batch(){
  if(!active_fusion_batch::deferred || !active_fusion_batch::flush)
    throw std::runtime_error("No initialized sensor batch");
  active_fusion_batch::deferred=false;active_fusion_batch::flush();
}
''')
anchor='  // Prepare logger';assert s.count(anchor)==1
s=s.replace(anchor,'  active_fusion_batch::flush=[this](){publishResults();};\n'+anchor)
anchor='''  if (result != DeclarationResult::SUCCESS_SAME_KEY) {
    // ? In the case of same key, the optimized values returned are empty for some reason
    optimized_values_ = smoother_->calculateEstimate();''';assert s.count(anchor)==1
s=s.replace(anchor,'''  // Every sensor update changes the estimate, including an existing timestamp.
  optimized_values_ = smoother_->calculateEstimate();
  if (result != DeclarationResult::SUCCESS_SAME_KEY) {''')
anchor='void Manager::publishResults()\n{';assert s.count(anchor)==1
s=s.replace(anchor,anchor+'\n  if(active_fusion_batch::deferred)return;')
graph=out/'graph_manager.cpp';graph.write_text(s)
node_original=WS/'build/mimosa-velocity/ordered_mimosa_node.cpp'
s=node_original.read_text()
s='extern "C" void mimosa_begin_sensor_batch();\nextern "C" void mimosa_end_sensor_batch();\n'+s
anchor='  void disconnect(){this->sub_.shutdown();}';assert s.count(anchor)==1
s=s.replace(anchor,anchor+'\n  bool inputInitialized() const{return this->initialized_;}')
anchor='''        if(tc<=tp){cloud=clouds.front();clouds.pop_front();}
        else{pose=poses.front();poses.pop_front();}''';assert s.count(anchor)==1
s=s.replace(anchor,'''        if(tc==tp && lidar.inputInitialized() && vio.inputInitialized()){
          pose=poses.front();poses.pop_front();cloud=clouds.front();clouds.pop_front();
        }else if(tc<=tp){cloud=clouds.front();clouds.pop_front();}
        else{pose=poses.front();poses.pop_front();}''')
anchor='''      if(cloud)lidar.callback(cloud);else vio.callback(pose);
      const double dt=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      audit<<(cloud?cloud->header.stamp.toNSec():pose->header.stamp.toNSec())<<','<<(cloud?"lidar":"vio")<<','
           <<std::setprecision(10)<<dt<<','<<nc<<','<<np<<','<<batch.size()<<','<<processed_imu<<'\\n';''';assert s.count(anchor)==1
s=s.replace(anchor,'''      auto record=[&](uint64_t stamp,const char* source,size_t imu_count){
        const double dt=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        audit<<stamp<<','<<source<<','<<std::setprecision(10)<<dt<<','<<nc<<','<<np<<','<<imu_count<<','<<processed_imu<<'\\n';
      };
      if(cloud && pose){
        mimosa_begin_sensor_batch();
        vio.callback(pose);record(pose->header.stamp.toNSec(),"vio",batch.size());
        lidar.callback(cloud);record(cloud->header.stamp.toNSec(),"lidar",0);
        mimosa_end_sensor_batch();
      }else{
        if(cloud)lidar.callback(cloud);else vio.callback(pose);
        record(cloud?cloud->header.stamp.toNSec():pose->header.stamp.toNSec(),cloud?"lidar":"vio",batch.size());
      }''')
node=out/'ordered_mimosa_node.cpp';node.write_text(s)
def flags(d):
    return {l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
libd=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir';f=flags(libd)
obj=out/'graph_manager.o'
subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],*f['CXX_FLAGS'],'-c',str(graph),'-o',str(obj)],check=True)
objects={'/graph/manager.cpp.o':obj,'/imu/manager.cpp.o':WS/'build/mimosa-sensor-bias/imu_manager.o',
    '/lidar/geometric.cpp.o':WS/'build/mimosa-compact-prefetch/geometric.o',
    '/lidar/incremental_voxel_map.cpp.o':WS/'build/mimosa-compact-prefetch/incremental_voxel_map.o',
    '/lidar/drpm_registration.cpp.o':WS/'build/active_perception/drpm_registration.o'}
cmd=shlex.split((libd/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
cmd=[next((str(p) for suffix,p in objects.items() if s.endswith(suffix)),s) for s in cmd]
subprocess.run(cmd,cwd=libd.parents[1],check=True)
nd=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir';f=flags(nd);obj=out/'node.o'
subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],*f['CXX_FLAGS'],'-I'+str(HERE/'performance'),'-DMIMOSA_BODY_VELOCITY','-O1','-c',str(node),'-o',str(obj)],check=True)
cmd=shlex.split((nd/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'mimosa_batched_node')
cmd=[str(obj) if s.endswith('/src/mimosa_node.cpp.o') else str(out/'libmimosa_lib.so') if s.endswith('/libmimosa_lib.so') else s for s in cmd]
subprocess.run(cmd,cwd=nd.parents[1],check=True)
paths=[Path(__file__).resolve(),graph_original,graph,node_original,node,*objects.values(),out/'libmimosa_lib.so',out/'mimosa_batched_node']
(out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
print('BUILT',out,flush=True)
