#!/usr/bin/env python3
"""Isolated read-only post-sensor state trace; optional no-vision-factor control."""
import hashlib,json,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/mimosa-fusion-trace';out.mkdir(exist_ok=True)
s=(WS/'build/mimosa-batched-vision/ordered_mimosa_node.cpp').read_text()
a='  try {\n    while(ros::ok())'
assert s.count(a)==1
s=s.replace(a,'''  std::ofstream state_audit(directory+"sensor_states.csv");
  state_audit<<std::setprecision(17)<<"stamp_ns,source,state_t,x,y,z,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\\n";
  try {
    while(ros::ok())''')
a="<<std::setprecision(10)<<dt<<','<<nc<<','<<np<<','<<imu_count<<','<<processed_imu<<'\\n';"
assert s.count(a)==1
s=s.replace(a,a+'''
        mimosa::State state;graph->getCurrentState(state);
        const auto pos=state.navState().position(),vel=state.navState().velocity();
        const auto ba=state.imuBias().accelerometer(),bg=state.imuBias().gyroscope();
        state_audit<<stamp<<','<<source<<','<<state.ts();
        for(const auto& v:{pos,vel,ba,bg})for(int i=0;i<3;++i)state_audit<<','<<v[i];
        state_audit<<std::endl;''')
node=out/'ordered_mimosa_node.cpp';node.write_text(s)
s=(HERE/'performance/velocity_odometry_manager.hpp').read_text()
s=s.replace('  double period_=1.,scale_=1.;','  double period_=1.,scale_=1.;\n  bool diagnostic_no_vision_=false;')
a='    bool aligned=false,vio_aligned=false,estimated=true;';assert s.count(a)==1
s=s.replace(a,a+'\n    pnh.param("diagnostic_no_vision",diagnostic_no_vision_,false);')
s=s.replace('if(update){','if(update && !diagnostic_no_vision_){')
(out/'velocity_odometry_manager.hpp').write_text(s)
def flags(d):return {l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
nd=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir';f=flags(nd);obj=out/'node.o'
subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],*f['CXX_FLAGS'],'-I'+str(HERE/'performance'),'-DMIMOSA_BODY_VELOCITY','-O1','-c',str(node),'-o',str(obj)],check=True)
lib=WS/'build/mimosa-batched-vision/libmimosa_lib.so'
cmd=shlex.split((nd/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'mimosa_trace_node')
cmd=[str(obj) if s.endswith('/src/mimosa_node.cpp.o') else str(lib) if s.endswith('/libmimosa_lib.so') else s for s in cmd]
subprocess.run(cmd,cwd=nd.parents[1],check=True)
paths=[Path(__file__).resolve(),node,out/'velocity_odometry_manager.hpp',lib,out/'mimosa_trace_node']
(out/'manifest.json').write_text(json.dumps({'scope':__doc__,'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}},indent=2)+'\n')
print(out,flush=True)
