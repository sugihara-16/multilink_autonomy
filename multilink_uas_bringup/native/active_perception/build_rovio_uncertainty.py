#!/usr/bin/env python3
"""Build a logging-only ROVIO replay binary; preserve the normal estimator."""
import hashlib,json,shlex,subprocess
from pathlib import Path
from build import HERE,WS,BUILD,flags

out=WS/'build/rovio-uncertainty';out.mkdir(exist_ok=False)
original=(HERE/'rovio_gimbal.cpp').read_text()
columns=['vx','vy','vz']+['pose_var_'+s for s in ['x','y','z','rx','ry','rz']]+['velocity_cov_'+str(i)+str(j) for i in range(3) for j in range(3)]
old='features,median_depth\\n";';assert original.count(old)==1
source=original.replace(old,'features,median_depth,'+','.join(columns)+'\\n";')
old="std::sort(depth.begin(),depth.end());diagnostic<<','<<depth.size()<<','<<(depth.empty()?0.:depth[depth.size()/2])<<'\\n';"
assert source.count(old)==1
new="""std::sort(depth.begin(),depth.end());diagnostic<<','<<depth.size()<<','<<(depth.empty()?0.:depth[depth.size()/2]);
    diagnostic<<','<<m.twist.twist.linear.x<<','<<m.twist.twist.linear.y<<','<<m.twist.twist.linear.z;
    for(int i=0;i<6;++i)diagnostic<<','<<m.pose.covariance[7*i];
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)diagnostic<<','<<m.twist.covariance[6*i+j];
    diagnostic<<'\\n';"""
source=source.replace(old,new);dest=out/'rovio_gimbal.cpp';dest.write_text(source)
directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';f=flags(directory/'flags.make')
defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
obj=out/'rovio_gimbal.o'
command=['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{BUILD}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',str(dest),'-o',str(obj)]
subprocess.run(command,check=True)
link=shlex.split((directory/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link];link[link.index('-o')+1]=str(out/'rovio_gimbal');subprocess.run(link,check=True)
(out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,original_sha256=hashlib.sha256(original.encode()).hexdigest(),columns=columns,compiler=command,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dest,HERE/'gimbal_rovio.hpp',HERE/'imu_bias_model.hpp',out/'rovio_gimbal']}),indent=2)+'\n')
print('BUILT',out,flush=True)
