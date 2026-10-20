#!/usr/bin/env python3
"""Build a separate read-only geometry covariance exporter; keep prior builds."""
import hashlib,json,shlex,subprocess
from build import HERE,WS,BUILD,flags


def main():
    out=WS/'build/rovio-joint-geometry'
    if (out/'manifest.json').exists() or (out/'rovio_gimbal').exists():raise RuntimeError('Completed build already exists')
    out.mkdir(exist_ok=True)
    original=(HERE/'rovio_gimbal.cpp').read_text()
    source='#include <std_msgs/Float64MultiArray.h>\n#include "rovio_joint_information.hpp"\n#include "rovio_joint_geometry.hpp"\n'+original
    marker='  double last_diagnostic=-1.;';assert source.count(marker)==1
    source=source.replace(marker,'''  auto joint_pub=np.advertise<std_msgs::Float64MultiArray>("joint_information",1);
  auto geometry_pub=np.advertise<std_msgs::Float64MultiArray>("joint_geometry",1);
  double last_joint=-1.,last_geometry=-1.;
'''+marker)
    marker='  auto diagnose=[&](){';assert source.count(marker)==1
    source=source.replace(marker,marker+'''
    if(node.gotFirstMessages_ && filter->safe_.t_>last_joint && joint_pub.getNumSubscribers()){
      std_msgs::Float64MultiArray joint; joint.data=active_joint_information::encode(filter->safe_);
      joint_pub.publish(joint); last_joint=filter->safe_.t_;
    }
    if(node.gotFirstMessages_ && filter->safe_.t_>last_geometry && geometry_pub.getNumSubscribers()){
      std_msgs::Float64MultiArray geometry; geometry.data=active_joint_geometry::encode(filter->safe_);
      geometry_pub.publish(geometry); last_geometry=filter->safe_.t_;
    }
''')
    dest=out/'rovio_gimbal.cpp';dest.write_text(source)
    directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';f=flags(directory/'flags.make')
    defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
    obj=out/'rovio_gimbal.o';command=['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{BUILD}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',str(dest),'-o',str(obj)]
    subprocess.run(command,check=True)
    link=shlex.split((directory/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
    link[link.index('-o')+1]=str(out/'rovio_gimbal');subprocess.run(link,check=True)
    paths=[dest,HERE/'rovio_gimbal.cpp',HERE/'rovio_joint_information.hpp',HERE/'rovio_joint_geometry.hpp',HERE/'gimbal_rovio.hpp',HERE/'imu_bias_model.hpp',out/'rovio_gimbal']
    (out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,compiler=command,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
    print('Built',out,flush=True)


if __name__=='__main__':main()
