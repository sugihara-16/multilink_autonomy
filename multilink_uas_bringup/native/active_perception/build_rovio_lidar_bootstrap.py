#!/usr/bin/env python3
"""Isolated optional LiDAR initial-distance prior; no existing-track updates."""
import hashlib,json,shlex,subprocess
from build import HERE,WS,BUILD,flags


def main():
    out=WS/'build/rovio-lidar-bootstrap'
    if (out/'manifest.json').exists() or (out/'rovio_gimbal').exists():raise RuntimeError('Preserve completed build')
    out.mkdir(exist_ok=True)
    original=WS/'build/rovio-joint-geometry/rovio_gimbal.cpp'
    source='#include "rovio_lidar_bootstrap.hpp"\n#include <sensor_msgs/point_cloud2_iterator.h>\n'+original.read_text()
    marker='  std::map<uint64_t,sensor_msgs::ImageConstPtr> images;';assert source.count(marker)==1
    source=source.replace(marker,'''  np.param("lidar_depth_bootstrap",active_depth_bootstrap::enabled(),false);
  Eigen::Matrix3d depth_R=Eigen::Matrix3d::Identity();Eigen::Vector3d depth_t=Eigen::Vector3d::Zero();
  if(active_depth_bootstrap::enabled()){
    std::vector<double> K,T;std::string depth_log;
    if(!np.getParam("depth_intrinsics",K)||K.size()!=9||!np.getParam("depth_T_B_L",T)||T.size()!=7)
      throw std::runtime_error("Explicit LiDAR/camera calibration is required");
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)active_depth_bootstrap::intrinsics()(i,j)=K[3*i+j];
    Eigen::Quaterniond q(T[6],T[3],T[4],T[5]);
    if(std::abs(q.norm()-1.)>1e-6)throw std::runtime_error("Invalid LiDAR rotation");
    depth_R=q.toRotationMatrix();depth_t=Eigen::Vector3d(T[0],T[1],T[2]);
    if(np.getParam("lidar_bootstrap_csv",depth_log)){
      active_depth_bootstrap::audit().open(depth_log);
      if(!active_depth_bootstrap::audit())throw std::runtime_error("Cannot create depth audit");
      active_depth_bootstrap::audit()<<std::setprecision(17)<<"stamp_ns,feature_id,supported,original_depth_m,initial_depth_m,support_sigma_m\\n";
    }
  }
'''+marker)
    marker='      node.imgCallback(it->second,0);';assert source.count(marker)==1
    source=source.replace(marker,'''      if(active_depth_bootstrap::needs_scan(t)&&!active_depth_bootstrap::has_scan(t))break;
'''+marker)
    marker='  auto on_imu=[&]';assert source.count(marker)==1
    source=source.replace(marker,'''  auto on_depth_scan=[&](const sensor_msgs::PointCloud2::ConstPtr& m){
    if(!active_depth_bootstrap::enabled())return;
    std::vector<Eigen::Vector3d> points;points.reserve(m->width*m->height);
    sensor_msgs::PointCloud2ConstIterator<float> x(*m,"x"),y(*m,"y"),z(*m,"z");
    for(;x!=x.end();++x,++y,++z){Eigen::Vector3d p(*x,*y,*z);if(p.allFinite())points.push_back(depth_R*p+depth_t);}
    const long long t=m->header.stamp.toNSec();active_depth_bootstrap::scans()[t]=std::move(points);
    auto& scans=active_depth_bootstrap::scans();
    while(!scans.empty()&&scans.begin()->first+60000000000LL<(long long)last_image)scans.erase(scans.begin());
    drain();diagnose();
  };
'''+marker)
    marker='    ros::spin();';assert source.count(marker)==1
    source=source.replace(marker,'''    ros::Subscriber depth_sub;
    if(active_depth_bootstrap::enabled())depth_sub=nh.subscribe<sensor_msgs::PointCloud2>("/rmf_unipilot/lidar/points",100,on_depth_scan,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
'''+marker)
    header_source=BUILD/'include/rovio/ImgUpdate.hpp';header=header_source.read_text()
    marker='          f.mpDistance_->p_ = medianDepthParameters[camID];';assert header.count(marker)==1
    header='#include "rovio_lidar_bootstrap.hpp"\n'+header.replace(marker,marker+'\n          active_depth_bootstrap::initialize(filterState,f,camID);')
    include=out/'include/rovio/ImgUpdate.hpp';include.parent.mkdir(parents=True,exist_ok=True);include.write_text(header)
    dest=out/'rovio_gimbal.cpp';dest.write_text(source)
    directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';f=flags(directory/'flags.make')
    defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
    obj=out/'rovio_gimbal.o';command=['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{out}/include',f'-I{BUILD}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',str(dest),'-o',str(obj)]
    subprocess.run(command,check=True)
    link=shlex.split((directory/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
    link[link.index('-o')+1]=str(out/'rovio_gimbal');subprocess.run(link,check=True)
    paths=[original,header_source,dest,include,HERE/'build_rovio_lidar_bootstrap.py',HERE/'rovio_lidar_bootstrap.hpp',HERE/'lidar_depth_bootstrap.hpp',HERE/'rovio_joint_information.hpp',HERE/'rovio_joint_geometry.hpp',HERE/'gimbal_rovio.hpp',HERE/'imu_bias_model.hpp',out/'rovio_gimbal']
    (out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,compiler=command,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
    print('Built',out,flush=True)


if __name__=='__main__':main()
