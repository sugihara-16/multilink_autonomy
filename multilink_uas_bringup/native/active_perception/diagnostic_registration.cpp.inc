
// Diagnostic at the ordinary MIMOSA linearization pose, without ICP or factors.
namespace mimosa { namespace lidar {
void publishActiveDiagnostics(const gtsam::Pose3& body,const gtsam::Pose3& extrinsic,
 const pcl::PointCloud<Point>& source,const IncrementalVoxelMapPCL::Ptr& target,
 const RegistrationConfig& cfg,double stamp){
 try {
  std::vector<V3> points;points.reserve(source.size());const auto inv=extrinsic.inverse();
  for(const auto& p:source.points)points.push_back(inv*V3(p.getVector3fMap().cast<double>()));
  const auto pose=body*extrinsic;auto l=linearize(pose,points,target,cfg);
  if(l.points.size()<6)return;
  auto r=drpmKernel(l.H,l.rhs,l.points,l.normals,l.weights,l.normal_covariances,l.point_covariances,
    cfg.lidar_point_noise_std_dev,cfg.drpm_snr_factor,1);
  publishActiveDrpm(stamp,pose,r);
 } catch(const std::runtime_error& e){ROS_WARN_THROTTLE(5,"View-only LiDAR diagnosis: %s",e.what());}
}
}}
