#include "mimosa/lidar/drpm_registration.hpp"
#include "mimosa/lidar/drpm.hpp"
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <cassert>
#include <iostream>
int main(){
 using namespace mimosa::lidar;using V=Eigen::Vector3d;using M=Eigen::Matrix3d;
 std::vector<V> world;std::vector<M> covs;
 pcl::PointCloud<Point> cloud;
 gtsam::Pose3 extrinsic(gtsam::Rot3::RzRyRx(.04,-.02,.1),V(-.1925,.001,.049));
 // A fully observable corner with known nonzero sensor extrinsic.
 for(int plane=0;plane<3;++plane)for(int a=-4;a<=4;++a)for(int b=-4;b<=4;++b){
   V p;if(plane==0)p=V(4,.4*a,.4*b);else if(plane==1)p=V(.4*a,4,.4*b);else p=V(.4*a,.4*b,3);
   world.push_back(p);covs.push_back(drpmRadialCovariance(p-extrinsic.translation(),.03));
   Point q;q.x=p.x();q.y=p.y();q.z=p.z();q.t=0;q.intensity=1;q.idx=cloud.size();q.range=p.norm();cloud.push_back(q);
 }
 auto map=std::make_shared<IncrementalVoxelMapPCL>(.5);map->underlying()->set_neighbor_voxel_mode(19);gtsam_points::PointCloudCPU frame(world);frame.add_covs(covs);map->underlying()->insert(frame);
 RegistrationConfig cfg;cfg.drpm_enabled=true;cfg.num_corres_points=5;cfg.lidar_point_noise_std_dev=.03;cfg.max_corres_distance=1;cfg.plane_validity_distance=.06;cfg.use_huber=false;cfg.drpm_icp_max_iterations=10;
 gtsam::Vector6 offset;offset<<.008,-.007,.006,.04,-.03,.02;gtsam::Pose3 initial=gtsam::Pose3::Expmap(offset);
 std::ofstream audit("/root/uas_native/logs/zab-offline/drpm_official/synthetic_registration.csv");
 for(int mode:{1,2})for(int noise:{0,2}){
  cfg.drpm_registration_mode=mode;cfg.drpm_noise_model=noise;
  auto f=drpmRegistration(0,initial,extrinsic,cloud,map,cfg,1,audit);assert(f);
  double before=f->evaluateError(initial).norm(),after=f->evaluateError(gtsam::Pose3()).norm();
  std::cout<<"mode="<<mode<<" noise="<<noise<<" residual_at_initial="<<before<<" residual_at_truth="<<after<<std::endl;
  assert(before>1 && after<before*.01);
 }
 std::cout<<"All four ICP modes recover synthetic corner alignment with nonzero extrinsic\n";
}
