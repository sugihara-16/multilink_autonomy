#include <mimosa/lidar/geometric_factor.hpp>
#include <iostream>
#include <stdexcept>
using namespace mimosa; using namespace mimosa::lidar;
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){
  pcl::PointCloud<Point> target,source;
  for(int axis=0;axis<3;axis++)for(int x=0;x<45;x++)for(int y=0;y<45;y++){
    Eigen::Vector3f v;v[axis]=3.;v[(axis+1)%3]=(x-22)*.12;v[(axis+2)%3]=(y-22)*.12;
    Point p{};p.x=v.x();p.y=v.y();p.z=v.z();target.push_back(p);
    if(x%2==0&&y%2==0){p.x+=.015;p.z-=.01;source.push_back(p);}
  }
  auto map=std::make_shared<IncrementalVoxelMapPCL>(.5);map->underlying()->set_neighbor_voxel_mode(19);map->insert(target);
  for(int k=0;k<15;k++){
    const auto pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.001*k,-.002*k,.003*k),gtsam::Point3(.01,.005,-.01));
    gtsam::Values values;values.insert(X(0),pose);values.insert(G(0),gtsam::Unit3(0,0,-1));
    RegistrationConfig config;config.project_on_degneneracy=false;config.drpm_enabled=false;config.reg_4_dof=false;
    ICPFactor full(X(0),map,source,config);config.reg_4_dof=true;ICPFactor four(X(0),map,source,config);
    const auto H=std::dynamic_pointer_cast<gtsam::HessianFactor>(full.linearize(values))->augmentedInformation();
    const auto H4=std::dynamic_pointer_cast<gtsam::HessianFactor>(four.linearize(values))->augmentedInformation();
    const auto up=pose.rotation().unrotate(gtsam::Point3(0,0,1));
    gtsam::Vector6 tilt=gtsam::Vector6::Zero();tilt.head<3>()=up.cross(gtsam::Point3(1,0,0)).normalized();
    require((H4.topLeftCorner(6,6)*tilt).norm()<1e-8*H.norm(),"LiDAR still constrains gravity tilt");
    require((H.topLeftCorner(6,6)*tilt).norm()>.01*H.norm(),"Synthetic geometry does not constrain tilt in full mode");
    Eigen::Matrix<double,6,4> A=Eigen::Matrix<double,6,4>::Zero();A.block<3,1>(0,0)=up;A.block<3,3>(3,1).setIdentity();
    require((A.transpose()*H.topLeftCorner(6,6)*A-A.transpose()*H4.topLeftCorner(6,6)*A).norm()<1e-10*H.norm(),"Position/yaw information changed");
    require((A.transpose()*H.block<6,1>(0,6)-A.transpose()*H4.block<6,1>(0,6)).norm()<1e-10*H.norm(),"Position/yaw correction changed");
    require(full.getStatuses()==four.getStatuses(),"Correspondence selection changed");
  }
  std::cout<<"15 non-level poses: no LiDAR tilt constraint; identical position/yaw information, correction, and correspondences: PASS\n";
}
