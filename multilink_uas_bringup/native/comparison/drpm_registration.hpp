#pragma once
#include "mimosa/lidar/geometric_config.hpp"
#include "mimosa/lidar/incremental_voxel_map.hpp"
#include "mimosa/lidar/drpm_pose_factor.hpp"
#include <fstream>
namespace mimosa { namespace lidar {
std::shared_ptr<DrpmPoseFactor> drpmRegistration(
  gtsam::Key key,const gtsam::Pose3& initial_body,const gtsam::Pose3& body_to_lidar,
  const pcl::PointCloud<Point>& source,const IncrementalVoxelMapPCL::Ptr& target,
  const RegistrationConfig& config,double stamp,std::ofstream& audit);
}}
