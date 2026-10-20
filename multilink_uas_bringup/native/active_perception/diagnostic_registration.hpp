#pragma once
#include "mimosa/lidar/drpm_registration.hpp"
#include <ros/ros.h>
namespace mimosa { namespace lidar {
inline bool activeDiagnosticsEnabled(){
  static const bool enabled=[](){bool b=false;ros::param::get("/active_lidar_diagnostics",b);return b;}();
  return enabled;
}
// No pose update, graph factor, or mutable map access. Diagnostic only.
void publishActiveDiagnostics(const gtsam::Pose3&,const gtsam::Pose3&,
  const pcl::PointCloud<Point>&,const IncrementalVoxelMapPCL::Ptr&,const RegistrationConfig&,double);
}}
