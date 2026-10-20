#pragma once
#include "lidar_depth_bootstrap.hpp"
#include <map>
#include <memory>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace active_depth_bootstrap {
inline bool& enabled(){static bool value=false;return value;}
inline Eigen::Matrix3d& intrinsics(){static Eigen::Matrix3d value=Eigen::Matrix3d::Identity();return value;}
inline std::map<long long,std::vector<Eigen::Vector3d>>& scans(){static std::map<long long,std::vector<Eigen::Vector3d>> value;return value;}
inline std::ofstream& audit(){static std::ofstream value;return value;}
inline bool needs_scan(long long t){return enabled()&&std::abs(t-std::llround(double(t)/100000000.)*100000000LL)<=2;}
inline bool has_scan(long long t){auto i=scans().lower_bound(t-2);return i!=scans().end()&&std::abs(i->first-t)<=2;}
template<class FS,class F> void initialize(const FS& fs,F& feature,int camera){
  if(!enabled())return;
  if(camera!=0)throw std::runtime_error("LiDAR bootstrap supports this single known camera only");
  const auto t=std::llround(fs.t_*1e9);
  auto scan=scans().lower_bound(t-2);
  if(scan==scans().end()||std::abs(scan->first-t)>2)return;
  // The camera calibration is the acquisition-time encoder value already
  // applied by before_image. No estimated world/body pose enters this lookup.
  static long long cached=-1;
  static std::unique_ptr<lidar_depth_bootstrap::View> view;
  if(cached!=t){
    std::vector<Eigen::Vector3d> camera_points;camera_points.reserve(scan->second.size());
    for(const auto& p:scan->second)camera_points.push_back(fs.state_.qCM(0).rotate(Eigen::Vector3d(p-fs.state_.MrMC(0))));
    view.reset(new lidar_depth_bootstrap::View(camera_points,intrinsics()));cached=t;
  }
  const double original=feature.mpDistance_->getDistance();
  const auto support=view->at(feature.mpCoordinates_->get_nor().getVec());
  if(support.valid())feature.mpDistance_->setParameter(support.distance);
  // Keep upstream relative initialization covariance. This is a data-supported
  // initial guess, not a Gaussian depth update or repeated posterior reset.
  if(audit().is_open())audit()<<t<<','<<feature.idx_<<','<<support.valid()<<','<<original<<','<<feature.mpDistance_->getDistance()<<','<<support.sigma<<'\n';
}
}
