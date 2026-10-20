#pragma once
// An optional starting distance for NEW visual features, never a pose update.
// Only a contemporaneous, locally planar LiDAR neighborhood may supply it.
// Existing visual tracks and their posterior covariance are not changed.
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lidar_depth_bootstrap {
struct Support {
  double distance=std::numeric_limits<double>::quiet_NaN();
  double sigma=std::numeric_limits<double>::quiet_NaN();
  bool valid() const {return std::isfinite(distance)&&std::isfinite(sigma);}
};
struct Sample {Eigen::Vector3d point;Eigen::Vector2d pixel;};
class View {
  std::vector<Sample> points_;
  Eigen::Matrix3d K_;
  double range_;
public:
  View(const std::vector<Eigen::Vector3d>& camera_points,const Eigen::Matrix3d& K,double range=30.)
      :K_(K),range_(range) {
    for(const auto& p:camera_points)if(p.allFinite()&&p.z()>.2&&p.norm()<range_){
      const Eigen::Vector3d uv=K_*p;
      points_.push_back({p,uv.head<2>()/uv.z()});
    }
  }
  Support at(const Eigen::Vector3d& ray) const {
    Support out;
    if(points_.size()<8||!ray.allFinite()||ray.z()<=0.||ray.norm()<1e-12)return out;
    Eigen::Vector3d projected=K_*ray;
    const Eigen::Vector2d pixel=projected.head<2>()/projected.z();
    std::vector<std::pair<double,size_t>> closest;closest.reserve(points_.size());
    for(size_t i=0;i<points_.size();++i)closest.emplace_back((pixel-points_[i].pixel).squaredNorm(),i);
    std::partial_sort(closest.begin(),closest.begin()+8,closest.end());
    if(closest[0].first>64.||closest[4].first>324.)return out;
    std::vector<Eigen::Vector3d> near;
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();
    for(size_t i=0;i<8;++i)if(closest[i].first<=324.){
      near.push_back(points_[closest[i].second].point);mean+=near.back();
    }
    mean/=double(near.size());Eigen::Matrix3d scatter=Eigen::Matrix3d::Zero();double radius=0.;
    for(const auto& p:near){const Eigen::Vector3d d=p-mean;scatter.noalias()+=d*d.transpose();radius=std::max(radius,d.norm());}
    scatter/=double(near.size());Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter);
    if(eig.info()!=Eigen::Success||eig.eigenvalues()[1]<1e-5)return out;
    const double residual=std::sqrt(std::max(0.,eig.eigenvalues()[0]));
    if(residual>.06||radius>std::max(.6,.15*mean.norm()))return out;
    const Eigen::Vector3d unit=ray.normalized(),normal=eig.eigenvectors().col(0);
    const double incidence=normal.dot(unit);
    if(std::abs(incidence)<.2)return out;
    const double distance=normal.dot(mean)/incidence;
    if(!(distance>.3&&distance<range_)||(distance*unit-mean).norm()>std::max(.3,.1*distance))return out;
    out.distance=distance;out.sigma=std::max(.08,(.03+residual)/std::abs(incidence));return out;
  }
};
}
