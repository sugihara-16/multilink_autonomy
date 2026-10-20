#pragma once
// Read-only joint covariance of body velocity and body-relative 3D landmarks.
// Geometry Jacobian is the same as upstream LandmarkOutputImuCT for known
// extrinsics, but operates on local values without mutating MultiCamera.
#include <vector>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

namespace active_joint_geometry {
template<class FS> std::vector<double> encode(const FS& fs) {
  using State=typename FS::mtState;
  const auto& s=fs.state_;
  if(s.aux().doVECalibration_)throw std::runtime_error("Joint geometry requires known camera extrinsics");
  std::vector<int> slots;
  for(int i=0;i<State::nMax_;++i)
    if(fs.fsm_.isValid_[i] && int(fs.fsm_.features_[i].mpStatistics_->status_[0])==4)slots.push_back(i);
  const auto stamp=std::llround(fs.t_*1e9);
  if(std::abs(fs.t_)>=9007199.)throw std::runtime_error("Joint geometry timestamp exceeds exact representation");
  const Eigen::Vector3d velocity=-s.MvM();
  std::vector<double> out{2.,double(stamp),double(slots.size()),velocity.x(),velocity.y(),velocity.z()};
  Eigen::MatrixXd J=Eigen::MatrixXd::Zero(3+3*slots.size(),fs.cov_.cols());
  J.template block<3,3>(0,State::template getId<State::_vel>())=-Eigen::Matrix3d::Identity();
  for(size_t k=0;k<slots.size();++k){
    const int i=slots[k],cam=s.CfP(i).camID_;
    const auto& normal=s.CfP(i).get_nor();
    Eigen::Matrix3d R;
    for(int j=0;j<3;++j){const Eigen::Vector3d basis=Eigen::Vector3d::Unit(j);R.col(j)=s.qCM(cam).inverseRotate(basis);}
    const Eigen::Vector3d p=s.MrMC(cam)+R*(normal.getVec()*s.dep(i).getDistance());
    out.push_back(double(fs.fsm_.features_[i].idx_));
    for(int j=0;j<3;++j)out.push_back(p[j]);
    const int index=State::template getId<State::_fea>(i);
    J.template block<3,2>(3+3*k,index)=R*(s.dep(i).getDistance()*normal.getM());
    J.template block<3,1>(3+3*k,index+2)=R*(normal.getVec()*s.dep(i).getDistanceDerivative());
  }
  const Eigen::MatrixXd C=J*fs.cov_*J.transpose();
  for(int i=0;i<C.rows();++i)for(int j=0;j<C.cols();++j)out.push_back(C(i,j));
  for(double x:out)if(!std::isfinite(x))throw std::runtime_error("Nonfinite joint geometry");
  return out;
}
}
