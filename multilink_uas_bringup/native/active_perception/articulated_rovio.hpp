#pragma once
#include <map>
#include <cmath>
#include <stdexcept>
#include "lightweight_filtering/State.hpp"

namespace active_gimbal {
struct Extrinsic { QPD qCM; V3D position; };
inline std::map<long long,Extrinsic>& samples(){static std::map<long long,Extrinsic> s;return s;}

// Deterministic SE(3) coordinate change, including the depth/bearing coupling
// and every cross covariance. The reference IMU state is NOT moved.
template<class FS> void reframe(FS& fs,const Extrinsic& ex){
  using S=typename FS::mtState;
  auto& s=fs.state_;
  if(s.aux().doVECalibration_)throw std::runtime_error("Articulated camera needs known extrinsics");
  const M3D R=MPD(ex.qCM*s.qCM(0).inverted()).matrix();
  const V3D delta=s.MrMC(0)-ex.position;
  const V3D t=ex.qCM.rotate(delta);
  if((R-M3D::Identity()).norm()<1e-13 && t.norm()<1e-13)return;
  MXD A=MXD::Identity(fs.cov_.rows(),fs.cov_.cols());
  for(int i=0;i<S::nMax_;++i){
    if(!fs.fsm_.isValid_[i])continue;
    auto& f=s.CfP(i);auto& depth=s.dep(i);
    const auto old=f.get_nor();
    const V3D p=R*(old.getVec()*depth.getDistance())+t;
    const double d=p.norm();
    if(!std::isfinite(d) || d<1e-6)throw std::runtime_error("Feature at moving camera centre");
    const LWF::NormalVectorElement next(p/d);
    Eigen::Matrix3d D;
    D.template leftCols<2>()=R*old.getM()*depth.getDistance();
    D.col(2)=R*old.getVec()*depth.getDistanceDerivative();
    depth.setParameter(d);
    Eigen::Matrix3d B;
    B.template topRows<2>()=next.getM().transpose()/d;
    B.row(2)=depth.getParameterDerivative()*next.getVec().transpose();
    const Eigen::Matrix3d J=B*D;
    const Eigen::Matrix2d warp=f.get_warp_nor();
    f.set_nor(next);
    if(f.trackWarping_)f.set_warp_nor(J.template topLeftCorner<2,2>()*warp);
    const int index=S::template getId<S::_fea>(i);
    A.template block<3,3>(index,index)=J;
  }
  fs.cov_=(A*fs.cov_*A.transpose()).eval();
  s.qCM(0)=ex.qCM;s.MrMC(0)=ex.position;
}
template<class FS> void before_image(FS& fs){
  const auto t=llround(fs.t_*1e9);auto it=samples().lower_bound(t-2);
  if(it==samples().end() || std::abs(it->first-t)>2)throw std::runtime_error("Missing acquisition-time joint measurement");
  reframe(fs,it->second);
}
}
