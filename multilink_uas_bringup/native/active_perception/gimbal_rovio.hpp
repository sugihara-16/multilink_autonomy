#pragma once
#include <map>
#include <cmath>
#include <stdexcept>
#include "lightweight_filtering/State.hpp"

namespace active_gimbal {
struct Extrinsic { QPD qCM; V3D position; };
inline std::map<long long,Extrinsic>& samples(){static std::map<long long,Extrinsic> s;return s;}

// Exact change of feature coordinates for a known rotation about the optical
// centre, applied AFTER body/IMU prediction and BEFORE the image update.
// ROVIO otherwise mistakes this rotation for body motion. No IMU modification.
template<class FS> void reframe(FS& fs, const Extrinsic& ex){
  using S=typename FS::mtState;
  auto& s=fs.state_;
  if(s.aux().doVECalibration_) throw std::runtime_error("Gimbal requires known extrinsics");
  if((ex.position-s.MrMC(0)).norm()>1e-5)
    throw std::runtime_error("Optical-centre translation is unsupported by rotation-only gimbal");
  QPD q=ex.qCM*s.qCM(0).inverted();
  const M3D R=MPD(q).matrix();
  if((R-M3D::Identity()).norm()<1e-13) return;
  MXD A=MXD::Identity(fs.cov_.rows(),fs.cov_.cols());
  for(int i=0;i<S::nMax_;++i){
    if(!fs.fsm_.isValid_[i]) continue;
    auto& f=s.CfP(i);
    const auto old=f.get_nor();
    const auto next=old.rotated(q);
    const Eigen::Matrix2d J=next.getM().transpose()*R*old.getM();
    const Eigen::Matrix2d warp=f.get_warp_nor();
    f.set_nor(next);
    if(f.trackWarping_) f.set_warp_nor(J*warp);
    const int index=S::template getId<S::_fea>(i);
    A.block<2,2>(index,index)=J;
  }
  fs.cov_=(A*fs.cov_*A.transpose()).eval();
  s.qCM(0)=ex.qCM;
}

template<class FS> void before_image(FS& fs){
  const auto t=llround(fs.t_*1e9);
  auto it=samples().lower_bound(t-2);
  if(it==samples().end() || std::abs(it->first-t)>2)
    throw std::runtime_error("No acquisition-time gimbal measurement");
  reframe(fs,it->second);
}
}
