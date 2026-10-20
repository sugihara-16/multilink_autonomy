#pragma once
// Read-only covariance export. No filter prediction/update or state mutation.
#include <vector>
#include <cmath>
#include <stdexcept>
#include <Eigen/Core>

namespace active_joint_information {
template<class FS> std::vector<double> encode(const FS& fs) {
  using State=typename FS::mtState;
  const auto& state=fs.state_;
  std::vector<int> slots, indices;
  for(int i=0;i<3;++i)indices.push_back(State::template getId<State::_vel>()+i);
  for(int i=0;i<State::nMax_;++i){
    if(!fs.fsm_.isValid_[i] || int(fs.fsm_.features_[i].mpStatistics_->status_[0])!=4)continue;
    slots.push_back(i);
    const int k=State::template getId<State::_fea>(i);
    indices.push_back(k);indices.push_back(k+1);
  }
  // Version, exact integer ns (within double's exact range), tracked count;
  // then one ID and IMU-frame point per track; then row-major joint covariance.
  const auto stamp=std::llround(fs.t_*1e9);
  if(std::abs(fs.t_)>=9007199.)throw std::runtime_error("Joint information stamp exceeds exact representation");
  std::vector<double> out{1.,double(stamp),double(slots.size())};
  for(const int i:slots){
    const int cam=state.CfP(i).camID_;
    const Eigen::Vector3d p=state.MrMC(cam)+state.qCM(cam).inverseRotate(Eigen::Vector3d(state.CfP(i).get_nor().getVec()*state.dep(i).getDistance()));
    out.push_back(double(fs.fsm_.features_[i].idx_));
    for(int j=0;j<3;++j)out.push_back(p[j]);
  }
  // ROVIO stores negative IMU-frame velocity; bearing coordinates unchanged.
  for(size_t i=0;i<indices.size();++i)for(size_t j=0;j<indices.size();++j)
    out.push_back((i<3?-1.:1.)*(j<3?-1.:1.)*fs.cov_(indices[i],indices[j]));
  for(double x:out)if(!std::isfinite(x))throw std::runtime_error("Non-finite joint information");
  return out;
}
}
