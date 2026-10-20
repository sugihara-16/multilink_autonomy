#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace mimosa { namespace lidar {
// Frozen ICP measurement with Eq.28 information in its local LiDAR frame.
// Whiten explicitly to permit zero-information directions without adding an
// arbitrary diagonal floor. The prior of Eq.26 stays inside ICP only.
class DrpmPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
  gtsam::Pose3 target_lidar_,body_to_lidar_;
  gtsam::Matrix6 sqrt_information_;
 public:
  DrpmPoseFactor(gtsam::Key key,const gtsam::Pose3& target_lidar,
      const gtsam::Pose3& body_to_lidar,const gtsam::Matrix6& sqrt_information)
   : gtsam::NoiseModelFactor1<gtsam::Pose3>(gtsam::noiseModel::Unit::Create(6),key),
     target_lidar_(target_lidar),body_to_lidar_(body_to_lidar),sqrt_information_(sqrt_information) {}
  gtsam::Vector evaluateError(const gtsam::Pose3& body,
      gtsam::Matrix* H=nullptr) const override {
    gtsam::Matrix6 A,B,C;
    const auto lidar=body.compose(body_to_lidar_,A);
    const auto relative=target_lidar_.between(lidar,{},B);
    const gtsam::Vector6 error=gtsam::Pose3::Logmap(relative,C);
    if(H)*H=sqrt_information_*C*B*A;
    return sqrt_information_*error;
  }
};
}}
