#pragma once
// Body-frame velocity observes V without importing the VIO world's yaw or
// differentiating position corrections. Extrinsics are body <- sensor.
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

inline gtsam::Matrix36 bodyVelocityTransform(const gtsam::Pose3& body_sensor) {
  gtsam::Matrix36 A;
  const auto R=body_sensor.rotation().matrix();
  A.leftCols<3>()=R;
  A.rightCols<3>()=gtsam::skewSymmetric(body_sensor.translation())*R;
  return A;
}

class BodyVelocityFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3,gtsam::Vector3> {
  gtsam::Vector3 measured_;
public:
  using Base=gtsam::NoiseModelFactorN<gtsam::Pose3,gtsam::Vector3>;
  BodyVelocityFactor(gtsam::Key pose,gtsam::Key velocity,const gtsam::Vector3& measured,
      const gtsam::SharedNoiseModel& noise):Base(noise,pose,velocity),measured_(measured){}
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<BodyVelocityFactor>(*this);
  }
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,const gtsam::Vector3& velocity,
      gtsam::OptionalMatrixType H1=nullptr,gtsam::OptionalMatrixType H2=nullptr) const override {
    const gtsam::Vector3 predicted=pose.rotation().unrotate(velocity);
    if(H1){*H1=gtsam::Matrix36::Zero();H1->leftCols(3)=gtsam::skewSymmetric(predicted);}
    if(H2)*H2=pose.rotation().matrix().transpose();
    return predicted-measured_;
  }
};
