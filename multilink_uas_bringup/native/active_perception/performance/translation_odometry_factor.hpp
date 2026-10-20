#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// Constrain displacement in the preceding body frame. No visual rotation
// measurement is added; the body rotations still affect displacement geometry.
class TranslationOdometryFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Pose3> {
  gtsam::Point3 measured_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TranslationOdometryFactor(gtsam::Key k0,gtsam::Key k1,const gtsam::Point3& measured,
      const gtsam::SharedNoiseModel& noise):NoiseModelFactor2(noise,k0,k1),measured_(measured){}
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new TranslationOdometryFactor(*this));
  }
  gtsam::Vector evaluateError(const gtsam::Pose3& p0,const gtsam::Pose3& p1,
      gtsam::OptionalMatrixType H0=nullptr,gtsam::OptionalMatrixType H1=nullptr) const override {
    gtsam::Matrix66 B0,B1;gtsam::Matrix36 J;
    const auto relative=p0.between(p1,B0,B1);
    const auto translation=relative.translation(J);
    if(H0)*H0=J*B0;
    if(H1)*H1=J*B1;
    return translation-measured_;
  }
};
