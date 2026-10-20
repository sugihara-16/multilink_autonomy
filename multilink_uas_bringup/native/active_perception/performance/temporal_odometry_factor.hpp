#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// Errors are right-local [rotation, translation] tangent vectors at the end
// of each measured increment. Transport the preceding error to this end
// frame before applying the body-frame AR(1) model. No truth is used online.
class TemporalOdometryFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3,gtsam::Pose3,gtsam::Pose3> {
  gtsam::Pose3 previous_, current_;
  gtsam::Matrix66 transition_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TemporalOdometryFactor(gtsam::Key k0,gtsam::Key k1,gtsam::Key k2,
      const gtsam::Pose3& previous,const gtsam::Pose3& current,
      const gtsam::Vector6& rho,const gtsam::SharedNoiseModel& innovation)
    :NoiseModelFactor3(innovation,k0,k1,k2),previous_(previous),current_(current),
     transition_(rho.asDiagonal()*current.inverse().AdjointMap()){}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new TemporalOdometryFactor(*this));
  }
  gtsam::Vector evaluateError(const gtsam::Pose3& p0,const gtsam::Pose3& p1,
      const gtsam::Pose3& p2,gtsam::OptionalMatrixType H0=nullptr,
      gtsam::OptionalMatrixType H1=nullptr,
      gtsam::OptionalMatrixType H2=nullptr) const override {
    gtsam::Matrix66 B0,B1,C0,C1,Jb,Jc;
    const auto b=p0.between(p1,B0,B1), c=p1.between(p2,C0,C1);
    const auto eb=gtsam::Pose3::Logmap(previous_.inverse()*b,Jb);
    const auto ec=gtsam::Pose3::Logmap(current_.inverse()*c,Jc);
    if(H0)*H0=-transition_*Jb*B0;
    if(H1)*H1=Jc*C0-transition_*Jb*B1;
    if(H2)*H2=Jc*C1;
    return ec-transition_*eb;
  }
};
