#pragma once
#include "body_velocity_factor.hpp"
#include <Eigen/Eigenvalues>

inline gtsam::Matrix3 velocityInverseRoot(const gtsam::Matrix3& C){
  if(!C.allFinite()||!C.isApprox(C.transpose(),1e-10))throw std::invalid_argument("Invalid velocity covariance");
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> eig(C);
  if(eig.info()!=Eigen::Success||eig.eigenvalues().minCoeff()<=0.)throw std::invalid_argument("Nonpositive velocity covariance");
  return eig.eigenvectors()*eig.eigenvalues().cwiseSqrt().cwiseInverse().asDiagonal()*eig.eigenvectors().transpose();
}

// A single current innovation, not another independent copy of the old error.
// Rotation is measured current-body <- previous-body. Global yaw/position are
// not observations. Frame transport is an approximation conditioned on VIO.
class TemporalVelocityFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3>{
  gtsam::Vector3 previous_,current_;
  gtsam::Matrix3 A_,B_;
public:
  using Base=gtsam::NoiseModelFactorN<gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3>;
  TemporalVelocityFactor(gtsam::Key p0,gtsam::Key v0,gtsam::Key p1,gtsam::Key v1,
    const gtsam::Vector3& previous,const gtsam::Vector3& current,
    const gtsam::Matrix3& previous_cov,const gtsam::Matrix3& current_cov,
    const gtsam::Rot3& current_from_previous,double rho,const gtsam::SharedNoiseModel& noise)
    :Base(noise,p0,v0,p1,v1),previous_(previous),current_(current),
     A_(rho*current_from_previous.matrix()*velocityInverseRoot(previous_cov)),B_(velocityInverseRoot(current_cov)){
    if(!std::isfinite(rho)||rho<0.||rho>=1.)throw std::invalid_argument("Invalid velocity persistence");
  }
  gtsam::NonlinearFactor::shared_ptr clone()const override{return std::make_shared<TemporalVelocityFactor>(*this);}
  gtsam::Vector evaluateError(const gtsam::Pose3& p0,const gtsam::Vector3& v0,
    const gtsam::Pose3& p1,const gtsam::Vector3& v1,
    gtsam::OptionalMatrixType H0=nullptr,gtsam::OptionalMatrixType H1=nullptr,
    gtsam::OptionalMatrixType H2=nullptr,gtsam::OptionalMatrixType H3=nullptr)const override{
    const gtsam::Vector3 u0=p0.rotation().unrotate(v0),u1=p1.rotation().unrotate(v1);
    if(H0){*H0=gtsam::Matrix36::Zero();H0->leftCols(3)=-A_*gtsam::skewSymmetric(u0);}
    if(H1)*H1=-A_*p0.rotation().matrix().transpose();
    if(H2){*H2=gtsam::Matrix36::Zero();H2->leftCols(3)=B_*gtsam::skewSymmetric(u1);}
    if(H3)*H3=B_*p1.rotation().matrix().transpose();
    return B_*(u1-current_)-A_*(u0-previous_);
  }
};
