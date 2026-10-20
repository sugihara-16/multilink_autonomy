#pragma once
// Restricted-DoF extension, NOT an unmodified six-DoF official DRPM pipeline.
// The observed variables are world-vertical heading and three translations.
// Eq.24 uses the unchanged official solver; excluded rotations have exactly
// zero probability/information. No regularizer is exported as a measurement.
#include "mimosa/lidar/drpm_kernel.hpp"

namespace mimosa { namespace lidar {
inline DrpmKernelResult drpmFourDofKernel(
    const DrpmM6& H, const DrpmV6& rhs,
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Vector3d>& normals,
    const std::vector<double>& weights,
    const std::vector<Eigen::Matrix3d>& normal_covariances,
    const std::vector<Eigen::Matrix3d>& point_covariances,
    double sigma, double snr, int mode, const Eigen::Vector3d& vertical_local) {
  if(mode!=1 || !vertical_local.allFinite() || vertical_local.norm()<1e-8)
    throw std::runtime_error("Four-DoF DRPM requires Eq.24 and a valid vertical");
  const size_t n=points.size();
  if(normals.size()!=n || weights.size()!=n || normal_covariances.size()!=n ||
      (!point_covariances.empty() && point_covariances.size()!=n))
    throw std::runtime_error("Four-DoF DRPM input size mismatch");
  using M64=Eigen::Matrix<double,6,4>;
  const Eigen::Vector3d z=vertical_local.normalized(), x=z.unitOrthogonal(), y=z.cross(x);
  M64 Q=M64::Zero();Q.topLeftCorner<3,1>()=z;Q.bottomRightCorner<3,3>().setIdentity();
  const Eigen::Matrix4d reduced=Q.transpose()*((H+H.transpose())*.5)*Q;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(reduced);
  if(es.info()!=Eigen::Success || !es.eigenvalues().allFinite() || es.eigenvalues().minCoeff()<=0)
    throw std::runtime_error("Four-DoF DRPM nonpositive reduced Hessian");
  DrpmKernelResult r;
  r.eigenvectors.setZero();r.eigenvectors.block<3,1>(0,0)=x;r.eigenvectors.block<3,1>(0,1)=y;
  r.eigenvectors.rightCols<4>()=Q*es.eigenvectors();
  r.eigenvalues.setZero();r.eigenvalues.tail<4>()=es.eigenvalues();
  r.probabilities.setZero();
  DrpmM6 mean=DrpmM6::Zero();Eigen::Vector4d variance=Eigen::Vector4d::Zero();
  for(size_t i=0;i<n;++i){
    const auto nx=drpmSkew(normals[i]),px=drpmSkew(points[i]);
    DrpmM6 B=DrpmM6::Zero(),N=DrpmM6::Zero();
    B.topLeftCorner<3,3>()=-nx;B.topRightCorner<3,3>()=px*nx;B.bottomRightCorner<3,3>()=nx;
    if(point_covariances.empty())N.topLeftCorner<3,3>()=Eigen::Matrix3d::Identity()*sigma*sigma;
    else N.topLeftCorner<3,3>()=point_covariances[i];
    N.bottomRightCorner<3,3>()=normal_covariances[i];
    const DrpmM6 C=weights[i]*B*N*B.transpose();mean+=C;
    DrpmV6 j;j<<points[i].cross(normals[i]),normals[i];j*=std::sqrt(weights[i]);
    for(int k=0;k<4;++k){
      const auto u=r.eigenvectors.col(k+2);const double a=std::max(0.,u.dot(C*u)),b=u.dot(j);
      variance[k]+=2*a*a+4*a*b*b;
    }
  }
  if(!mean.allFinite() || !variance.allFinite())throw std::runtime_error("Invalid four-DoF DRPM noise");
  for(int k=0;k<4;++k){
    const auto u=r.eigenvectors.col(k+2);
    const double test=r.eigenvalues[k+2]/(1.+snr),mu=u.dot(mean*u);
    r.probabilities[k+2]=variance[k]>1e-30 ?
      boost::math::cdf(boost::math::normal_distribution<double>(mu,std::sqrt(variance[k])),test) : double(test>mu);
  }
  // Padding solely adapts the official fixed-size solver to four variables:
  // p=0 on the two excluded axes, so their dummy denominator contributes zero.
  DrpmV6 denominator=r.eigenvalues;denominator.head<2>().setOnes();
  r.update=degeneracy::SolveWithSnrProbabilities<double>(r.eigenvectors,denominator,rhs,r.probabilities);
  r.sqrt_information=(r.eigenvalues.array()*r.probabilities.array()).matrix().cwiseSqrt().asDiagonal()*r.eigenvectors.transpose();
  r.information=r.sqrt_information.transpose()*r.sqrt_information;
  if(!r.update.allFinite() || !r.information.allFinite())throw std::runtime_error("Invalid four-DoF DRPM result");
  return r;
}
}}
