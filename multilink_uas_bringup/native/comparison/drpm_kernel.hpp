#pragma once
#include "official_drpm/degeneracy.h"
#include "drpm.hpp"

namespace mimosa { namespace lidar {
using DrpmM6=Eigen::Matrix<double,6,6>;
using DrpmV6=Eigen::Matrix<double,6,1>;
struct DrpmKernelResult {
  DrpmM6 eigenvectors, information, sqrt_information;
  DrpmV6 eigenvalues, probabilities, update;
};

// mode=1: unmodified upstream Eq.24 solver. mode=2: old weighted-cost
// objective; p cancels for nonzero directions, exact-zero directions omitted.
inline DrpmKernelResult drpmKernel(
    const DrpmM6& H, const DrpmV6& rhs,
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Vector3d>& normals,
    const std::vector<double>& weights,
    const std::vector<Eigen::Matrix3d>& normal_covariances,
    const std::vector<Eigen::Matrix3d>& point_covariances,
    double sigma, double snr, int mode) {
  Eigen::SelfAdjointEigenSolver<DrpmM6> es(H);
  if(es.info()!=Eigen::Success || es.eigenvalues().minCoeff()<=0)
    throw std::runtime_error("DRPM nonpositive Hessian eigenvalue");
  DrpmKernelResult r;r.eigenvectors=es.eigenvectors();r.eigenvalues=es.eigenvalues();
  DrpmM6 mean;DrpmV6 variance;
  if(point_covariances.empty()) {
    std::tie(mean,variance)=degeneracy::ComputeNoiseEstimate<double,double>(
      points,normals,weights,normal_covariances,r.eigenvectors,sigma);
  } else {
    if(point_covariances.size()!=points.size())throw std::runtime_error("DRPM covariance count");
    mean.setZero();variance.setZero();
    for(size_t i=0;i<points.size();++i) {
      const Eigen::Matrix3d nx=drpmSkew(normals[i]),px=drpmSkew(points[i]);
      DrpmM6 B=DrpmM6::Zero(),N=DrpmM6::Zero();
      B.topLeftCorner<3,3>()=-nx;B.topRightCorner<3,3>()=px*nx;B.bottomRightCorner<3,3>()=nx;
      N.topLeftCorner<3,3>()=point_covariances[i];N.bottomRightCorner<3,3>()=normal_covariances[i];
      DrpmM6 C=weights[i]*B*N*B.transpose();mean+=C;
      DrpmV6 v;v<<points[i].cross(normals[i]),normals[i];v*=std::sqrt(weights[i]);
      for(int k=0;k<6;++k){double a=r.eigenvectors.col(k).dot(C*r.eigenvectors.col(k)),b=r.eigenvectors.col(k).dot(v);variance[k]+=2*a*a+4*a*b*b;}
    }
  }
  if(!mean.allFinite() || !variance.allFinite() || variance.minCoeff()<=0)
    throw std::runtime_error("DRPM undefined noise distribution");
  r.probabilities=degeneracy::ComputeSignalToNoiseProbabilities<double>(H,mean,variance,r.eigenvectors,snr);
  if(mode==1) {
    r.update=degeneracy::SolveWithSnrProbabilities<double>(r.eigenvectors,r.eigenvalues,rhs,r.probabilities);
  } else if(mode==2) {
    DrpmV6 inverse;
    for(int k=0;k<6;++k)inverse[k]=r.probabilities[k]>0 ? 1.0/r.eigenvalues[k] : 0.0;
    r.update=r.eigenvectors*inverse.asDiagonal()*r.eigenvectors.transpose()*rhs;
  } else throw std::runtime_error("DRPM unknown update mode");
  const DrpmV6 strength=(r.probabilities.array()*r.eigenvalues.array()).matrix();
  // Eq.28: H has already been whitened by sigma_r. Do not include the
  // zero-update regularizer as measurement information, or whiten twice.
  r.sqrt_information=strength.cwiseSqrt().asDiagonal()*r.eigenvectors.transpose();
  r.information=r.sqrt_information.transpose()*r.sqrt_information;
  if(!r.update.allFinite() || !r.information.allFinite())throw std::runtime_error("DRPM invalid solution");
  return r;
}
}}
