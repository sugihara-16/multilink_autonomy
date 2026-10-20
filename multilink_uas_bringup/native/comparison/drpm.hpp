#pragma once
#include <Eigen/Eigenvalues>
#include <boost/math/distributions/normal.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <atomic>

namespace mimosa { namespace lidar {
// Hatleskog & Alexis, DRPM, arXiv:2410.10784v2, Eqs. 14--23, 28, 35.
// This is the information-weighting integration into an existing factor graph.
// No zero-motion regularizer is added: IMU and external VIO constrain weak axes.
inline Eigen::Matrix3d drpmSkew(const Eigen::Vector3d& x) {
  Eigen::Matrix3d m; m<<0,-x.z(),x.y(),x.z(),0,-x.x(),-x.y(),x.x(),0;return m;
}
// Conditional sensor covariance, in the same frame as the ray. The origin
// must be the acquisition-time LiDAR origin, not the body or map origin.
inline Eigen::Matrix3d drpmRadialCovariance(const Eigen::Vector3d& ray, double sigma) {
  if (!ray.allFinite() || ray.squaredNorm() < 1e-12)
    throw std::runtime_error("DRPM invalid acquisition ray");
  const Eigen::Vector3d u = ray.normalized();
  return sigma*sigma*(u*u.transpose());
}
// First-order PCA eigenvector perturbation for independent, potentially
// anisotropic point noise. Eigenvalues use population (1/N) scatter.
// Return small-rotation covariance eta such that delta_n = [n]x eta.
inline Eigen::Matrix3d drpmNormalCovariance(
    const Eigen::Matrix<double,Eigen::Dynamic,3>& centered,
    const Eigen::Vector3d& eigenvalues, const Eigen::Matrix3d& eigenvectors,
    const std::vector<Eigen::Matrix3d>& point_covariances) {
  if (point_covariances.size()!=size_t(centered.rows()))
    throw std::runtime_error("DRPM normal covariance size mismatch");
  const Eigen::Vector3d n=eigenvectors.col(0);
  Eigen::Matrix3d C=Eigen::Matrix3d::Zero();
  for (int j=0;j<centered.rows();++j) {
    const Eigen::Vector3d y=centered.row(j).transpose();
    Eigen::Matrix3d D=Eigen::Matrix3d::Zero();
    for (int a=1;a<3;++a) {
      const Eigen::Vector3d v=eigenvectors.col(a);
      const double gap=std::max(eigenvalues[a]-eigenvalues[0],1e-12);
      D-=v*(y.dot(n)*v.transpose()+y.dot(v)*n.transpose())/(centered.rows()*gap);
    }
    C+=D*point_covariances[j]*D.transpose();
  }
  const Eigen::Matrix3d nx=drpmSkew(n);
  return nx.transpose()*C*nx;
}
inline Eigen::Matrix<double,6,1> drpmReweight(
    Eigen::Matrix<double,6,6>& H, Eigen::Matrix<double,6,1>& b,
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Vector3d>& normals,
    const std::vector<double>& weights,
    const std::vector<Eigen::Matrix3d>& normal_covariances,
    double sigma, double snr,
    const std::vector<Eigen::Matrix3d>& point_covariances = {}) {
  if (!point_covariances.empty() && point_covariances.size()!=points.size())
    throw std::runtime_error("DRPM source covariance size mismatch");
  using M6=Eigen::Matrix<double,6,6>;using V6=Eigen::Matrix<double,6,1>;
  Eigen::SelfAdjointEigenSolver<M6> es(H);
  if(es.info()!=Eigen::Success) throw std::runtime_error("DRPM Hessian eigendecomposition failed");
  const M6 U=es.eigenvectors();M6 mean=M6::Zero();V6 variance=V6::Zero();
  for(size_t i=0;i<points.size();++i) {
    auto nx=drpmSkew(normals[i]);auto px=drpmSkew(points[i]);
    M6 B=M6::Zero(),N=M6::Zero();
    B.topLeftCorner<3,3>()=-nx;B.topRightCorner<3,3>()=px*nx;B.bottomRightCorner<3,3>()=nx;
    if (point_covariances.empty()) N.topLeftCorner<3,3>()=Eigen::Matrix3d::Identity()*sigma*sigma;
    else N.topLeftCorner<3,3>()=point_covariances[i];
    N.bottomRightCorner<3,3>()=normal_covariances[i];
    const M6 C=weights[i]*B*N*B.transpose();mean+=C;
    V6 v;v.head<3>()=points[i].cross(normals[i]);v.tail<3>()=normals[i];v*=std::sqrt(weights[i]);
    for(int k=0;k<6;++k) {double a=std::max(0.0,U.col(k).dot(C*U.col(k)));double q=U.col(k).dot(v);variance[k]+=2*a*a+4*a*q*q;}
  }
  V6 probabilities;
  for(int k=0;k<6;++k) {
    double test=es.eigenvalues()[k]/(1+snr),mu=U.col(k).dot(mean*U.col(k));
    probabilities[k]=variance[k]>1e-30 ? boost::math::cdf(boost::math::normal_distribution<double>(mu,std::sqrt(variance[k])),test) : double(test>mu);
  }
  // W-weighted residual cost: H'=U P Lambda U', b'=U P U' b.
  // H already includes 1/sigma_r^2, so Eq.28 must not whiten it again.
  b=U*probabilities.asDiagonal()*U.transpose()*b;
  H=U*(probabilities.array()*es.eigenvalues().array().max(0.0)).matrix().asDiagonal()*U.transpose();
  static std::atomic<size_t> calls{0};if((++calls)%100==0)std::cout<<"DRPM probabilities "<<probabilities.transpose()<<std::endl;
  return probabilities;
}
}}
