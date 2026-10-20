// Thin C ABI around the unmodified official FIF bearing Jacobian.
// This is the direct landmark reference path, not its voxel approximation.
#include <vi_utils/vi_jacobians.h>
#include <Eigen/Eigenvalues>
#include <cmath>
using M3 = Eigen::Matrix3d;
using V3 = Eigen::Vector3d;
using Row3 = Eigen::Matrix<double,3,3,Eigen::RowMajor>;
using Row36 = Eigen::Matrix<double,3,6,Eigen::RowMajor>;
using Row6 = Eigen::Matrix<double,6,6,Eigen::RowMajor>;
// Inputs: landmark positions/covariances in CURRENT BODY coordinates,
// T_B_C optical frame, normalized-bearing noise standard deviation.
// Output uses BODY RIGHT perturbations [rotation(rad), translation(m)].
extern "C" int fif_information(int n, const double* points, const double* covs,
    const double* Rbc_data, const double* tbc_data, double sigma,
    double* information, double* jacobians, double* contributions) {
  if(n<0 || !(sigma>0) || !std::isfinite(sigma)) return -1;
  const M3 Rbc=Eigen::Map<const Row3>(Rbc_data);
  const V3 tbc=Eigen::Map<const V3>(tbc_data);
  rpg::Pose Tbc(rpg::Rotation(Eigen::Quaterniond(Rbc)),tbc);
  rpg::Pose identity; identity.setIdentity();
  Eigen::Matrix<double,6,6> H=Eigen::Matrix<double,6,6>::Zero();
  for(int i=0;i<n;++i){
    V3 p=Eigen::Map<const V3>(points+3*i);
    const V3 pc=Tbc.inverse()*p;
    if(!p.allFinite() || pc.norm()<1e-6) return -2;
    auto official=vi_utils::jacobians::dBearing_dIMUPose(p,identity,Tbc.inverse());
    Eigen::Matrix<double,3,6> J;
    J.leftCols<3>()=official.rightCols<3>();
    J.rightCols<3>()=official.leftCols<3>();
    const M3 C=Eigen::Map<const Row3>(covs+9*i);
    if(!C.allFinite() || Eigen::SelfAdjointEigenSolver<M3>(C).eigenvalues().minCoeff() < -1e-9) return -3;
    const M3 D=vi_utils::jacobians::dBearing_dPointCam(pc)*Rbc.transpose();
    const M3 S=sigma*sigma*M3::Identity()+D*C*D.transpose();
    const Eigen::Matrix<double,6,6> Hi=J.transpose()*S.ldlt().solve(J);
    H+=Hi;
    if(contributions){Eigen::Map<Row6> out(contributions+36*i);out=Hi;}
    if(jacobians) { Eigen::Map<Row36> out(jacobians+18*i);out=J; }
  }
  Eigen::Map<Row6> out(information);out=H;
  return 0;
}
