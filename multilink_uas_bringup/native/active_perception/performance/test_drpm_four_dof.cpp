#include "drpm_four_dof_kernel.hpp"
#include <Eigen/Geometry>
#include <random>
#include <iostream>
#include <stdexcept>
using namespace mimosa::lidar;
using V3=Eigen::Vector3d;using M3=Eigen::Matrix3d;
void require(bool b,const char* message){if(!b)throw std::runtime_error(message);}
int main(){
  std::mt19937 rng(923);std::normal_distribution<double> gaussian;
  auto random=[&](){return V3(gaussian(rng),gaussian(rng),gaussian(rng));};
  for(int trial=0;trial<40;++trial){
    std::vector<V3> p,n;std::vector<M3> cn,cp;std::vector<double>w;
    DrpmM6 H=DrpmM6::Zero();DrpmV6 rhs=DrpmV6::Zero();
    for(int i=0;i<80;++i){
      p.push_back(3.*random());n.push_back(random().normalized());w.push_back(1.+.1*(i%7));
      cn.push_back(M3::Identity()*.04);cp.push_back(drpmRadialCovariance(p.back(),.03));
      DrpmV6 j;j<<p.back().cross(n.back()),n.back();H+=w.back()*j*j.transpose();rhs+=.01*gaussian(rng)*j;
    }
    const V3 z=random().normalized();
    const auto r=drpmFourDofKernel(H,rhs,p,n,w,cn,cp,.03,10.,1,z);
    require(r.eigenvalues.head<2>().isZero(0.) && r.probabilities.head<2>().isZero(0.),"Excluded axes were given information");
    require(r.update.head<3>().cross(z).norm()<1e-12,"Update changes tilt");
    require((r.information-r.information.transpose()).norm()<1e-10,"Asymmetric information");
    require(Eigen::SelfAdjointEigenSolver<DrpmM6>(r.information).eigenvalues().minCoeff()>-1e-9,"Information is not PSD");
    require((r.information*r.eigenvectors.leftCols<2>()).norm()<1e-9,"Excluded rotation has nonzero information");
    DrpmV6 reference=DrpmV6::Zero();
    for(int k=2;k<6;++k)reference+=r.eigenvectors.col(k)*(r.probabilities[k]/r.eigenvalues[k])*r.eigenvectors.col(k).dot(rhs);
    require((r.update-reference).norm()<1e-12,"Official Eq.24 mismatch");
    const M3 R=Eigen::AngleAxisd(.7,random().normalized()).toRotationMatrix();
    DrpmM6 G=DrpmM6::Zero();G.topLeftCorner<3,3>()=R;G.bottomRightCorner<3,3>()=R;
    auto pp=p,nn=n;auto ccn=cn,ccp=cp;
    for(size_t i=0;i<p.size();++i){pp[i]=R*p[i];nn[i]=R*n[i];ccn[i]=R*cn[i]*R.transpose();ccp[i]=R*cp[i]*R.transpose();}
    const auto transformed=drpmFourDofKernel(G*H*G.transpose(),G*rhs,pp,nn,w,ccn,ccp,.03,10.,1,R*z);
    require((transformed.update-G*r.update).norm()<1e-9,"Update depends on arbitrary local frame");
    require((transformed.information-G*r.information*G.transpose()).norm()<1e-8,"Information frame mismatch");
    const M3 world_body=Eigen::Quaterniond::FromTwoVectors(z,V3::UnitZ()).toRotationMatrix();
    const double angle=r.update.head<3>().norm();
    const M3 updated=world_body*Eigen::AngleAxisd(angle,angle>1e-15?r.update.head<3>().normalized():z).toRotationMatrix();
    require((updated.transpose()*V3::UnitZ()-z).norm()<1e-12,"Finite update changes body gravity direction");
    // Isotropic-input noise probabilities must match the official formula
    // on the retained eigenvectors, even though the excluded axes are padded.
    const auto isotropic=drpmFourDofKernel(H,rhs,p,n,w,cn,{},.03,10.,1,z);
    DrpmM6 mean;DrpmV6 variance;
    std::tie(mean,variance)=degeneracy::ComputeNoiseEstimate<double,double>(p,n,w,cn,isotropic.eigenvectors,.03);
    const auto probabilities=degeneracy::ComputeSignalToNoiseProbabilities<double>(H,mean,variance,isotropic.eigenvectors,10.);
    require((probabilities.tail<4>()-isotropic.probabilities.tail<4>()).norm()<1e-10,"Noise probabilities disagree with official formula");
    // Negligible noise recovers constrained least squares, with no extra gain.
    auto quiet=cn;for(auto& c:quiet)c=M3::Identity()*1e-12;
    const auto certain=drpmFourDofKernel(H,rhs,p,n,w,quiet,{},1e-6,10.,1,z);
    Eigen::Matrix<double,6,4> Q=Eigen::Matrix<double,6,4>::Zero();Q.topLeftCorner<3,1>()=z;Q.bottomRightCorner<3,3>().setIdentity();
    const DrpmV6 least=Q*(Q.transpose()*H*Q).ldlt().solve(Q.transpose()*rhs);
    require((certain.update-least).norm()<1e-10,"Constrained noiseless least-squares mismatch");
  }
  {
    // Long tunnel: the small apparent forward constraint is produced by
    // uncertain surface normals. It must not cause a large forward update,
    // while the transverse constraints remain usable.
    std::vector<V3> p,n;std::vector<M3> cn,cp;std::vector<double>w;
    DrpmM6 H=DrpmM6::Zero();
    for(int i=0;i<160;++i){
      p.push_back(random()*3.);V3 normal=random();normal.x()*=.001;normal.normalize();n.push_back(normal);
      w.push_back(1.);cn.push_back(M3::Identity()*.0004);cp.push_back(drpmRadialCovariance(p.back(),.03));
      DrpmV6 j;j<<p.back().cross(normal),normal;H+=j*j.transpose();
    }
    DrpmV6 target=DrpmV6::Zero();target[3]=.1;
    const auto r=drpmFourDofKernel(H,H*target,p,n,w,cn,cp,.03,10.,1,V3::UnitZ());
    require(std::abs(r.update[3])<.01,"Noise-dominated tunnel-axis update not suppressed");
    require(r.probabilities.tail<4>().minCoeff()<1e-8 && r.probabilities.tail<4>().maxCoeff()>.99,"Tunnel does not retain strong and suppress weak directions");
  }
  bool rejected=false;
  try{drpmFourDofKernel(DrpmM6::Zero(),DrpmV6::Zero(),{},{},{},{},{},.03,10.,1,V3::UnitZ());}catch(const std::runtime_error&){rejected=true;}
  require(rejected,"Rank-deficient reduced system accepted");
  std::cout<<"PASS 40 random geometry/frame/finite-gravity/equation/PSD cases; noisy tunnel direction suppressed; rank-deficient input rejected\n";
}
