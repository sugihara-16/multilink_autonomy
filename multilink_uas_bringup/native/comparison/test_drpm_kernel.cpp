#include "drpm_kernel.hpp"
#include <random>
#include <cassert>
int main(){
 using namespace mimosa::lidar;using V3=Eigen::Vector3d;using M3=Eigen::Matrix3d;
 std::mt19937 gen(109);std::normal_distribution<double>d;
 for(bool plane:{false,true}){
  std::vector<V3> ps,ns;std::vector<M3> cs;std::vector<double> ws;
  DrpmM6 H=DrpmM6::Zero();DrpmV6 rhs=DrpmV6::Zero();
  for(int i=0;i<700;++i){
   V3 p(5*d(gen),5*d(gen),3+d(gen)),n=(plane?V3(.02*d(gen),.02*d(gen),1):V3(d(gen),d(gen),d(gen))).normalized();
   ps.push_back(p);ns.push_back(n);cs.push_back(.001*M3::Identity());ws.push_back(4);
   DrpmV6 j;j<<p.cross(n),n;H+=4*j*j.transpose();rhs+=4*j*.04*d(gen);
  }
  auto r=drpmKernel(H,rhs,ps,ns,ws,cs,{},.03,10,1);
  DrpmM6 mu;DrpmV6 var;std::tie(mu,var)=degeneracy::ComputeNoiseEstimate<double,double>(ps,ns,ws,cs,r.eigenvectors,.03);
  DrpmV6 p=degeneracy::ComputeSignalToNoiseProbabilities<double>(H,mu,var,r.eigenvectors,10);
  DrpmV6 step=degeneracy::SolveWithSnrProbabilities<double>(r.eigenvectors,r.eigenvalues,rhs,p);
  assert((r.probabilities-p).norm()==0 && (r.update-step).norm()==0);
  // Eq.26/27 gives the same update, without counting the prior in Eq.28.
  DrpmV6 regularized=H.ldlt().solve(r.eigenvectors*p.asDiagonal()*r.eigenvectors.transpose()*rhs);
  assert((regularized-r.update).norm()<1e-10);
  assert((r.information-r.eigenvectors*(p.array()*r.eigenvalues.array()).matrix().asDiagonal()*r.eigenvectors.transpose()).norm()/H.norm()<1e-12);
  auto w=drpmKernel(H,rhs,ps,ns,ws,cs,{},.03,10,2);
  DrpmV6 projected=w.eigenvectors.transpose()*w.update,projected_rhs=w.eigenvectors.transpose()*rhs;
  for(int k=0;k<6;++k)assert(std::abs(projected[k]-(w.probabilities[k]>0?projected_rhs[k]/w.eigenvalues[k]:0))<1e-10);
  // Explicit isotropic point covariance equals the upstream path.
  std::vector<M3> pc(ps.size(),.0009*M3::Identity());auto a=drpmKernel(H,rhs,ps,ns,ws,cs,pc,.03,10,1);
  assert((a.probabilities-r.probabilities).norm()<1e-10 && (a.update-r.update).norm()<1e-10);
 }
 DrpmV6 p=DrpmV6::Constant(.1),ones=DrpmV6::Ones();DrpmM6 I=DrpmM6::Identity();
 auto step=degeneracy::SolveWithSnrProbabilities<double>(I,ones,ones,p);assert((step-p).norm()==0);
 std::cout<<"Unmodified official probabilities/update: exact match; Eq26/28, weighted-cost control, isotropic/radial API equivalence passed\n";
}
