#include "drpm.hpp"
#include <random>
#include <cassert>
int main(){
using namespace mimosa::lidar; using V3=Eigen::Vector3d;using M3=Eigen::Matrix3d;using M6=Eigen::Matrix<double,6,6>;using V6=Eigen::Matrix<double,6,1>;
std::mt19937 rng(41);std::normal_distribution<double> n(0,1);
for(bool plane:{false,true}){
std::vector<V3> ps,ns;std::vector<M3> cs;std::vector<double> ws;M6 H=M6::Zero();V6 b=V6::Ones();
for(int i=0;i<4000;i++){V3 p(5*n(rng),5*n(rng),plane?0:5*n(rng));V3 normal=plane?V3(0,0,1):V3(n(rng),n(rng),n(rng)).normalized();normal=(normal+V3(.01*n(rng),.01*n(rng),.01*n(rng))).normalized();ps.push_back(p);ns.push_back(normal);cs.push_back(M3::Identity()*.0001);ws.push_back(1.0);V6 j;j.head<3>()=p.cross(normal);j.tail<3>()=normal;H+=j*j.transpose();}
M6 old=H;auto prob=drpmReweight(H,b,ps,ns,ws,cs,.03,10.0);std::cout<<plane<<" "<<prob.transpose()<<std::endl;assert(H.allFinite()&&b.allFinite());Eigen::SelfAdjointEigenSolver<M6> ev(H);assert(ev.eigenvalues().minCoeff()>-1e-7);if(plane){assert((prob.array()<.01).count()==3);assert(H(3,3)<old(3,3)*.02&&H(4,4)<old(4,4)*.02);}else{assert(prob.minCoeff()>.99);assert((old-H).norm()/old.norm()<1e-9);}}
}
