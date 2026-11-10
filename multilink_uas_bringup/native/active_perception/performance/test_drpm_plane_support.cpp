// Regression: inadequate plane support is rejected, larger measured support is usable.
#include "mimosa/lidar/official_drpm/degeneracy.h"
#include <iostream>
#include <stdexcept>
int main(){
 Eigen::Matrix<double,3,5> small;
 small<<3,3,3,3,3, 0,.1,-.1,0,0, 0,0,0,.1,-.1;
 auto [n5,var5,d5,c5]=degeneracy::EstimateNormal<5,double>(small,.03,true);
 if(!(var5>.01))throw std::runtime_error("Five close samples must fail the 0.1rad uncertainty bound");
 Eigen::Matrix<double,3,20> large;
 for(int i=0;i<20;++i)large.col(i)<<3.,(i%5-2)*.15,(i/5-1.5)*.15;
 auto [n20,var20,d20,c20]=degeneracy::EstimateNormal<20,double>(large,.03,true);
 if(!(var20<.01 && std::abs(n20.x())>1.-1e-12 && std::abs(std::abs(d20)-3.)<1e-12))throw std::runtime_error("Twenty spread samples should estimate the plane with bounded uncertainty");
 Eigen::AngleAxisd aa(.7,Eigen::Vector3d(1,2,3).normalized());Eigen::Matrix<double,3,20> rotated=aa.toRotationMatrix()*large;
 auto [nr,vr,dr,cr]=degeneracy::EstimateNormal<20,double>(rotated,.03,true);
 if(std::abs(vr-var20)>1e-10 || std::abs(nr.dot(aa*n20))<1.-1e-10)throw std::runtime_error("Support validation must not depend on world axes");
 std::cout<<"PASS: inadequate support, expanded support, arbitrary rotation; variances "<<var5<<" "<<var20<<"\n";
}
