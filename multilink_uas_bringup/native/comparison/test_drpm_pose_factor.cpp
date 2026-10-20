#include "drpm_pose_factor.hpp"
#include <cassert>
#include <iostream>
int main(){
 using namespace gtsam;using mimosa::lidar::DrpmPoseFactor;
 Vector6 e;e<<.03,-.02,.1,2,-3,1;Pose3 body=Pose3::Expmap(e);
 e<<.1,.2,-.3,-.1925,.001,.049;Pose3 extrinsic=Pose3::Expmap(e);
 Matrix6 S=Matrix6::Random();S.row(0).setZero();
 for(int i=0;i<20;++i){Vector6 noise=Vector6::Random()*.05;Pose3 target=body*extrinsic*Pose3::Expmap(noise);DrpmPoseFactor f(0,target,extrinsic,S);Matrix H;Vector error=f.evaluateError(body,&H);Matrix numerical(6,6);
  for(int k=0;k<6;++k){Vector6 step=Vector6::Zero();step[k]=1e-6;numerical.col(k)=(f.evaluateError(body*Pose3::Expmap(step))-f.evaluateError(body*Pose3::Expmap(-step)))/2e-6;}
  assert((H-numerical).norm()<2e-8);assert(f.evaluateError(target*extrinsic.inverse()).norm()<1e-12);assert(std::abs(error[0])<1e-15);
 }
 std::cout<<"Pose factor: 20 SE3 finite-difference Jacobians, sensor extrinsic, zero residual at target and rank-deficient information passed\n";
}
