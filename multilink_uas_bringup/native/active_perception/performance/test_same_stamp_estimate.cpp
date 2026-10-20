#include "mimosa/graph/manager.hpp"
#include "mimosa/imu/manager.hpp"
#include <gtsam/slam/PriorFactor.h>
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv){
  ros::init(argc,argv,"same_stamp_estimate_test");ros::NodeHandle nh("~");
  bool expect_stale=false;nh.param("expect_stale",expect_stale,false);
  auto imu=std::make_shared<mimosa::imu::Manager>(nh);
  auto graph=std::make_shared<mimosa::graph::Manager>(nh,imu);
  for(int i=0;i<=2100;++i){
    auto msg=boost::make_shared<sensor_msgs::Imu>();msg->header.stamp.fromSec(1.+.001*i);
    msg->linear_acceleration.z=9.8;imu->callback(msg);
  }
  gtsam::Key key;graph->declare(3.,key,true);
  auto prior=gtsam::noiseModel::Isotropic::Sigma(3,.1);
  gtsam::NonlinearFactorGraph factors;factors.add(gtsam::PriorFactor<gtsam::Vector3>(mimosa::V(key),gtsam::Vector3(1,0,0),prior));
  gtsam::Values optimized;graph->define(factors,optimized,mimosa::graph::Manager::DeclarationResult::SUCCESS_SAME_KEY);
  mimosa::State state;graph->getCurrentState(state);
  double difference=(optimized.at<gtsam::Vector3>(mimosa::V(key))-state.navState().velocity()).norm();
  if(expect_stale){
    if(difference<.5)throw std::runtime_error("Original stale-cache regression was not reproduced");
    std::cout<<"REPRODUCED original same-timestamp cached/state velocity difference "<<difference<<" m/s\n";
  }else{
    if(difference>1e-12)throw std::runtime_error("Same-timestamp estimate remains stale");
    auto begin=reinterpret_cast<void(*)()>(dlsym(RTLD_DEFAULT,"mimosa_begin_sensor_batch"));
    auto end=reinterpret_cast<void(*)()>(dlsym(RTLD_DEFAULT,"mimosa_end_sensor_batch"));
    if(!begin||!end)throw std::runtime_error("Batch API missing");
    begin();
    factors=gtsam::NonlinearFactorGraph();factors.add(gtsam::PriorFactor<gtsam::Vector3>(mimosa::V(key),gtsam::Vector3(2,0,0),prior));
    graph->define(factors,optimized,mimosa::graph::Manager::DeclarationResult::SUCCESS_SAME_KEY);
    graph->getCurrentState(state);
    if((optimized.at<gtsam::Vector3>(mimosa::V(key))-state.navState().velocity()).norm()>1e-12)
      throw std::runtime_error("Batched second estimate remains stale");
    end();
    bool rejected=false;try{end();}catch(const std::runtime_error&){rejected=true;}
    if(!rejected)throw std::runtime_error("Unmatched batch end was not rejected");
    std::cout<<"PASS actual graph: repeated timestamp refresh and batched final estimate; velocity "<<state.navState().velocity().transpose()<<"\n";
  }
  ros::shutdown();return 0;
}
