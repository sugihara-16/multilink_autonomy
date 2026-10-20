#include "mimosa/graph/manager.hpp"
#include "mimosa/imu/manager.hpp"
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <stdexcept>
extern "C" void mimosa_begin_sensor_batch();
extern "C" void mimosa_end_sensor_batch();

int main(int argc,char** argv){
  ros::init(argc,argv,"batched_publication_test");ros::NodeHandle nh("~");
  auto imu=std::make_shared<mimosa::imu::Manager>(nh);
  auto graph=std::make_shared<mimosa::graph::Manager>(nh,imu);
  std::vector<nav_msgs::Odometry> published;
  auto sub=nh.subscribe<nav_msgs::Odometry>("graph/odometry",10,
    [&](const nav_msgs::Odometry::ConstPtr& msg){published.push_back(*msg);});
  auto spin=[](){for(int i=0;i<20;++i){ros::spinOnce();ros::WallDuration(.01).sleep();}};
  for(int i=0;i<20 && sub.getNumPublishers()==0;++i)spin();
  if(sub.getNumPublishers()!=1)throw std::runtime_error("Actual graph publisher unavailable");
  for(int i=0;i<=2200;++i){
    auto msg=boost::make_shared<sensor_msgs::Imu>();msg->header.stamp.fromSec(1.+.001*i);
    msg->linear_acceleration.z=9.8;imu->callback(msg);
  }
  gtsam::Key key;graph->declare(3.,key,true);spin();published.clear();
  mimosa_begin_sensor_batch();
  auto first=graph->declare(3.1,key,false);
  gtsam::Values optimized;auto noise=gtsam::noiseModel::Isotropic::Sigma(3,.1);
  gtsam::NonlinearFactorGraph factors;
  factors.add(gtsam::PriorFactor<gtsam::Vector3>(mimosa::V(key),gtsam::Vector3(1,0,0),noise));
  graph->define(factors,optimized,first);spin();
  if(!published.empty())throw std::runtime_error("Intermediate estimate was published");
  factors=gtsam::NonlinearFactorGraph();
  factors.add(gtsam::PriorFactor<gtsam::Vector3>(mimosa::V(key),gtsam::Vector3(2,0,0),noise));
  graph->define(factors,optimized,mimosa::graph::Manager::DeclarationResult::SUCCESS_SAME_KEY);
  mimosa::State state;graph->getCurrentState(state);spin();
  if(!published.empty())throw std::runtime_error("Same-key intermediate estimate was published");
  mimosa_end_sensor_batch();spin();
  if(published.size()!=1)throw std::runtime_error("Expected exactly one final publication");
  const auto& msg=published.front();
  gtsam::Vector3 velocity(msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z);
  if(std::abs(msg.header.stamp.toSec()-3.1)>1e-12 ||
      (velocity-state.navState().velocity()).norm()>1e-12 ||
      (optimized.at<gtsam::Vector3>(mimosa::V(key))-velocity).norm()>1e-12)
    throw std::runtime_error("Published estimate does not match the final graph estimate");
  std::cout<<"PASS no intermediate publication, one final output at exact original timestamp, fresh cache/state/published velocity identical\n";
  ros::shutdown();
}
