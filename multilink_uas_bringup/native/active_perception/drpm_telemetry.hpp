#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
// Read-only telemetry. Does not change registration, weighting or graph factors.
// Layout: version=1, stamp, T_W_L(4x4), effective H_L(6x6), U_L(6x6), p(6).
inline void publishActiveDrpm(double stamp,const gtsam::Pose3& pose,
                            const mimosa::lidar::DrpmKernelResult& r){
  static ros::NodeHandle nh;
  static auto pub=nh.advertise<std_msgs::Float64MultiArray>("/active/lidar_information",5);
  std_msgs::Float64MultiArray m;m.data={1.,stamp};
  auto append=[&](const auto& a){for(int i=0;i<a.rows();++i)for(int j=0;j<a.cols();++j)m.data.push_back(a(i,j));};
  append(pose.matrix());append(r.information);append(r.eigenvectors);append(r.probabilities);
  pub.publish(m);
}
