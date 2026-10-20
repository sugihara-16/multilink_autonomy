// Diagnostic replay harness. Uses the installed filter's headers/library, no ROS queues.
// Input: little-endian uint32 kind, uint64 source stamp, then six doubles (IMU)
// or 480*360 mono8 bytes (camera kind 1..3). Config/measurement stamps unchanged.
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <memory>
#include <ros/ros.h>
#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode.hpp"
using Filter=rovio::RovioFilter<rovio::FilterState<ROVIO_NMAXFEATURE,4,6,3,0>>;
int main(int argc,char** argv){
 if(argc<4)return 2;
 ros::init(argc,argv,"rovio_audit");ros::NodeHandle nh,np("~");
 auto filter=std::make_shared<Filter>();filter->readFromInfo(argv[1]);
 for(int i=0;i<3;i++)filter->cameraCalibrationFile_[i]=std::string(argv[2])+"/cam"+std::to_string(i)+".yaml";
 filter->refreshProperties();rovio::RovioNode<Filter> node(nh,np,filter);
 node.forceOdometryPublishing_=true;
 std::ofstream od(std::string(argv[3])+"_vio.csv"),dg(std::string(argv[3])+"_state.csv"),ft(std::string(argv[3])+"_features.csv");
 od<<std::setprecision(17);dg<<std::setprecision(17);ft<<std::setprecision(10);
 od<<"stamp_ns,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
 dg<<"stamp_ns,acb_x,acb_y,acb_z,gyb_x,gyb_y,gyb_z,acb_var_x,acb_var_y,acb_var_z,n0,n1,n2\n";
 ft<<"stamp_ns,id,cam,depth,bx,by,bz,status0,status1,status2\n";
 std::map<uint64_t,std::array<sensor_msgs::ImageConstPtr,3>> images;
 double last=-1;uint64_t input[4]={},sets=0;
 uint32_t kind;uint64_t stamp;
 while(std::cin.read(reinterpret_cast<char*>(&kind),sizeof(kind))){
  if(!std::cin.read(reinterpret_cast<char*>(&stamp),sizeof(stamp)))return 3;
  if(kind>3)return 4;++input[kind];
  if(kind==0){
   double x[6];if(!std::cin.read(reinterpret_cast<char*>(x),sizeof(x)))return 5;
   sensor_msgs::ImuPtr m(new sensor_msgs::Imu);m->header.stamp.fromNSec(stamp);m->linear_acceleration.x=x[0];m->linear_acceleration.y=x[1];m->linear_acceleration.z=x[2];m->angular_velocity.x=x[3];m->angular_velocity.y=x[4];m->angular_velocity.z=x[5];m->orientation_covariance[0]=-1;
   node.imuCallback(m);
  }else{
   sensor_msgs::ImagePtr m(new sensor_msgs::Image);m->header.stamp.fromNSec(stamp);m->height=360;m->width=480;m->step=480;m->encoding="mono8";m->data.resize(480*360);
   if(!std::cin.read(reinterpret_cast<char*>(m->data.data()),m->data.size()))return 6;
   images[stamp][kind-1]=m;
   auto& batch=images[stamp];if(batch[0]&&batch[1]&&batch[2]){for(int c=0;c<3;c++)node.imgCallback(batch[c],c);images.erase(stamp);++sets;}
  }
  if(filter->safe_.t_<=last || !node.gotFirstMessages_)continue;
  last=filter->safe_.t_;auto&m=node.odometryMsg_;auto&p=m.pose.pose.position;auto&q=m.pose.pose.orientation;auto&v=m.twist.twist.linear;auto&w=m.twist.twist.angular;auto t=m.header.stamp.toNSec();
  od<<t<<','<<p.x<<','<<p.y<<','<<p.z<<','<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<','<<v.x<<','<<v.y<<','<<v.z<<','<<w.x<<','<<w.y<<','<<w.z<<'\n';
  auto& s=filter->safe_.state_;int counts[3]={};
  for(int i=0;i<ROVIO_NMAXFEATURE;i++)if(filter->safe_.fsm_.isValid_[i]){
   auto& f=filter->safe_.fsm_.features_[i];int c=f.mpCoordinates_->camID_;++counts[c];auto b=s.CfP(i).get_nor().getVec();
   ft<<t<<','<<f.idx_<<','<<c<<','<<s.dep(i).getDistance()<<','<<b[0]<<','<<b[1]<<','<<b[2];for(int j=0;j<3;j++)ft<<','<<f.mpStatistics_->status_[j];ft<<'\n';
  }
  dg<<t;for(int j=0;j<3;j++)dg<<','<<s.acb()[j];for(int j=0;j<3;j++)dg<<','<<s.gyb()[j];
  for(int j=0;j<3;j++){int k=Filter::mtState::template getId<Filter::mtState::_acb>()+j;dg<<','<<filter->safe_.cov_(k,k);}
  for(int c:counts)dg<<','<<c;dg<<'\n';
 }
 std::cerr<<"AUDIT_INPUT "<<input[0]<<' '<<input[1]<<' '<<input[2]<<' '<<input[3]<<" sets "<<sets<<" incomplete "<<images.size()<<std::endl;
 return images.empty()?0:7;
}
