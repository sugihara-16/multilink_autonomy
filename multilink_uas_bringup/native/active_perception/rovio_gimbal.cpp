#include "gimbal_rovio.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <fstream>
#include <iomanip>
#include <rovio/RovioFilter.hpp>
#include <rovio/RovioNode.hpp>
using Filter=rovio::RovioFilter<rovio::FilterState<25,4,6,1,0>>;
int main(int argc,char** argv){
  ros::init(argc,argv,"rovio_gimbal");ros::NodeHandle nh,np("~");
  std::string config,calibration;np.getParam("config",config);np.getParam("calibration",calibration);
  auto filter=std::make_shared<Filter>();filter->readFromInfo(config);
  filter->cameraCalibrationFile_[0]=calibration;filter->refreshProperties();
  rovio::RovioNode<Filter> node(nh,np,filter);
  // Own queues so no image is processed before its actual encoder reading.
  node.subImu_.shutdown();for(auto& s:node.subImages_)s.shutdown();
  node.subGroundtruth_.shutdown();node.subGroundtruthOdometry_.shutdown();node.subVelocity_.shutdown();
  node.forceOdometryPublishing_=true;node.forcePclPublishing_=true;
  double gravity;np.param("gravity_magnitude",gravity,9.81);
  if(!std::isfinite(gravity) || gravity<=0)throw std::runtime_error("Invalid gravity");
  filter->mPrediction_.g_=V3D(0,0,-gravity);
  for(const auto& item : {std::make_pair("acc_bias_tau",&active_imu::acc_tau()),std::make_pair("gyro_bias_tau",&active_imu::gyro_tau())}){
    std::vector<double> tau;if(!np.hasParam(item.first))continue;
    if(!np.getParam(item.first,tau))throw std::runtime_error("Bias correlation times must be a typed numeric array, loaded with rosparam");
    if(tau.size()!=3)throw std::runtime_error("Expected three bias correlation times");
    for(int i=0;i<3;++i){if(!std::isfinite(tau[i]) || tau[i]<0)throw std::runtime_error("Invalid bias correlation time");(*item.second)[i]=tau[i];}
  }
  ROS_INFO_STREAM("IMU model: gravity="<<gravity<<" acc_tau="<<Eigen::Map<const V3D>(active_imu::acc_tau().data()).transpose()<<" gyro_tau="<<Eigen::Map<const V3D>(active_imu::gyro_tau().data()).transpose());
  std::map<uint64_t,sensor_msgs::ImageConstPtr> images;
  uint64_t imu_time=0,last_image=0;
  double stationary_duration=0.;np.param("stationary_initialization_duration",stationary_duration,0.);
  bool stationary_ready=stationary_duration<=0.;double stationary_start=-1.;size_t stationary_count=0;
  V3D stationary_acc=V3D::Zero(),stationary_gyro=V3D::Zero();
  std::string stationary_mode,record_path,replay_path,diagnostic_path,feature_path;
  np.param<std::string>("stationary_mode",stationary_mode,"legacy");
  np.param<std::string>("input_record",record_path,"");
  np.param<std::string>("input_replay",replay_path,"");
  np.param<std::string>("diagnostic_csv",diagnostic_path,"");
  np.param<std::string>("feature_csv",feature_path,"");
  if(stationary_mode!="legacy" && stationary_mode!="mean" && stationary_mode!="wait")throw std::runtime_error("Unknown stationary initialization mode");
  if(!record_path.empty() && !replay_path.empty())throw std::runtime_error("Record and replay are exclusive");
  std::ofstream record,diagnostic,feature_log;
  if(!feature_path.empty()){feature_log.open(feature_path);if(!feature_log)throw std::runtime_error("Cannot open feature log");feature_log<<std::setprecision(17)<<"stamp_ns,id,slot,nx,ny,nz,depth_m,status,age_s,tracked_count,total_count\n";}
  if(!record_path.empty()){record.open(record_path,std::ios::binary);if(!record)throw std::runtime_error("Cannot open input record");}
  if(!diagnostic_path.empty()){
    diagnostic.open(diagnostic_path);if(!diagnostic)throw std::runtime_error("Cannot open diagnostic output");
    diagnostic<<std::setprecision(17)<<"stamp_ns,x,y,z,qx,qy,qz,qw,acb_x,acb_y,acb_z,gyb_x,gyb_y,gyb_z,features,median_depth\n";
  }
  auto save=[&](uint32_t kind,const auto& message){
    if(!record.is_open())return;
    uint32_t size=ros::serialization::serializationLength(*message);
    std::vector<uint8_t> bytes(size);ros::serialization::OStream stream(bytes.data(),size);
    ros::serialization::serialize(stream,*message);
    record.write(reinterpret_cast<const char*>(&kind),4);record.write(reinterpret_cast<const char*>(&size),4);
    record.write(reinterpret_cast<const char*>(bytes.data()),size);
    if(!record)throw std::runtime_error("Input recording failed");
  };
  double last_diagnostic=-1.;
  auto diagnose=[&](){
    if(!diagnostic.is_open() || filter->safe_.t_<=last_diagnostic || !node.gotFirstMessages_)return;
    last_diagnostic=filter->safe_.t_;const auto&m=node.odometryMsg_;const auto&p=m.pose.pose.position;const auto&q=m.pose.pose.orientation;
    diagnostic<<m.header.stamp.toNSec()<<','<<p.x<<','<<p.y<<','<<p.z<<','<<q.x<<','<<q.y<<','<<q.z<<','<<q.w;
    auto&s=filter->safe_.state_;for(int i=0;i<3;++i)diagnostic<<','<<s.acb()[i];for(int i=0;i<3;++i)diagnostic<<','<<s.gyb()[i];
    std::vector<double> depth;for(int i=0;i<25;++i)if(filter->safe_.fsm_.isValid_[i])depth.push_back(s.dep(i).getDistance());
    std::sort(depth.begin(),depth.end());diagnostic<<','<<depth.size()<<','<<(depth.empty()?0.:depth[depth.size()/2])<<'\n';
    if(feature_log.is_open())for(int i=0;i<25;++i)if(filter->safe_.fsm_.isValid_[i]){
      const auto& f=filter->safe_.fsm_.features_[i];const auto& stats=*f.mpStatistics_;
      const auto& n=s.CfP(i).get_nor().getVec();
      feature_log<<m.header.stamp.toNSec()<<','<<f.idx_<<','<<i<<','<<n.x()<<','<<n.y()<<','<<n.z()<<','<<s.dep(i).getDistance()<<','<<int(stats.status_[0])<<','<<filter->safe_.t_-stats.initTime_<<','<<stats.trackedCount_<<','<<stats.totCount_<<'\n';
    }
  };
  auto drain=[&](){
    while(!images.empty()){
      auto it=images.begin();auto t=it->first;
      if(t<=last_image || t<(uint64_t)llround(filter->safe_.t_*1e9)){images.erase(it);continue;}
      if(t>imu_time || !active_gimbal::samples().count(t))break;
      node.imgCallback(it->second,0);last_image=t;images.erase(it);
    }
    if(images.size()>40)throw std::runtime_error("Image/extrinsic synchronization timed out");
    auto& samples=active_gimbal::samples();
    while(!samples.empty() && samples.begin()->first+60000000000LL<(long long)last_image)samples.erase(samples.begin());
  };
  auto on_imu=[&](const sensor_msgs::ImuConstPtr& m){
    save(0,m);
    sensor_msgs::ImuConstPtr initialization_message=m;
    if(!stationary_ready){
      const double t=m->header.stamp.toSec();if(stationary_start<0)stationary_start=t;
      stationary_acc+=V3D(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z);
      stationary_gyro+=V3D(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);++stationary_count;
      if(t-stationary_start<stationary_duration)return;
      const V3D acc=stationary_acc/double(stationary_count),gyro=stationary_gyro/double(stationary_count);
      if(stationary_mode!="wait"){
      if(acc.norm()<8 || acc.norm()>12 || gyro.norm()>.1)throw std::runtime_error("Stationary IMU initialization rejected");
      // Gyro bias is observable at rest. Only the radial accelerometer bias is
      // observable without known orientation; do not invent transverse bias.
      filter->init_.state_.gyb()=gyro;
      filter->init_.state_.acb()=acc*(1.-gravity/acc.norm());
      ROS_INFO_STREAM("Stationary IMU samples="<<stationary_count<<" gyro="<<gyro.transpose()<<" radial_acc_bias="<<filter->init_.state_.acb().transpose());
      }else{
        ROS_INFO("Initialization delay complete; use upstream accelerometer initialization without assuming a stationary body or assigning measured rotation to bias");
      }
      stationary_ready=true;images.clear();
      if(stationary_mode=="mean"){
        sensor_msgs::ImuPtr averaged(new sensor_msgs::Imu(*m));
        averaged->linear_acceleration.x=acc.x();averaged->linear_acceleration.y=acc.y();averaged->linear_acceleration.z=acc.z();
        initialization_message=averaged;
      }
    }
    if(m->header.stamp.toNSec()<=imu_time)return;
    node.imuCallback(initialization_message);imu_time=m->header.stamp.toNSec();drain();diagnose();
  };
  auto on_image=[&](const sensor_msgs::ImageConstPtr& m){save(1,m);if(!stationary_ready || imu_time==0)return;images[m->header.stamp.toNSec()]=m;drain();diagnose();};
  auto on_extr=[&](const geometry_msgs::PoseStamped::ConstPtr& m){
    save(2,m);
    const auto& q=m->pose.orientation;const auto& p=m->pose.position;
    if(std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-6)throw std::runtime_error("Invalid encoder rotation");
    active_gimbal::samples()[m->header.stamp.toNSec()]={QPD(q.w,q.x,q.y,q.z).inverted(),V3D(p.x,p.y,p.z)};
    drain();diagnose();
  };
  if(!replay_path.empty()){
    std::ifstream input(replay_path,std::ios::binary);if(!input)throw std::runtime_error("Cannot open replay");
    uint32_t kind,size;uint64_t counts[3]={};
    while(input.read(reinterpret_cast<char*>(&kind),4)){
      if(!input.read(reinterpret_cast<char*>(&size),4) || size>10000000 || kind>2)throw std::runtime_error("Invalid input record");
      std::vector<uint8_t> bytes(size);if(!input.read(reinterpret_cast<char*>(bytes.data()),size))throw std::runtime_error("Truncated input record");
      ros::serialization::IStream stream(bytes.data(),size);++counts[kind];
      if(kind==0){sensor_msgs::ImuPtr m(new sensor_msgs::Imu);ros::serialization::deserialize(stream,*m);on_imu(m);}
      if(kind==1){sensor_msgs::ImagePtr m(new sensor_msgs::Image);ros::serialization::deserialize(stream,*m);on_image(m);}
      if(kind==2){geometry_msgs::PoseStamped::Ptr m(new geometry_msgs::PoseStamped);ros::serialization::deserialize(stream,*m);on_extr(m);}
    }
    ROS_INFO_STREAM("Replay complete: IMU="<<counts[0]<<" images="<<counts[1]<<" extrinsics="<<counts[2]<<" pending="<<images.size());
  }else{
    auto imu=nh.subscribe<sensor_msgs::Imu>("/slam/input/imu",2000,on_imu,ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
    auto image=nh.subscribe<sensor_msgs::Image>("/active/image",20,on_image);
    auto extr=nh.subscribe<geometry_msgs::PoseStamped>("/active/extrinsic",100,on_extr);
    ros::spin();
  }
}
