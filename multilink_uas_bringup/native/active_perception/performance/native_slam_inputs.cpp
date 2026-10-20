#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <cstring>
#include <optional>
#include <fstream>
#include <filesystem>
#include <vector>

struct InputAudit {
  uint64_t count=0,first=0,last=0,max_gap=0,missing=0,duplicates=0,reordered=0;const uint64_t period;
  std::vector<std::pair<uint64_t,uint64_t>> gaps;
  explicit InputAudit(uint64_t p):period(p){}
  void record(uint64_t t){if(count){if(t==last)++duplicates;if(t<last)++reordered;if(t>last){const auto gap=t-last;max_gap=std::max(max_gap,gap);if(gap>period){missing+=gap/period-1;if(gaps.size()<1000)gaps.emplace_back(last,t);}}}else first=t;last=t;++count;}
  void write(std::ostream& s)const{s<<"{\"count\":"<<count<<",\"first_stamp_ns\":"<<first<<",\"last_stamp_ns\":"<<last<<",\"max_gap_ns\":"<<max_gap<<",\"missing_periods\":"<<missing<<",\"duplicates\":"<<duplicates<<",\"reordered\":"<<reordered<<",\"gaps\":[";for(size_t i=0;i<gaps.size();++i){if(i)s<<',';s<<'['<<gaps[i].first<<','<<gaps[i].second<<']';}s<<"]}";}
};
int main(int argc,char**argv){
  ros::init(argc,argv,"slam_inputs");ros::NodeHandle nh,pnh("~");
  auto imu_pub=nh.advertise<sensor_msgs::Imu>("/slam/input/imu",2000);
  auto vio_pub=nh.advertise<sensor_msgs::Imu>("/vio/initialized_imu",2000);
  auto cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/slam/input/points",5);
  double imu_rate;pnh.param("imu_rate",imu_rate,200.);if(imu_rate<=0)throw std::runtime_error("Invalid IMU rate");
  InputAudit imu_audit(static_cast<uint64_t>(1e9/imu_rate+.5)),cloud_audit(100000000);std::string audit_path;pnh.param<std::string>("audit_path",audit_path,"");
  auto audit=[&](){if(audit_path.empty())return;std::ofstream f(audit_path+".tmp");f<<"{\"imu\":";imu_audit.write(f);f<<",\"lidar\":";cloud_audit.write(f);f<<"}\n";f.close();std::filesystem::rename(audit_path+".tmp",audit_path);};
  auto audit_timer=nh.createWallTimer(ros::WallDuration(1.),[&](const ros::WallTimerEvent&){audit();});
  bool enabled=false,gate_slam=false;pnh.param("gate_slam",gate_slam,false);std::optional<double> first;
  auto gate=nh.subscribe<std_msgs::Bool>("/active/initialize_vio",1,[&](const std_msgs::Bool::ConstPtr&m){if(m->data && !enabled){enabled=true;ROS_INFO("Stationary hover gate opened");}});
  std::string imu_topic,points_topic;pnh.param<std::string>("imu_topic",imu_topic,"/rmf_unipilot/imu");pnh.param<std::string>("points_topic",points_topic,"/rmf_unipilot/lidar/points");
  auto imu=nh.subscribe<sensor_msgs::Imu>(imu_topic,2000,[&](const sensor_msgs::Imu::ConstPtr& m){
    imu_audit.record(m->header.stamp.toNSec());
    const double t=m->header.stamp.toSec();if(!first)first=t;if(t-*first<2.)return;
    auto out=*m;out.header.frame_id="slam_imu";out.orientation.x=out.orientation.y=out.orientation.z=0.;out.orientation.w=1.;
    out.orientation_covariance.fill(0.);out.orientation_covariance[0]=-1.;
    if(!gate_slam || enabled)imu_pub.publish(out);if(enabled)vio_pub.publish(out);
  },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  auto cloud=nh.subscribe<sensor_msgs::PointCloud2>(points_topic,5,[&](const sensor_msgs::PointCloud2::ConstPtr& m){
    cloud_audit.record(m->header.stamp.toNSec());
    if(gate_slam && !enabled)return;
    if(m->is_bigendian){ROS_ERROR_THROTTLE(5,"Big-endian input is unsupported");return;}
    uint32_t offsets[5];const char* names[]={"x","y","z","intensity","ring"};
    for(int j=0;j<5;++j){bool found=false;for(const auto& f:m->fields)if(f.name==names[j]){offsets[j]=f.offset;found=true;}
      if(!found || offsets[j]+(j==4?2:4)>m->point_step)throw std::runtime_error("Invalid point fields");}
    sensor_msgs::PointCloud2 out;out.header=m->header;out.header.frame_id="slam_lidar";out.height=1;out.width=m->width*m->height;
    out.is_bigendian=false;out.point_step=22;out.row_step=22*out.width;out.is_dense=false;out.data.resize(out.row_step,0);
    const char* output_names[]={"x","y","z","t","intensity","tag","line"};
    const uint32_t output_offsets[]={0,4,8,12,16,20,21};const uint8_t types[]={7,7,7,6,7,2,2};
    for(int j=0;j<7;++j){sensor_msgs::PointField f;f.name=output_names[j];f.offset=output_offsets[j];f.datatype=types[j];f.count=1;out.fields.push_back(f);}
    for(uint32_t r=0;r<m->height;++r)for(uint32_t c=0;c<m->width;++c){
      const auto* src=m->data.data()+r*m->row_step+c*m->point_step;auto* dst=out.data.data()+(r*m->width+c)*22;
      for(int j=0;j<4;++j)std::memcpy(dst+(j==3?16:j*4),src+offsets[j],4);
      uint16_t ring;std::memcpy(&ring,src+offsets[4],2);dst[21]=uint8_t(ring);
    }
    cloud_pub.publish(out);
  },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  ros::spin();audit();
}
