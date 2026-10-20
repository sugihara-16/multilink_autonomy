#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <deque>
#include <filesystem>

class ImageEncoder:public rclcpp::Node {
  using Image=sensor_msgs::msg::Image;using Joint=sensor_msgs::msg::JointState;
  struct Angle {int64_t t;Eigen::Vector2d a;};
  bool fixed_;Eigen::Matrix3d home_;Eigen::Vector3d position_;
  cv::Mat map_x_,map_y_;
  int raw_width_,raw_height_,width_,height_;std::deque<Angle> angles_;std::deque<Image::ConstSharedPtr> images_;
  uint64_t paired_=0,dropped_=0;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angle_pub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<Joint>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr report_;
  static int64_t stamp(const std_msgs::msg::Header& h){return int64_t(h.stamp.sec)*1000000000+h.stamp.nanosec;}
  void publish_angle(const Angle& a){std_msgs::msg::Float64MultiArray m;m.data={a.t/1e9,a.a.x(),a.a.y()};angle_pub_->publish(m);}
  void drain() {
    while(!images_.empty() && angles_.size()>1) {
      auto m=images_.front();const auto t=stamp(m->header);
      if(t>angles_.back().t)break;
      images_.pop_front();
      auto it=std::lower_bound(angles_.begin(),angles_.end(),t,[](const Angle& a,int64_t t){return a.t<t;});
      if(it==angles_.begin() && t!=it->t){++dropped_;continue;}
      Eigen::Vector2d a;
      if(it->t==t)a=it->a;
      else {const auto& prev=*std::prev(it);if(it->t-prev.t>20000000){++dropped_;continue;}
        a=prev.a+(it->a-prev.a)*(double(t-prev.t)/double(it->t-prev.t));}
      Eigen::Matrix3d optical;optical<<0,0,1,-1,0,0,0,-1,0;
      const Eigen::Matrix3d r=home_*(Eigen::AngleAxisd(a.x(),Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(a.y(),Eigen::Vector3d::UnitY())).toRotationMatrix()*optical;
      Eigen::Quaterniond q(r);q.normalize();
      geometry_msgs::msg::PoseStamped pose;pose.header=m->header;pose.header.frame_id="fc";
      pose.pose.position.x=position_.x();pose.pose.position.y=position_.y();pose.pose.position.z=position_.z();
      pose.pose.orientation.x=q.x();pose.pose.orientation.y=q.y();pose.pose.orientation.z=q.z();pose.pose.orientation.w=q.w();
      if(m->encoding!="rgb8" && m->encoding!="bgr8")throw std::runtime_error("Expected RGB/BGR input");
      if(int(m->width)!=raw_width_ || int(m->height)!=raw_height_)throw std::runtime_error("Image/calibration size mismatch");
      cv::Mat rgb(m->height,m->width,CV_8UC3,const_cast<uint8_t*>(m->data.data()),m->step),gray;
      cv::cvtColor(rgb,gray,m->encoding=="rgb8"?cv::COLOR_RGB2GRAY:cv::COLOR_BGR2GRAY);
      if(!map_x_.empty()){cv::Mat converted;cv::remap(gray,converted,map_x_,map_y_,cv::INTER_LINEAR);gray=converted;}
      // Preserve the Python adapter's published image frame label.
      Image out;out.header=m->header;out.header.frame_id="fc";out.width=width_;out.height=height_;out.encoding="mono8";out.step=width_;
      out.data.assign(gray.data,gray.data+gray.total());
      pose_pub_->publish(pose);image_pub_->publish(out);++paired_;
      while(angles_.size()>2 && angles_[1].t<=t)angles_.pop_front();
    }
    while(images_.size()>10){images_.pop_front();++dropped_;}
  }
public:
  ImageEncoder(const std::filesystem::path& directory,bool fixed):Node(fixed?"fixed_image_encoder":"active_image_encoder"),fixed_(fixed) {
    // All synchronization uses message timestamps. A 1000 Hz /clock subscriber
    // is unnecessary here; the report timer below intentionally uses wall time.
    auto manifest=YAML::LoadFile((directory/"camera_manifest.json").string());
    if(manifest["expected_joints"].size())throw std::runtime_error("Native encoder currently supports UniPilot only");
    auto camera=manifest["cameras"][0],t=camera["T_FC_camera"];
    for(int i=0;i<3;++i){position_[i]=t[i][3].as<double>();for(int j=0;j<3;++j)home_(i,j)=t[i][j].as<double>();}
    auto raw=YAML::LoadFile((directory/"raw_cam0.yaml").string()),rect=YAML::LoadFile((directory/"cam0.yaml").string());
    width_=rect["image_width"].as<int>();height_=rect["image_height"].as<int>();
    raw_width_=raw["image_width"].as<int>();raw_height_=raw["image_height"].as<int>();
    if(raw["camera_matrix"]["data"].as<std::vector<double>>()!=rect["camera_matrix"]["data"].as<std::vector<double>>() ||
       raw["image_width"].as<int>()!=width_ || raw["image_height"].as<int>()!=height_){
      cv::Mat raw_k(3,3,CV_64F),rect_k(3,3,CV_64F);
      for(int i=0;i<9;++i){raw_k.at<double>(i/3,i%3)=raw["camera_matrix"]["data"][i].as<double>();rect_k.at<double>(i/3,i%3)=rect["camera_matrix"]["data"][i].as<double>();}
      cv::initUndistortRectifyMap(raw_k,cv::Mat::zeros(1,5,CV_64F),cv::Mat::eye(3,3,CV_64F),rect_k,cv::Size(width_,height_),CV_32FC1,map_x_,map_y_);
    }
    const std::string prefix=fixed?"/fixed":"";
    image_pub_=create_publisher<Image>(prefix+"/active/image",10);
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>(prefix+"/active/extrinsic",10);
    angle_pub_=create_publisher<std_msgs::msg::Float64MultiArray>(prefix+"/active/encoder",2000);
    if(!fixed)joint_sub_=create_subscription<Joint>("/active/gimbal/states",rclcpp::QoS(2000).reliable(),[this](Joint::ConstSharedPtr m){
      int yaw=-1,pitch=-1;
      for(size_t i=0;i<m->name.size();++i){if(m->name[i]=="active_camera_yaw_joint")yaw=i;if(m->name[i]=="active_camera_pitch_joint")pitch=i;}
      if(yaw<0 || pitch<0 || m->position.size()<=size_t(std::max(yaw,pitch)))return;
      const auto t=stamp(m->header);if(!angles_.empty() && t<=angles_.back().t)return;
      angles_.push_back({t,{m->position[yaw],m->position[pitch]}});
      while(angles_.size()>10000)angles_.pop_front();publish_angle(angles_.back());drain();
    });
    image_sub_=create_subscription<Image>(camera["topic"].as<std::string>(),rclcpp::QoS(30).reliable(),[this](Image::ConstSharedPtr m){
      if(fixed_){Angle a{stamp(m->header),Eigen::Vector2d::Zero()};angles_.push_back(a);if(angles_.size()==1)angles_.push_back(a);publish_angle(a);}
      images_.push_back(m);drain();
    });
    report_=create_wall_timer(std::chrono::seconds(10),[this](){RCLCPP_INFO(get_logger(),"paired=%lu dropped=%lu pending=%zu",paired_,dropped_,images_.size());});
  }
};
int main(int argc,char**argv){
  std::string calibration;bool fixed=false;
  for(int i=1;i<argc;++i){if(std::string(argv[i])=="--calibration" && i+1<argc)calibration=argv[++i];else if(std::string(argv[i])=="--fixed")fixed=true;}
  if(calibration.empty())throw std::runtime_error("--calibration is required");
  cv::setNumThreads(1);rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ImageEncoder>(calibration,fixed));rclcpp::shutdown();
}
