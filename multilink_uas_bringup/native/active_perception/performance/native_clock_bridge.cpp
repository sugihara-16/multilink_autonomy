// Simulation-time relay: at 1000 Hz it removes only repeated paused timestamps.
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <mutex>
#include <limits>
int main(int argc,char**argv) {
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("native_clock_bridge");
 int hz=1000;for(int i=1;i+1<argc;++i)if(std::string(argv[i])=="--hz")hz=std::stoi(argv[++i]);
 if(hz<=0 || 1000000000%hz)return 2;const int64_t period=1000000000/hz;
 auto publisher=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::QoS(10));
 std::mutex mutex;int64_t last=-1,last_bucket=-1;gz::transport::Node transport;
 std::function<void(const gz::msgs::Clock&)> callback=[&](const gz::msgs::Clock& input){
  std::lock_guard<std::mutex> guard(mutex);
  const int64_t now=input.sim().sec()*1000000000LL+input.sim().nsec(),bucket=now/period;
  if(now==last)return;
  const bool reset=last>=0 && now<last;last=now;
  if(!reset && bucket==last_bucket)return;
  last_bucket=bucket;rosgraph_msgs::msg::Clock output;
  output.clock.sec=input.sim().sec();output.clock.nanosec=input.sim().nsec();publisher->publish(output);
 };
 if(!transport.Subscribe<gz::msgs::Clock>("/clock",callback))return 3;
 RCLCPP_INFO(node->get_logger(),"Clock relay cap %d Hz; physics and sensor timestamps unchanged",hz);
 rclcpp::spin(node);rclcpp::shutdown();return 0;
}
