// UniPilot experiment bridge: use upstream message conversions, but request
// reliable delivery on BOTH ROS2 publishers and ROS2 subscriptions.
// ros1_bridge 0.10.3 parameter_bridge applies its YAML QoS only to publishers.
#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <ros1_bridge/bridge.hpp>
#include <vector>
#include <stdexcept>

int main(int argc, char ** argv)
{
  // The installed upstream Factory::ros1_callback explicitly filters callerid
  // "/ros_bridge". Another ROS1 name echoes incoming sensors back into ROS2.
  ros::init(argc, argv, "ros_bridge");
  if (ros::this_node::getName() != "/ros_bridge")
    throw std::runtime_error("Upstream loop prevention requires ROS1 name /ros_bridge");
  rclcpp::init(argc, argv);
  ros::NodeHandle ros1;
  auto ros2 = std::make_shared<rclcpp::Node>("uas_reliable_bridge");
  std::vector<ros1_bridge::BridgeHandles> bridges;
  try {
    XmlRpc::XmlRpcValue topics;
    if (!ros1.getParam("topics", topics) || topics.getType() != XmlRpc::XmlRpcValue::TypeArray)
      throw std::runtime_error("Expected topics parameter array");
    for (const auto & name : {"services_1_to_2", "services_2_to_1"}) {
      XmlRpc::XmlRpcValue services;
      if (ros1.getParam(name, services) && services.size() != 0)
        throw std::runtime_error("Experiment bridge does not implement service bridging");
    }
    for (int i = 0; i < topics.size(); ++i) {
      const std::string topic = topics[i]["topic"];
      const std::string type2 = topics[i]["type"];
      const int depth = topics[i]["queue_size"];
      if (depth <= 0) throw std::runtime_error("Invalid queue depth");
      std::string type1;
      if (!ros1_bridge::get_2to1_mapping(type2, type1))
        throw std::runtime_error("No upstream conversion for " + type2);
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
      ros1_bridge::BridgeHandles h;
      h.bridge1to2 = ros1_bridge::create_bridge_from_1_to_2(
        ros1, ros2, type1, topic, depth, type2, topic, qos);
      h.bridge2to1 = ros1_bridge::create_bridge_from_2_to_1(
        ros2, ros1, type2, topic, qos, type1, topic, depth,
        h.bridge1to2.ros2_publisher);  // Upstream loop prevention.
      bridges.push_back(std::move(h));
      RCLCPP_INFO(ros2->get_logger(), "%s: reliable publish AND subscribe, depth=%d",
                  topic.c_str(), depth);
    }
    ros::AsyncSpinner spinner(1);
    spinner.start();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(ros2);
    while (ros::ok() && rclcpp::ok()) {
      executor.spin_once(std::chrono::milliseconds(10));
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(ros2->get_logger(), "%s", e.what());
    ros::shutdown();
    rclcpp::shutdown();
    return 1;
  }
  ros::shutdown();
  rclcpp::shutdown();
  return 0;
}
