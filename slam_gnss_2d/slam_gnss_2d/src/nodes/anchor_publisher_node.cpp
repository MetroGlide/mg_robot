#include <fstream>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace slam_gnss_2d {

class AnchorPublisherNode : public rclcpp::Node {
 public:
  AnchorPublisherNode() : rclcpp::Node("anchor_publisher_node") {
    declare_parameter("gnss_transform_file", "");
    declare_parameter("map_frame_id", "map");

    std::string transform_file = get_parameter("gnss_transform_file").as_string();
    std::string map_frame_id = get_parameter("map_frame_id").as_string();

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();
    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/slam_gnss_2d/anchor", map_qos);

    if (transform_file.empty()) {
      RCLCPP_ERROR(get_logger(), "gnss_transform_file parameter is empty.");
      return;
    }

    try {
      YAML::Node data = YAML::LoadFile(transform_file);
      double lat = data["anchor"]["latitude"].as<double>();
      double lon = data["anchor"]["longitude"].as<double>();

      sensor_msgs::msg::NavSatFix msg;
      msg.header.stamp = now();
      msg.header.frame_id = map_frame_id;
      msg.latitude = lat;
      msg.longitude = lon;
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

      pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "Published anchor: lat=%.7f, lon=%.7f from %s",
                  lat, lon, transform_file.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to read transform file %s: %s",
                   transform_file.c_str(), e.what());
    }
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
};

}  // namespace slam_gnss_2d

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam_gnss_2d::AnchorPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
