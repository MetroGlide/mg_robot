#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/gnss/utm.hpp"
#include "slam_gnss_2d/input/base.hpp"

namespace slam_gnss_2d {
namespace input {

class ROS2ScanSource : public ScanSourceBase {
 public:
  ROS2ScanSource(rclcpp::Node::SharedPtr node, const std::string& topic);

  void set_scan_callback(
      std::function<void(const core::ScanDataPtr&)> callback) override;
  void start() override;
  void stop() override;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  std::function<void(const core::ScanDataPtr&)> callback_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double lidar_yaw_{0.0};
  double lidar_x_{0.0};
  double lidar_y_{0.0};
  bool lidar_tf_ready_{false};
  int recv_count_{0};

  void on_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

class ROS2OdomSource : public OdomSourceBase {
 public:
  ROS2OdomSource(rclcpp::Node::SharedPtr node, const std::string& topic);

  void start() override;
  void stop() override;
  std::optional<core::OdomData> get_odom_at(double timestamp) override;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  std::deque<core::OdomData> buffer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  int recv_count_{0};
  bool empty_warned_{false};

  void on_msg(const nav_msgs::msg::Odometry::SharedPtr msg);
};

class ROS2GnssUtmSource : public GnssSourceBase {
 public:
  ROS2GnssUtmSource(rclcpp::Node::SharedPtr node, const std::string& topic);

  void start() override;
  void stop() override;
  std::optional<core::GnssData> get_gnss_at(double timestamp) override;
  std::vector<core::GnssData> get_all_gnss() override;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  std::deque<core::GnssData> buffer_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_;
  std::optional<gnss::UtmTransformer> transformer_;
  int recv_count_{0};

  void on_msg(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

class ROS2NavpvtSource : public GnssSourceBase {
 public:
  ROS2NavpvtSource(
      rclcpp::Node::SharedPtr node,
      const std::string& topic,
      double hacc_scale = 1.0);

  void start() override;
  void stop() override;
  std::optional<core::GnssData> get_gnss_at(double timestamp) override;
  std::vector<core::GnssData> get_all_gnss() override;

 private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  double hacc_scale_;
  std::deque<core::GnssData> buffer_;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr sub_;
  std::optional<gnss::UtmTransformer> transformer_;
  int recv_count_{0};

  void on_msg(const ublox_msgs::msg::NavPVT::SharedPtr msg);
};

}  // namespace input
}  // namespace slam_gnss_2d
