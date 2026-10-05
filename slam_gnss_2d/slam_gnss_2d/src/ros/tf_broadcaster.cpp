#include "slam_gnss_2d/ros/tf_broadcaster.hpp"

#include <cmath>

namespace slam_gnss_2d {
namespace ros {

SlamTfBroadcaster::SlamTfBroadcaster(rclcpp::Node::SharedPtr node)
    : node_(node),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node)) {}

void SlamTfBroadcaster::update(
    const core::PoseNode& node_pose, const core::OdomData& odom) {
  double c_o = std::cos(odom.yaw);
  double s_o = std::sin(odom.yaw);
  double inv_x = -(c_o * odom.x + s_o * odom.y);
  double inv_y = -(-s_o * odom.x + c_o * odom.y);

  double c_m = std::cos(node_pose.yaw);
  double s_m = std::sin(node_pose.yaw);
  map_to_odom_x_ = node_pose.x + c_m * inv_x - s_m * inv_y;
  map_to_odom_y_ = node_pose.y + s_m * inv_x + c_m * inv_y;
  map_to_odom_yaw_ = node_pose.yaw - odom.yaw;
}

void SlamTfBroadcaster::publish() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.translation.x = map_to_odom_x_;
  t.transform.translation.y = map_to_odom_y_;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = std::cos(map_to_odom_yaw_ / 2.0);
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = std::sin(map_to_odom_yaw_ / 2.0);
  tf_broadcaster_->sendTransform(t);
}

}  // namespace ros
}  // namespace slam_gnss_2d
