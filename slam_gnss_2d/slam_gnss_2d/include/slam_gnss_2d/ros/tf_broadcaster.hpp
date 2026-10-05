#pragma once

#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace ros {

class SlamTfBroadcaster {
 public:
  explicit SlamTfBroadcaster(rclcpp::Node::SharedPtr node);

  void update(const core::PoseNode& node_pose, const core::OdomData& odom);
  void publish();

 private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double map_to_odom_x_{0.0};
  double map_to_odom_y_{0.0};
  double map_to_odom_yaw_{0.0};
};

using SlamTfBroadcasterPtr = std::shared_ptr<SlamTfBroadcaster>;

}  // namespace ros
}  // namespace slam_gnss_2d
