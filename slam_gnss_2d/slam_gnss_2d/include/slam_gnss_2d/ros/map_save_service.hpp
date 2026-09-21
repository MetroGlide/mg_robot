#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <slam_gnss_2d_msgs/srv/save_slam_map.hpp>

#include "slam_gnss_2d/core/graph_orchestrator.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"

namespace slam_gnss_2d {
namespace ros {

class MapSaveService {
 public:
  MapSaveService(
      rclcpp::Node::SharedPtr node,
      pose_graph::PoseGraphBuilderPtr pose_graph,
      core::GraphOrchestratorPtr orchestrator);

 private:
  rclcpp::Node::SharedPtr node_;
  pose_graph::PoseGraphBuilderPtr pose_graph_;
  core::GraphOrchestratorPtr orchestrator_;
  rclcpp::Service<slam_gnss_2d_msgs::srv::SaveSlamMap>::SharedPtr save_srv_;

  void handle_save(
      const std::shared_ptr<slam_gnss_2d_msgs::srv::SaveSlamMap::Request> request,
      std::shared_ptr<slam_gnss_2d_msgs::srv::SaveSlamMap::Response> response);
};

using MapSaveServicePtr = std::shared_ptr<MapSaveService>;

}  // namespace ros
}  // namespace slam_gnss_2d
