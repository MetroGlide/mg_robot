#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <slam_gnss_2d_msgs/srv/get_pose_graph.hpp>

#include "slam_gnss_2d/core/graph_orchestrator.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"
#include "slam_gnss_2d/pose_graph/loop_closure_builder.hpp"
#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"

namespace slam_gnss_2d {
namespace ros {

class PoseGraphService {
 public:
  PoseGraphService(
      rclcpp::Node::SharedPtr node,
      pose_graph::PoseGraphBuilderPtr pose_graph,
      core::GraphOrchestratorPtr orchestrator);

 private:
  rclcpp::Node::SharedPtr node_;
  pose_graph::PoseGraphBuilderPtr pose_graph_;
  core::GraphOrchestratorPtr orchestrator_;
  rclcpp::Service<slam_gnss_2d_msgs::srv::GetPoseGraph>::SharedPtr srv_;

  void handle_service(
      const std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Request> request,
      std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Response> response);
};

using PoseGraphServicePtr = std::shared_ptr<PoseGraphService>;

}  // namespace ros
}  // namespace slam_gnss_2d
