#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;

  void handle_save(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

using MapSaveServicePtr = std::shared_ptr<MapSaveService>;

}  // namespace ros
}  // namespace slam_gnss_2d
