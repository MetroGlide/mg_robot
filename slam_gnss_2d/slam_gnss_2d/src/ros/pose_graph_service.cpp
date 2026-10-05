#include "slam_gnss_2d/ros/pose_graph_service.hpp"

#include "slam_gnss_2d/ros/pose_graph_message_builder.hpp"

namespace slam_gnss_2d {
namespace ros {

PoseGraphService::PoseGraphService(
    rclcpp::Node::SharedPtr node,
    pose_graph::PoseGraphBuilderPtr pose_graph,
    core::GraphOrchestratorPtr orchestrator)
    : node_(node), pose_graph_(pose_graph), orchestrator_(orchestrator) {
  srv_ = node_->create_service<slam_gnss_2d_msgs::srv::GetPoseGraph>(
      "slam_gnss_2d/get_pose_graph",
      [this](
          const std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Request> request,
          std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Response> response) {
        this->handle_service(request, response);
      });
}

void PoseGraphService::handle_service(
    [[maybe_unused]] const std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Request> request,
    std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Response> response) {
  auto nodes = pose_graph_->get_nodes();
  auto all_edges = pose_graph_->get_edges();

  std::vector<core::PoseEdge> loop_edges;
  auto loop_builder = std::dynamic_pointer_cast<pose_graph::LoopClosureBuilder>(pose_graph_);
  if (loop_builder) {
    loop_edges = loop_builder->get_loop_edges();
  }

  auto [seq_edges, split_loops] = split_edges(all_edges, loop_edges);
  auto diff = build_pose_graph_diff(
      nodes, seq_edges, split_loops, {}, false, true);

  response->graph = diff;
  response->total_nodes = static_cast<int>(nodes.size());
  response->total_seq_edges = static_cast<int>(seq_edges.size());
  response->total_loop_edges = static_cast<int>(split_loops.size());

  auto sm_builder = std::dynamic_pointer_cast<pose_graph::ScanMatchingBuilder>(pose_graph_);
  if (loop_builder) {
    response->icp_attempt_count = loop_builder->icp_attempt_count();
    response->icp_success_count = loop_builder->icp_success_count();
    response->odom_fallback_count = loop_builder->odom_fallback_count();
    response->loop_attempt_count = loop_builder->loop_attempt_count();
    response->loop_success_count = loop_builder->loop_success_count();
  } else if (sm_builder) {
    response->icp_attempt_count = sm_builder->icp_attempt_count();
    response->icp_success_count = sm_builder->icp_success_count();
    response->odom_fallback_count = sm_builder->odom_fallback_count();
    response->loop_attempt_count = 0;
    response->loop_success_count = 0;
  } else {
    response->icp_attempt_count = 0;
    response->icp_success_count = 0;
    response->odom_fallback_count = 0;
    response->loop_attempt_count = 0;
    response->loop_success_count = 0;
  }
}

}  // namespace ros
}  // namespace slam_gnss_2d
