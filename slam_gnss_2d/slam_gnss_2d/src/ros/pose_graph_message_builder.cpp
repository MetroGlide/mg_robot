#include "slam_gnss_2d/ros/pose_graph_message_builder.hpp"

#include <cmath>
#include <set>
#include <utility>

namespace slam_gnss_2d {
namespace ros {

std::pair<std::vector<core::PoseEdge>, std::vector<core::PoseEdge>> split_edges(
    const std::vector<core::PoseEdge>& all_edges,
    const std::vector<core::PoseEdge>& loop_edges) {
  std::set<std::pair<int, int>> loop_set;
  for (const auto& e : loop_edges) {
    loop_set.insert({e.from_index, e.to_index});
  }

  std::vector<core::PoseEdge> seq_edges;
  for (const auto& e : all_edges) {
    if (loop_set.find({e.from_index, e.to_index}) == loop_set.end()) {
      seq_edges.push_back(e);
    }
  }

  return {seq_edges, loop_edges};
}

slam_gnss_2d_msgs::msg::PoseGraphDiff build_pose_graph_diff(
    const std::vector<core::PoseNode>& nodes,
    const std::vector<core::PoseEdge>& seq_edges,
    const std::vector<core::PoseEdge>& loop_edges,
    const std::vector<core::GnssPrior>& priors,
    bool loop_closed,
    bool full_refresh_needed) {
  slam_gnss_2d_msgs::msg::PoseGraphDiff msg;
  msg.loop_closed = loop_closed;
  msg.full_refresh_needed = full_refresh_needed;

  for (const auto& node : nodes) {
    msg.new_node_indices.push_back(node.index);
    msg.new_node_x.push_back(node.x);
    msg.new_node_y.push_back(node.y);
    msg.new_node_yaw.push_back(node.yaw);
    msg.new_node_timestamps.push_back(node.timestamp);
  }

  for (const auto& edge : seq_edges) {
    msg.seq_edge_from.push_back(edge.from_index);
    msg.seq_edge_to.push_back(edge.to_index);
    msg.seq_edge_score.push_back(edge.score);
    msg.seq_edge_type.push_back(edge.is_odom_fallback ? 1 : 0);
    msg.seq_edge_info_diag.push_back(edge.information(0, 0));
    msg.seq_edge_info_diag.push_back(edge.information(1, 1));
    msg.seq_edge_info_diag.push_back(edge.information(2, 2));
  }

  for (const auto& prior : priors) {
    msg.prior_node_indices.push_back(prior.node_index);
    double sigma = -1.0;
    if (prior.information(0, 0) > 0.0) {
      sigma = 1.0 / std::sqrt(prior.information(0, 0));
    }
    msg.prior_sigma_m.push_back(sigma);
    msg.prior_gnss_status.push_back(0);
  }

  for (const auto& edge : loop_edges) {
    msg.loop_edge_from.push_back(edge.from_index);
    msg.loop_edge_to.push_back(edge.to_index);
    msg.loop_edge_score.push_back(edge.score);
    msg.loop_edge_info_diag.push_back(edge.information(0, 0));
    msg.loop_edge_info_diag.push_back(edge.information(1, 1));
    msg.loop_edge_info_diag.push_back(edge.information(2, 2));
  }

  return msg;
}

}  // namespace ros
}  // namespace slam_gnss_2d
