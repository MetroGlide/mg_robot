#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <slam_gnss_2d_msgs/msg/pose_graph_diff.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/core/graph_orchestrator.hpp"
#include "slam_gnss_2d/map_manager/base.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"

namespace slam_gnss_2d {
namespace ros {

class SlamVisualizer {
 public:
  SlamVisualizer(rclcpp::Node::SharedPtr node, bool use_gnss);

  void publish_anchor(core::GraphOrchestrator& orchestrator);
  void publish_map(map_manager::MapRendererBase& renderer);
  void publish_path_increment(const core::PoseNode& node);
  void rebuild_path(const std::vector<core::PoseNode>& nodes);
  void publish_path_before_optimize();
  void publish_pose_graph_markers(const pose_graph::PoseGraphBuilderBase& pose_graph);
  void publish_pose_graph_diff(
      const std::vector<core::PoseNode>& new_nodes,
      const std::vector<core::PoseEdge>& new_seq_edges,
      const std::vector<core::GnssPrior>& new_priors,
      const std::vector<core::PoseEdge>& new_loop_edges,
      bool loop_closed,
      bool full_refresh_needed);

 private:
  rclcpp::Node::SharedPtr node_;
  bool use_gnss_;
  std::optional<std::pair<double, double>> last_anchor_latlon_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pg_marker_pub_;
  rclcpp::Publisher<slam_gnss_2d_msgs::msg::PoseGraphDiff>::SharedPtr diff_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_before_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr anchor_pub_;

  nav_msgs::msg::Path path_msg_;
};

using SlamVisualizerPtr = std::shared_ptr<SlamVisualizer>;

}  // namespace ros
}  // namespace slam_gnss_2d
