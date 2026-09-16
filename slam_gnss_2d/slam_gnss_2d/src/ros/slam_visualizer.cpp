#include "slam_gnss_2d/ros/slam_visualizer.hpp"

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <unordered_map>

#include "slam_gnss_2d/ros/pose_graph_message_builder.hpp"

namespace slam_gnss_2d {
namespace ros {

SlamVisualizer::SlamVisualizer(rclcpp::Node::SharedPtr node, bool use_gnss)
    : node_(node), use_gnss_(use_gnss) {
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();

  map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("slam_gnss_2d/path", 1);
  pg_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "slam_gnss_2d/pose_graph", 1);
  diff_pub_ = node_->create_publisher<slam_gnss_2d_msgs::msg::PoseGraphDiff>(
      "slam_gnss_2d/pose_graph_diff", 1);
  path_before_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "slam_gnss_2d/path_before_optimize", 1);
  anchor_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "slam_gnss_2d/anchor", map_qos);

  path_msg_.header.frame_id = "map";
}

void SlamVisualizer::publish_anchor(core::GraphOrchestrator& orchestrator) {
  if (use_gnss_ && !anchor_published_) {
    auto latlon = orchestrator.anchor_latlon();
    if (latlon.has_value()) {
      auto [lat, lon] = *latlon;
      sensor_msgs::msg::NavSatFix msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = "map";
      msg.latitude = lat;
      msg.longitude = lon;
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      anchor_pub_->publish(msg);
      anchor_published_ = true;
      RCLCPP_INFO(node_->get_logger(), "Anchor published: Lat=%.7f, Lon=%.7f", lat, lon);
    }
  }
}

void SlamVisualizer::publish_map(map_manager::MapRendererBase& renderer) {
  auto occ_data = renderer.to_occupancy_array();
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.header.frame_id = "map";
  msg.info.resolution = occ_data.resolution;
  msg.info.width = occ_data.data.cols;
  msg.info.height = occ_data.data.rows;
  msg.info.origin.position.x = occ_data.origin_x;
  msg.info.origin.position.y = occ_data.origin_y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(occ_data.data.rows * occ_data.data.cols);
  for (int r = 0; r < occ_data.data.rows; ++r) {
    const int8_t* row = occ_data.data.ptr<int8_t>(r);
    for (int c = 0; c < occ_data.data.cols; ++c) {
      msg.data[r * occ_data.data.cols + c] = row[c];
    }
  }
  map_pub_->publish(msg);
}

void SlamVisualizer::publish_path_increment(const core::PoseNode& node) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_->get_clock()->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = node.x;
  pose.pose.position.y = node.y;
  pose.pose.orientation.w = std::cos(node.yaw / 2.0);
  pose.pose.orientation.z = std::sin(node.yaw / 2.0);

  path_msg_.header.stamp = pose.header.stamp;
  path_msg_.poses.push_back(pose);
  path_pub_->publish(path_msg_);
}

void SlamVisualizer::rebuild_path(const std::vector<core::PoseNode>& nodes) {
  auto now = node_->get_clock()->now();
  path_msg_.header.stamp = now;
  path_msg_.poses.clear();
  path_msg_.poses.reserve(nodes.size());

  for (const auto& n : nodes) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = n.x;
    pose.pose.position.y = n.y;
    pose.pose.orientation.w = std::cos(n.yaw / 2.0);
    pose.pose.orientation.z = std::sin(n.yaw / 2.0);
    path_msg_.poses.push_back(pose);
  }
  path_pub_->publish(path_msg_);
}

void SlamVisualizer::publish_path_before_optimize() {
  path_before_pub_->publish(path_msg_);
}

void SlamVisualizer::publish_pose_graph_markers(
    const pose_graph::PoseGraphBuilderBase& pose_graph) {
  auto nodes = pose_graph.get_nodes();
  if (nodes.empty()) {
    return;
  }

  auto all_edges = pose_graph.get_edges();
  std::unordered_map<int, core::PoseNode> node_map;
  for (const auto& n : nodes) {
    node_map[n.index] = n;
  }

  auto now = node_->get_clock()->now();
  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker node_m;
  node_m.header.stamp = now;
  node_m.header.frame_id = "map";
  node_m.ns = "nodes";
  node_m.id = 0;
  node_m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  node_m.action = visualization_msgs::msg::Marker::ADD;
  node_m.scale.x = node_m.scale.y = node_m.scale.z = 0.2;
  node_m.color.r = node_m.color.g = node_m.color.b = node_m.color.a = 1.0;

  int latest_idx = nodes.back().index;
  for (const auto& n : nodes) {
    geometry_msgs::msg::Point pt;
    pt.x = n.x;
    pt.y = n.y;
    pt.z = 0.0;
    node_m.points.push_back(pt);

    std_msgs::msg::ColorRGBA c;
    if (n.index == latest_idx) {
      c.r = 0.0; c.g = 1.0; c.b = 1.0; c.a = 1.0;
    } else {
      c.r = 1.0; c.g = 1.0; c.b = 1.0; c.a = 0.8;
    }
    node_m.colors.push_back(c);
  }
  array.markers.push_back(node_m);

  visualization_msgs::msg::Marker seq_m;
  seq_m.header.stamp = now;
  seq_m.header.frame_id = "map";
  seq_m.ns = "seq_edges";
  seq_m.id = 1;
  seq_m.type = visualization_msgs::msg::Marker::LINE_LIST;
  seq_m.action = visualization_msgs::msg::Marker::ADD;
  seq_m.scale.x = 0.05;
  seq_m.color.r = 0.2; seq_m.color.g = 0.5; seq_m.color.b = 1.0; seq_m.color.a = 0.9;

  visualization_msgs::msg::Marker loop_m;
  loop_m.header.stamp = now;
  loop_m.header.frame_id = "map";
  loop_m.ns = "loop_edges";
  loop_m.id = 2;
  loop_m.type = visualization_msgs::msg::Marker::LINE_LIST;
  loop_m.action = visualization_msgs::msg::Marker::ADD;
  loop_m.scale.x = 0.08;
  loop_m.color.r = 0.0; loop_m.color.g = 1.0; loop_m.color.b = 0.4; loop_m.color.a = 1.0;

  for (const auto& edge : all_edges) {
    auto it0 = node_map.find(edge.from_index);
    auto it1 = node_map.find(edge.to_index);
    if (it0 == node_map.end() || it1 == node_map.end()) {
      continue;
    }
    bool is_loop = std::abs(edge.to_index - edge.from_index) > 1;
    auto& target = is_loop ? loop_m : seq_m;

    geometry_msgs::msg::Point p0, p1;
    p0.x = it0->second.x; p0.y = it0->second.y; p0.z = 0.0;
    p1.x = it1->second.x; p1.y = it1->second.y; p1.z = 0.0;
    target.points.push_back(p0);
    target.points.push_back(p1);
  }

  array.markers.push_back(seq_m);
  array.markers.push_back(loop_m);
  pg_marker_pub_->publish(array);
}

void SlamVisualizer::publish_pose_graph_diff(
    const std::vector<core::PoseNode>& new_nodes,
    const std::vector<core::PoseEdge>& new_seq_edges,
    const std::vector<core::GnssPrior>& new_priors,
    const std::vector<core::PoseEdge>& new_loop_edges,
    bool loop_closed,
    bool full_refresh_needed) {
  auto msg = build_pose_graph_diff(
      new_nodes,
      new_seq_edges,
      new_loop_edges,
      new_priors,
      loop_closed,
      full_refresh_needed);
  diff_pub_->publish(msg);
}

}  // namespace ros
}  // namespace slam_gnss_2d
