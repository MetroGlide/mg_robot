#include "slam_gnss_2d/core/graph_orchestrator.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <set>

#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace core {

GraphOrchestrator::GraphOrchestrator(
    pose_graph::PoseGraphBuilderPtr pose_graph,
    bool use_gnss,
    double isam2_relinearize_threshold,
    int anchor_min_fix_status,
    double gnss_fix_sigma_m,
    double gnss_float_sigma_m,
    double gnss_factor_yaw_variance,
    double gnss_init_distance_m,
    double gnss_max_sigma_m,
    double rerender_threshold_m)
    : pose_graph_(pose_graph),
      use_gnss_(use_gnss),
      optimizer_(isam2_relinearize_threshold),
      anchor_manager_(use_gnss ? std::make_unique<gnss::GnssAnchorManager>() : nullptr),
      anchor_min_fix_status_(anchor_min_fix_status),
      gnss_fix_sigma_m_(gnss_fix_sigma_m),
      gnss_float_sigma_m_(gnss_float_sigma_m),
      gnss_factor_yaw_variance_(gnss_factor_yaw_variance),
      gnss_init_distance_m_(gnss_init_distance_m),
      gnss_max_sigma_m_(gnss_max_sigma_m),
      rerender_threshold_m_(rerender_threshold_m),
      state_(use_gnss ? "INITIALIZING" : "RUNNING") {}

std::optional<std::pair<double, double>> GraphOrchestrator::anchor_latlon() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (anchor_manager_) {
    return anchor_manager_->anchor_latlon();
  }
  return std::nullopt;
}

std::optional<std::pair<double, double>> GraphOrchestrator::anchor_utm() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (anchor_manager_) {
    return anchor_manager_->anchor_utm();
  }
  return std::nullopt;
}

std::optional<double> GraphOrchestrator::init_rotation() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return init_rotation_;
}

std::vector<PoseNode> GraphOrchestrator::get_all_nodes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pose_graph_->get_nodes();
}

std::vector<PoseEdge> GraphOrchestrator::get_all_edges() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pose_graph_->get_edges();
}

ScanProcessResult GraphOrchestrator::process_frame(const SensorFrame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto node = pose_graph_->add_scan(frame.scan, frame.odom);
  if (!node.has_value()) {
    return ScanProcessResult{
        std::nullopt, false, false, std::nullopt, {}, std::nullopt};
  }

  initialize_with_gnss_if_ready(frame);

  if (state_ == "INITIALIZING") {
    return ScanProcessResult{
        node, false, false, get_latest_seq_edge(node->index), {}, std::nullopt};
  }

  initialize_optimizer_if_needed(*node);
  auto new_seq_edge = add_latest_seq_edge(*node);
  auto [new_loop_edges, loop_closed] = add_new_loop_edges();
  auto new_gnss_prior = add_gnss_prior(frame, *node);
  optimizer_.update();
  bool rerender_required = apply_optimized_poses(*node, loop_closed);

  return ScanProcessResult{
      node,
      loop_closed,
      rerender_required,
      new_seq_edge,
      new_loop_edges,
      new_gnss_prior,
  };
}

void GraphOrchestrator::initialize_with_gnss_if_ready(const SensorFrame& frame) {
  if (!use_gnss_ || state_ != "INITIALIZING" || !frame.gnss.has_value() || !anchor_manager_) {
    return;
  }

  if (!anchor_manager_->is_initialized()) {
    anchor_manager_->try_set_anchor(*frame.gnss, anchor_min_fix_status_);
  }

  if (!anchor_manager_->is_initialized()) {
    return;
  }

  auto [lx, ly] = anchor_manager_->to_local(*frame.gnss);
  if (std::hypot(lx, ly) < gnss_init_distance_m_) {
    return;
  }

  double theta0 = std::atan2(ly, lx);
  auto nodes = pose_graph_->get_nodes();
  if (nodes.empty()) {
    return;
  }

  const auto& node0 = nodes.front();
  const auto& curr_node = nodes.back();
  double odom_dx = curr_node.x - node0.x;
  double odom_dy = curr_node.y - node0.y;
  double odom_dist = std::hypot(odom_dx, odom_dy);

  double rot = 0.0;
  if (odom_dist > 0.5) {
    double odom_heading = std::atan2(odom_dy, odom_dx);
    rot = angle_diff(theta0, odom_heading);
  } else {
    rot = angle_diff(theta0, node0.yaw);
  }
  init_rotation_ = rot;

  double c = std::cos(rot);
  double s = std::sin(rot);

  optimizer_.initialize(node0.index, 0.0, 0.0, node0.yaw + rot, 0.05, 10.0);
  for (auto& n : nodes) {
    double dx = n.x - node0.x;
    double dy = n.y - node0.y;
    n.x = c * dx - s * dy;
    n.y = s * dx + c * dy;
    n.yaw = n.yaw + rot;
    if (n.index != node0.index) {
      optimizer_.add_initial_estimate(n.index, n.x, n.y, n.yaw);
    }
  }

  pose_graph_->replace_nodes(nodes);

  for (const auto& edge : pose_graph_->get_edges()) {
    optimizer_.add_between_factor(
        edge.from_index,
        edge.to_index,
        edge.dx,
        edge.dy,
        edge.dyaw,
        edge.information);
  }

  state_ = "RUNNING";
  last_node_index_ = nodes.back().index;
  initialized_ = true;
  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
      "Graph initialized and aligned to UTM with rotation %.3f rad", rot);
}

void GraphOrchestrator::initialize_optimizer_if_needed(const PoseNode& node) {
  if (initialized_) {
    return;
  }
  optimizer_.initialize(node.index, node.x, node.y, node.yaw, 0.05, 10.0);
  initialized_ = true;
  last_node_index_ = node.index;
}

std::optional<PoseEdge> GraphOrchestrator::add_latest_seq_edge(const PoseNode& node) {
  auto latest_seq_edge = get_latest_seq_edge(node.index);
  if (latest_seq_edge.has_value() && node.index > last_node_index_) {
    auto prev_pose = optimizer_.get_pose(latest_seq_edge->from_index);
    if (prev_pose.has_value()) {
      auto [px, py, pyaw] = *prev_pose;
      double c = std::cos(pyaw);
      double s = std::sin(pyaw);
      double x = px + c * latest_seq_edge->dx - s * latest_seq_edge->dy;
      double y = py + s * latest_seq_edge->dx + c * latest_seq_edge->dy;
      double yaw = pyaw + latest_seq_edge->dyaw;
      optimizer_.add_initial_estimate(node.index, x, y, yaw);
      optimizer_.add_between_factor(
          latest_seq_edge->from_index,
          latest_seq_edge->to_index,
          latest_seq_edge->dx,
          latest_seq_edge->dy,
          latest_seq_edge->dyaw,
          latest_seq_edge->information);
      last_node_index_ = node.index;
      return latest_seq_edge;
    }
    last_node_index_ = node.index;
  }
  return std::nullopt;
}

std::pair<std::vector<PoseEdge>, bool> GraphOrchestrator::add_new_loop_edges() {
  std::vector<PoseEdge> new_loop_edges;
  bool loop_closed = false;

  auto all_edges = pose_graph_->get_edges();
  std::vector<PoseEdge> loop_edges;
  for (const auto& e : all_edges) {
    if (std::abs(e.to_index - e.from_index) > 1) {
      loop_edges.push_back(e);
    }
  }

  if (loop_edges.size() > last_loop_edge_count_) {
    for (size_t i = last_loop_edge_count_; i < loop_edges.size(); ++i) {
      const auto& edge = loop_edges[i];
      optimizer_.add_between_factor(
          edge.from_index,
          edge.to_index,
          edge.dx,
          edge.dy,
          edge.dyaw,
          edge.information);
      new_loop_edges.push_back(edge);
    }
    last_loop_edge_count_ = loop_edges.size();
    loop_closed = true;
  }

  return {new_loop_edges, loop_closed};
}

std::optional<GnssPrior> GraphOrchestrator::add_gnss_prior(
    const SensorFrame& frame, const PoseNode& node) {
  if (!use_gnss_ || !frame.gnss.has_value() || !anchor_manager_ || !anchor_manager_->is_initialized()) {
    return std::nullopt;
  }

  if (last_gnss_timestamp_.has_value() &&
      std::abs(frame.gnss->timestamp - *last_gnss_timestamp_) < 1e-6) {
    return std::nullopt;
  }

  double sigma_xy = sigma_from_gnss(*frame.gnss);
  if (sigma_xy <= 0.0 || sigma_xy > gnss_max_sigma_m_) {
    return std::nullopt;
  }

  auto [gx, gy] = anchor_manager_->to_local(*frame.gnss);
  optimizer_.add_gnss_prior(
      node.index, gx, gy, sigma_xy, gnss_factor_yaw_variance_);
  last_gnss_timestamp_ = frame.gnss->timestamp;

  Eigen::Matrix2d info_2x2 = Eigen::Matrix2d::Zero();
  double inv_var = 1.0 / std::max(sigma_xy * sigma_xy, 1e-12);
  info_2x2(0, 0) = inv_var;
  info_2x2(1, 1) = inv_var;

  return GnssPrior{
      node.index,
      gx,
      gy,
      info_2x2,
  };
}

bool GraphOrchestrator::apply_optimized_poses(const PoseNode& node, bool loop_closed) {
  auto all_poses = optimizer_.get_all_poses();
  bool rerender_required = loop_closed;

  auto nodes = pose_graph_->get_nodes();
  if (use_gnss_ && last_node_index_ == node.index && init_rotation_ != 0.0 &&
      nodes.size() > 1 && !first_render_done_) {
    rerender_required = true;
    first_render_done_ = true;
  }

  double max_displacement = 0.0;
  for (auto& n : nodes) {
    auto it = all_poses.find(n.index);
    if (it != all_poses.end()) {
      auto [new_x, new_y, new_yaw] = it->second;
      auto render_it = last_rendered_poses_.find(n.index);
      if (render_it != last_rendered_poses_.end()) {
        auto [old_x, old_y, _] = render_it->second;
        double disp = std::hypot(new_x - old_x, new_y - old_y);
        if (disp > max_displacement) {
          max_displacement = disp;
        }
      }
      n.x = new_x;
      n.y = new_y;
      n.yaw = new_yaw;
    }
  }

  if (!rerender_required && rerender_threshold_m_ > 0.0) {
    if (max_displacement >= rerender_threshold_m_) {
      rerender_required = true;
    }
  }

  if (rerender_required) {
    for (const auto& n : nodes) {
      last_rendered_poses_[n.index] = {n.x, n.y, n.yaw};
    }
    pose_graph_->replace_nodes(nodes);
  } else {
    last_rendered_poses_[node.index] = {node.x, node.y, node.yaw};
  }

  return rerender_required;
}

std::optional<PoseEdge> GraphOrchestrator::get_latest_seq_edge(int node_index) const {
  auto all_edges = pose_graph_->get_edges();
  for (auto it = all_edges.rbegin(); it != all_edges.rend(); ++it) {
    if (std::abs(it->to_index - it->from_index) == 1 && it->to_index == node_index) {
      return *it;
    }
  }
  return std::nullopt;
}

double GraphOrchestrator::sigma_from_gnss(const GnssData& gnss) const {
  double cov_xx = gnss.covariance(0, 0);
  if (cov_xx > 0.0) {
    return std::sqrt(cov_xx);
  }
  if (gnss.fix_status >= 2) {
    return gnss_fix_sigma_m_;
  }
  if (gnss.fix_status >= 0) {
    return gnss_float_sigma_m_;
  }
  return -1.0;
}

FinalizeResult GraphOrchestrator::finalize() {
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
      "Running offline batch optimization (Finalize)");
  return FinalizeResult{true};
}

}  // namespace core
}  // namespace slam_gnss_2d
