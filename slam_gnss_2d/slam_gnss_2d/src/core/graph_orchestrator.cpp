#include "slam_gnss_2d/core/graph_orchestrator.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <Eigen/Dense>

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
    double rerender_threshold_m,
    double gnss_min_interval_m,
    double anchor_sigma_m,
    double anchor_init_yaw_sigma_rad,
    double gnss_max_innovation_m,
    const std::string& gnss_robust_kernel,
    double gnss_robust_kernel_scale,
    int gnss_prior_min_fix_status,
    bool dynamic_reanchor_enabled,
    int dynamic_reanchor_min_fix_status,
    int dynamic_reanchor_min_samples,
    double dynamic_reanchor_min_distance_m,
    double dynamic_reanchor_max_residual_rms_m,
    bool batch_on_finalize,
    int batch_max_iterations)
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
      gnss_min_interval_m_(gnss_min_interval_m),
      anchor_sigma_m_(anchor_sigma_m),
      anchor_init_yaw_sigma_rad_(anchor_init_yaw_sigma_rad),
      gnss_max_innovation_m_(gnss_max_innovation_m),
      gnss_robust_kernel_(gnss_robust_kernel),
      gnss_robust_kernel_scale_(gnss_robust_kernel_scale),
      gnss_prior_min_fix_status_(gnss_prior_min_fix_status),
      dynamic_reanchor_enabled_(dynamic_reanchor_enabled),
      dynamic_reanchor_min_fix_status_(dynamic_reanchor_min_fix_status),
      dynamic_reanchor_min_samples_(dynamic_reanchor_min_samples),
      dynamic_reanchor_min_distance_m_(dynamic_reanchor_min_distance_m),
      dynamic_reanchor_max_residual_rms_m_(dynamic_reanchor_max_residual_rms_m),
      batch_on_finalize_(batch_on_finalize),
      batch_max_iterations_(batch_max_iterations),
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
  auto all_edges = pose_graph_->get_edges();
  auto new_seq_edge = add_latest_seq_edge(*node, all_edges);
  auto [new_loop_edges, loop_closed] = add_new_loop_edges(all_edges);

  bool reanchored = false;
  if (dynamic_reanchor_enabled_ && !dynamic_reanchored_) {
    reanchored = try_dynamic_reanchor(*node, frame);
    if (reanchored) {
      auto current_nodes = pose_graph_->get_nodes();
      if (!current_nodes.empty()) {
        *node = current_nodes.back();
      }
    }
  }

  auto new_gnss_prior = add_gnss_prior(frame, *node);
  try {
    optimizer_.update();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
        rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
        "optimizer_.update() exception at node %d: %s", node->index, e.what());
    throw;
  }
  bool rerender_required = apply_optimized_poses(*node, loop_closed || reanchored);

  auto updated_pose = optimizer_.get_pose(node->index);
  if (updated_pose.has_value()) {
    node->x = std::get<0>(*updated_pose);
    node->y = std::get<1>(*updated_pose);
    node->yaw = std::get<2>(*updated_pose);
  }

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
  init_gnss_pts_.push_back({lx, ly});

  auto nodes = pose_graph_->get_nodes();
  if (nodes.empty()) {
    return;
  }
  const auto& curr_node = nodes.back();
  init_odom_pts_.push_back({curr_node.x, curr_node.y});

  if (std::hypot(lx, ly) < gnss_init_distance_m_) {
    return;
  }

  double rot = 0.0;
  auto gnss_pca = estimate_heading_pca(init_gnss_pts_);
  auto odom_pca = estimate_heading_pca(init_odom_pts_);

  if (gnss_pca.has_value() && odom_pca.has_value() && gnss_pca->second >= 0.85) {
    rot = angle_diff(gnss_pca->first, odom_pca->first);
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
        "PCA Heading alignment succeeded: GNSS=%.1f deg (linearity=%.1f%%, N=%zu), "
        "Odom=%.1f deg (linearity=%.1f%%, N=%zu), rot=%.3f rad (%.1f deg)",
        gnss_pca->first * 180.0 / M_PI, gnss_pca->second * 100.0, init_gnss_pts_.size(),
        odom_pca->first * 180.0 / M_PI, odom_pca->second * 100.0, init_odom_pts_.size(),
        rot, rot * 180.0 / M_PI);
  } else {
    double theta0 = std::atan2(ly, lx);
    const auto& node0 = nodes.front();
    double odom_dx = curr_node.x - node0.x;
    double odom_dy = curr_node.y - node0.y;
    double odom_dist = std::hypot(odom_dx, odom_dy);
    if (odom_dist > 0.5) {
      double odom_heading = std::atan2(odom_dy, odom_dx);
      rot = angle_diff(theta0, odom_heading);
    } else {
      rot = angle_diff(theta0, node0.yaw);
    }
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
        "PCA Heading alignment fell back to 2-point vector: rot=%.3f rad (%.1f deg)",
        rot, rot * 180.0 / M_PI);
  }

  init_rotation_ = rot;
  init_gnss_pts_.clear();
  init_odom_pts_.clear();

  const PoseNode node0 = nodes.front();
  optimizer_.initialize(node0.index, 0.0, 0.0, node0.yaw + rot, anchor_sigma_m_, anchor_init_yaw_sigma_rad_);
  nodes = rebase_and_rotate_nodes(nodes, rot);
  for (const auto& n : nodes) {
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

  last_loop_edge_count_ = 0;
  for (const auto& e : pose_graph_->get_edges()) {
    if (std::abs(e.to_index - e.from_index) >= 15) {
      last_loop_edge_count_++;
    }
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
  optimizer_.update();
  initialized_ = true;
  last_node_index_ = node.index;
}

std::optional<PoseEdge> GraphOrchestrator::add_latest_seq_edge(const PoseNode& node) {
  return add_latest_seq_edge(node, pose_graph_->get_edges());
}

std::optional<PoseEdge> GraphOrchestrator::add_latest_seq_edge(
    const PoseNode& node, const std::vector<PoseEdge>& all_edges) {
  auto latest_seq_edge = get_latest_seq_edge(node.index, all_edges);
  if (latest_seq_edge.has_value() && node.index > last_node_index_) {
    double x = node.x;
    double y = node.y;
    double yaw = node.yaw;

    auto prev_pose = optimizer_.get_pose(latest_seq_edge->from_index);
    if (prev_pose.has_value()) {
      auto [px, py, pyaw] = *prev_pose;
      double c = std::cos(pyaw);
      double s = std::sin(pyaw);
      x = px + c * latest_seq_edge->dx - s * latest_seq_edge->dy;
      y = py + s * latest_seq_edge->dx + c * latest_seq_edge->dy;
      yaw = pyaw + latest_seq_edge->dyaw;
    }

    optimizer_.add_initial_estimate(node.index, x, y, yaw);
    optimizer_.add_between_factor(
        latest_seq_edge->from_index,
        latest_seq_edge->to_index,
        latest_seq_edge->dx,
        latest_seq_edge->dy,
        latest_seq_edge->dyaw,
        latest_seq_edge->information);

    // 新ノード node.index に向かう局所メッシュリンク (1 < diff < 15) も BetweenFactor に登録
    // 直近に追加されたエッジなので末尾から逆順走査
    for (auto it = all_edges.rbegin(); it != all_edges.rend(); ++it) {
      if (it->to_index == node.index) {
        int diff = std::abs(it->to_index - it->from_index);
        if (diff > 1 && diff < 15) {
          optimizer_.add_between_factor(
              it->from_index,
              it->to_index,
              it->dx,
              it->dy,
              it->dyaw,
              it->information);
        }
      }
    }

    last_node_index_ = node.index;
    return latest_seq_edge;
  }
  return std::nullopt;
}

std::pair<std::vector<PoseEdge>, bool> GraphOrchestrator::add_new_loop_edges() {
  return add_new_loop_edges(pose_graph_->get_edges());
}

std::pair<std::vector<PoseEdge>, bool> GraphOrchestrator::add_new_loop_edges(
    const std::vector<PoseEdge>& all_edges) {
  std::vector<PoseEdge> new_loop_edges;
  bool loop_closed = false;

  std::vector<const PoseEdge*> loop_edge_ptrs;
  for (const auto& e : all_edges) {
    if (std::abs(e.to_index - e.from_index) >= 15) {
      loop_edge_ptrs.push_back(&e);
    }
  }

  if (loop_edge_ptrs.size() > last_loop_edge_count_) {
    for (size_t i = last_loop_edge_count_; i < loop_edge_ptrs.size(); ++i) {
      const auto& edge = *loop_edge_ptrs[i];
      optimizer_.add_between_factor(
          edge.from_index,
          edge.to_index,
          edge.dx,
          edge.dy,
          edge.dyaw,
          edge.information);
      new_loop_edges.push_back(edge);
    }
    last_loop_edge_count_ = loop_edge_ptrs.size();
    loop_closed = true;
  }

  return {new_loop_edges, loop_closed};
}

bool GraphOrchestrator::try_dynamic_reanchor(const PoseNode& node, const SensorFrame& frame) {
  if (!use_gnss_ || !frame.gnss.has_value() || !anchor_manager_ || !anchor_manager_->is_initialized()) {
    return false;
  }

  if (frame.gnss->fix_status < dynamic_reanchor_min_fix_status_) {
    return false;
  }

  double sigma = sigma_from_gnss(*frame.gnss);
  if (sigma <= 0.0 || sigma > 0.5) {
    return false;
  }

  if (reanchor_samples_.empty() ||
      (Eigen::Vector2d(node.x, node.y) - reanchor_samples_.back().slam_pos).norm() > 0.5) {
    reanchor_samples_.push_back({
        Eigen::Vector2d(node.x, node.y),
        Eigen::Vector2d(frame.gnss->x, frame.gnss->y)});
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
        "Dynamic Re-anchoring sample collected: [%zu/%d], fix_status=%d, sigma=%.2fm, current node=%d",
        reanchor_samples_.size(), dynamic_reanchor_min_samples_,
        frame.gnss->fix_status, sigma, node.index);
  }

  // 直近の局所ウィンドウに限定（最大サンプル数: min_samples + 5、最大スパン: 25m以内）
  while (reanchor_samples_.size() > static_cast<size_t>(dynamic_reanchor_min_samples_ + 5)) {
    reanchor_samples_.pop_front();
  }
  while (reanchor_samples_.size() >= 2 &&
         (reanchor_samples_.back().slam_pos - reanchor_samples_.front().slam_pos).norm() > 25.0) {
    reanchor_samples_.pop_front();
  }

  if (static_cast<int>(reanchor_samples_.size()) < dynamic_reanchor_min_samples_) {
    return false;
  }

  double max_span = 0.0;
  for (size_t i = 0; i < reanchor_samples_.size(); ++i) {
    for (size_t j = i + 1; j < reanchor_samples_.size(); ++j) {
      double d = (reanchor_samples_[i].slam_pos - reanchor_samples_[j].slam_pos).norm();
      if (d > max_span) {
        max_span = d;
      }
    }
  }
  if (max_span < dynamic_reanchor_min_distance_m_) {
    return false;
  }

  const size_t n = reanchor_samples_.size();
  Eigen::Vector2d p_mean = Eigen::Vector2d::Zero();
  Eigen::Vector2d q_mean = Eigen::Vector2d::Zero();
  for (const auto& s : reanchor_samples_) {
    p_mean += s.slam_pos;
    q_mean += s.utm_pos;
  }
  p_mean /= static_cast<double>(n);
  q_mean /= static_cast<double>(n);

  Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
  for (const auto& s : reanchor_samples_) {
    H += (s.slam_pos - p_mean) * (s.utm_pos - q_mean).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();

  double det = (V * U.transpose()).determinant();
  Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
  if (det < 0.0) {
    S(1, 1) = -1.0;
  }

  Eigen::Matrix2d R = V * S * U.transpose();
  double delta_theta = std::atan2(R(1, 0), R(0, 0));

  Eigen::Vector2d anchor_star = q_mean - R * p_mean;

  double sum_sq_err = 0.0;
  for (const auto& s : reanchor_samples_) {
    Eigen::Vector2d pred = R * s.slam_pos + anchor_star;
    sum_sq_err += (pred - s.utm_pos).squaredNorm();
  }
  double rms = std::sqrt(sum_sq_err / static_cast<double>(n));

  if (rms > dynamic_reanchor_max_residual_rms_m_) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
        "Dynamic Re-anchoring rejected: estimated delta_theta=%.1f deg, but RMS residual %.3fm exceeds threshold %.3fm",
        delta_theta * 180.0 / M_PI, rms, dynamic_reanchor_max_residual_rms_m_);
    return false;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
      "Dynamic Re-anchoring (translation only): Samples=%zu, span=%.2fm, rot=%.3f rad (%.1f deg), RMS=%.3fm, "
      "anchor=(%.2f, %.2f) -> (%.2f, %.2f)",
      n, max_span, delta_theta, delta_theta * 180.0 / M_PI, rms,
      anchor_manager_->anchor_utm()->first, anchor_manager_->anchor_utm()->second,
      anchor_star.x(), anchor_star.y());

  anchor_manager_->update_anchor(anchor_star.x(), anchor_star.y());
  last_gnss_pos_.reset();
  dynamic_reanchored_ = true;
  return true;
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

  if (frame.gnss->fix_status < gnss_prior_min_fix_status_) {
    gnss_rejected_status_count_++;
    return std::nullopt;
  }

  double sigma_xy = sigma_from_gnss(*frame.gnss);
  if (sigma_xy <= 0.0 || sigma_xy > gnss_max_sigma_m_) {
    gnss_rejected_sigma_count_++;
    return std::nullopt;
  }

  auto [gx, gy] = anchor_manager_->to_local(*frame.gnss);

  if (gnss_max_innovation_m_ > 0.0) {
    double innovation = std::hypot(gx - node.x, gy - node.y);
    if (innovation > gnss_max_innovation_m_) {
      gnss_rejected_innovation_count_++;
      return std::nullopt;
    }
  }

  if (last_gnss_pos_.has_value() && gnss_min_interval_m_ > 0.0) {
    double dx = gx - last_gnss_pos_->first;
    double dy = gy - last_gnss_pos_->second;
    if (std::hypot(dx, dy) < gnss_min_interval_m_) {
      gnss_rejected_interval_count_++;
      return std::nullopt;
    }
  }

  optimizer_.add_gnss_prior(
      node.index, gx, gy, sigma_xy, gnss_factor_yaw_variance_,
      gnss_robust_kernel_, gnss_robust_kernel_scale_);
  last_gnss_timestamp_ = frame.gnss->timestamp;
  last_gnss_pos_ = {gx, gy};
  gnss_prior_count_++;

  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.graph_orchestrator"),
      "GNSS prior #%d added at node %d: (%.2f, %.2f), sigma=%.2fm, status=%d",
      gnss_prior_count_, node.index, gx, gy, sigma_xy, frame.gnss->fix_status);

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

  // ノード座標は最適化完了時に常に同期更新
  pose_graph_->replace_nodes(nodes);

  if (rerender_required) {
    for (const auto& n : nodes) {
      last_rendered_poses_[n.index] = {n.x, n.y, n.yaw};
    }
  } else {
    last_rendered_poses_[node.index] = {node.x, node.y, node.yaw};
  }

  return rerender_required;
}

std::optional<PoseEdge> GraphOrchestrator::get_latest_seq_edge(int node_index) const {
  return get_latest_seq_edge(node_index, pose_graph_->get_edges());
}

std::optional<PoseEdge> GraphOrchestrator::get_latest_seq_edge(
    int node_index, const std::vector<PoseEdge>& all_edges) const {
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
      "Finalizing GraphOrchestrator. GNSS stats: added=%d, "
      "rejected_status=%d, rejected_sigma=%d, rejected_interval=%d, rejected_innovation=%d",
      gnss_prior_count_, gnss_rejected_status_count_, gnss_rejected_sigma_count_,
      gnss_rejected_interval_count_, gnss_rejected_innovation_count_);

  if (batch_on_finalize_) {
    optimizer_.run_batch_optimization(batch_max_iterations_);
  }

  auto all_poses = optimizer_.get_all_poses();
  auto nodes = pose_graph_->get_nodes();
  for (auto& n : nodes) {
    auto it = all_poses.find(n.index);
    if (it != all_poses.end()) {
      n.x = std::get<0>(it->second);
      n.y = std::get<1>(it->second);
      n.yaw = std::get<2>(it->second);
    }
  }
  pose_graph_->replace_nodes(nodes);

  return FinalizeResult{true};
}

std::optional<std::pair<double, double>> GraphOrchestrator::estimate_heading_pca(
    const std::vector<std::pair<double, double>>& pts) {
  if (pts.size() < 2) {
    return std::nullopt;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto& p : pts) {
    sum_x += p.first;
    sum_y += p.second;
  }
  double inv_n = 1.0 / static_cast<double>(pts.size());
  double mean_x = sum_x * inv_n;
  double mean_y = sum_y * inv_n;

  double c_xx = 0.0;
  double c_yy = 0.0;
  double c_xy = 0.0;
  for (const auto& p : pts) {
    double dx = p.first - mean_x;
    double dy = p.second - mean_y;
    c_xx += dx * dx;
    c_yy += dy * dy;
    c_xy += dx * dy;
  }
  c_xx *= inv_n;
  c_yy *= inv_n;
  c_xy *= inv_n;

  Eigen::Matrix2d cov;
  cov << c_xx, c_xy,
         c_xy, c_yy;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }

  Eigen::Vector2d major_axis = solver.eigenvectors().col(1);
  double lambda_max = solver.eigenvalues()(1);
  double lambda_min = solver.eigenvalues()(0);
  double linearity = lambda_max / std::max(lambda_max + lambda_min, 1e-9);

  double d_x = pts.back().first - pts.front().first;
  double d_y = pts.back().second - pts.front().second;
  if (major_axis.x() * d_x + major_axis.y() * d_y < 0.0) {
    major_axis = -major_axis;
  }

  double heading = std::atan2(major_axis.y(), major_axis.x());
  return std::make_pair(heading, linearity);
}

}  // namespace core
}  // namespace slam_gnss_2d
