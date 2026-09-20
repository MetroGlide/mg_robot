#include "slam_gnss_2d/pose_graph/loop_closure_builder.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

LoopClosureBuilder::LoopClosureBuilder(
    std::shared_ptr<ScanMatchingBuilder> inner,
    scan_matching::ScanMatcherPtr loop_matcher,
    double loop_closure_search_radius,
    int loop_closure_min_node_gap,
    int loop_closure_max_failure_streak,
    double max_loop_dyaw_deg,
    double loop_closure_crossing_reject_deg,
    double loop_closure_submap_radius,
    double loop_closure_max_score)
    : inner_(inner),
      loop_matcher_(loop_matcher),
      search_radius_(loop_closure_search_radius),
      min_node_gap_(loop_closure_min_node_gap),
      max_failure_streak_(loop_closure_max_failure_streak),
      max_loop_dyaw_rad_(max_loop_dyaw_deg * M_PI / 180.0),
      crossing_reject_rad_(loop_closure_crossing_reject_deg > 0.0
                              ? loop_closure_crossing_reject_deg * M_PI / 180.0
                              : 0.0),
      submap_radius_(loop_closure_submap_radius),
      max_score_(loop_closure_max_score) {}

std::optional<core::PoseNode> LoopClosureBuilder::add_scan(
    const core::ScanDataPtr& scan,
    const core::OdomData& odom) {
  auto node = inner_->add_scan(scan, odom);
  if (!node.has_value()) {
    return std::nullopt;
  }

  all_nodes_cache_.push_back(*node);
  loop_just_closed_flag_ = false;

  auto candidates = find_loop_candidates(*node);
  int new_loops_added = 0;
  for (const auto& candidate : candidates) {
    if (try_add_loop_edge(*node, candidate)) {
      new_loops_added++;
    }
  }

  if (new_loops_added > 0) {
    loop_just_closed_flag_ = true;
  }

  return node;
}

std::vector<core::PoseNode> LoopClosureBuilder::find_loop_candidates(
    const core::PoseNode& node) const {
  std::vector<core::PoseNode> candidates;
  if (all_nodes_cache_.empty()) {
    return candidates;
  }

  double search_radius_sq = search_radius_ * search_radius_;
  for (const auto& n : all_nodes_cache_) {
    if (!n.scan) {
      continue;
    }
    if ((node.index - n.index) < min_node_gap_) {
      continue;
    }
    double dx = node.x - n.x;
    double dy = node.y - n.y;
    if (dx * dx + dy * dy <= search_radius_sq) {
      candidates.push_back(n);
    }
  }
  return candidates;
}

std::vector<Eigen::Vector2d> LoopClosureBuilder::build_candidate_submap(
    const core::PoseNode& candidate) const {
  auto pair = build_candidate_submap_with_normals(candidate);
  return pair.first;
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
LoopClosureBuilder::build_candidate_submap_with_normals(
    const core::PoseNode& candidate) const {
  if (submap_radius_ <= 0.0) {
    if (!candidate.scan) return {{}, {}};
    if (candidate.normals && !candidate.normals->empty()) {
      return {core::scan_to_points(candidate.scan), *candidate.normals};
    }
    return core::scan_to_points_and_normals(candidate.scan);
  }

  std::vector<Eigen::Vector2d> world_pts;
  std::vector<Eigen::Vector2d> world_normals;
  double submap_radius_sq = submap_radius_ * submap_radius_;

  for (const auto& node : all_nodes_cache_) {
    if (!node.scan) {
      continue;
    }
    double dx = node.x - candidate.x;
    double dy = node.y - candidate.y;
    if (dx * dx + dy * dy > submap_radius_sq) {
      continue;
    }

    std::vector<Eigen::Vector2d> pts;
    std::vector<Eigen::Vector2d> normals;
    if (node.normals && !node.normals->empty()) {
      pts = core::scan_to_points(node.scan);
      normals = *node.normals;
    } else {
      auto pair = core::scan_to_points_and_normals(node.scan);
      pts = std::move(pair.first);
      normals = std::move(pair.second);
    }

    auto w_pts = core::points_local_to_world(pts, node.x, node.y, node.yaw);
    auto w_normals = core::normals_local_to_world(normals, node.yaw);
    world_pts.insert(world_pts.end(), w_pts.begin(), w_pts.end());
    world_normals.insert(world_normals.end(), w_normals.begin(), w_normals.end());
  }

  if (world_pts.empty()) {
    if (!candidate.scan) return {{}, {}};
    return core::scan_to_points_and_normals(candidate.scan);
  }

  auto local_pts = core::points_world_to_local(world_pts, candidate.x, candidate.y, candidate.yaw);
  auto local_normals = core::normals_world_to_local(world_normals, candidate.yaw);
  return {std::move(local_pts), std::move(local_normals)};
}

std::vector<Eigen::Vector2d> LoopClosureBuilder::build_query_submap(
    const core::PoseNode& node, int query_window) const {
  std::vector<Eigen::Vector2d> world_pts;
  int total_nodes = static_cast<int>(all_nodes_cache_.size());
  int start_idx = std::max(0, total_nodes - query_window);

  for (int i = start_idx; i < total_nodes; ++i) {
    const auto& n = all_nodes_cache_[i];
    if (!n.scan) continue;
    auto pts = core::scan_to_points(n.scan);
    auto w_pts = core::points_local_to_world(pts, n.x, n.y, n.yaw);
    world_pts.insert(world_pts.end(), w_pts.begin(), w_pts.end());
  }

  if (world_pts.empty()) {
    return core::scan_to_points(node.scan);
  }
  return core::points_world_to_local(world_pts, node.x, node.y, node.yaw);
}

bool LoopClosureBuilder::try_add_loop_edge(
    const core::PoseNode& node,
    const core::PoseNode& candidate) {
  for (const auto& e : loop_edges_) {
    if (e.from_index == candidate.index) {
      return false;
    }
  }

  auto [src_pts, src_normals] = build_candidate_submap_with_normals(candidate);
  if (src_pts.empty()) {
    return false;
  }

  double dx_w = node.x - candidate.x;
  double dy_w = node.y - candidate.y;
  auto [dx_local, dy_local] = core::world_delta_to_local(dx_w, dy_w, candidate.yaw);
  core::OdomData initial_guess{
      node.timestamp,
      dx_local,
      dy_local,
      core::angle_diff(node.yaw, candidate.yaw),
  };

  loop_attempt_count_++;
  loop_matcher_->set_target_cloud_with_normals(src_pts, src_normals);

  // クエリ側も直近サブマップ点群（直近5キーフレーム）を使用
  auto query_pts = build_query_submap(node, 5);

  core::MatchResult result;
  if (auto icp = dynamic_cast<scan_matching::ICPMatcher*>(loop_matcher_.get())) {
    result = icp->match(query_pts, initial_guess);
  } else {
    result = loop_matcher_->match(node.scan, initial_guess);
  }

  if (!result.converged) {
    loop_failure_streak_++;
    if (loop_failure_streak_ >= max_failure_streak_) {
      RCLCPP_WARN(
          rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
          "Loop failure streak limit reached at node %d; resetting streak",
          node.index);
      loop_failure_streak_ = 0;
    }
    return false;
  }

  // 1. 幾何ゲート：初期値からの移動量乖離チェック（過大なズレは誤マッチング）
  double translation_drift = std::hypot(result.dx - initial_guess.x, result.dy - initial_guess.y);
  if (translation_drift > 1.0) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
        "Loop edge translation drift check failed: node %d <- candidate %d (drift=%.3fm > 1.0m)",
        node.index, candidate.index, translation_drift);
    return false;
  }

  // 2. 角度ゲート
  double abs_dyaw = std::abs(result.dyaw);
  if (abs_dyaw > max_loop_dyaw_rad_) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
        "Loop edge dyaw check failed (over limit): node %d <- candidate %d (dyaw=%.1f deg, limit=%.1f deg)",
        node.index, candidate.index, abs_dyaw * 180.0 / M_PI, max_loop_dyaw_rad_ * 180.0 / M_PI);
    return false;
  }

  if (crossing_reject_rad_ > 0.0 &&
      crossing_reject_rad_ <= abs_dyaw &&
      abs_dyaw <= (M_PI - crossing_reject_rad_)) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
        "Loop edge dyaw check failed (crossing band): node %d <- candidate %d (dyaw=%.1f deg)",
        node.index, candidate.index, abs_dyaw * 180.0 / M_PI);
    return false;
  }

  // 3. マッチングスコアチェック
  if (max_score_ > 0.0 && result.score > max_score_) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
        "Loop edge score check failed: node %d <- candidate %d (score=%.6f, limit=%.6f)",
        node.index, candidate.index, result.score, max_score_);
    return false;
  }

  // 4. 縮退（Rank / 拘束不足）チェック：情報行列の最小固有値
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(result.information);
  if (eigensolver.info() == Eigen::Success && eigensolver.eigenvalues()(0) < 5.0) {
    RCLCPP_DEBUG(
        rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
        "Loop edge degenerate check failed: node %d <- candidate %d (min eigenvalue=%.2f < 5.0)",
        node.index, candidate.index, eigensolver.eigenvalues()(0));
    return false;
  }

  loop_failure_streak_ = 0;
  loop_success_count_++;
  loop_edges_.emplace_back(core::PoseEdge{
      candidate.index,
      node.index,
      result.dx,
      result.dy,
      result.dyaw,
      result.information,
      result.score,
      false,
  });

  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
      "Submap loop edge added: %d -> %d (dx=%.3f, dy=%.3f, dyaw=%.1f deg, score=%.6f)",
      candidate.index, node.index, result.dx, result.dy,
      result.dyaw * 180.0 / M_PI, result.score);
  return true;
}

std::vector<core::PoseNode> LoopClosureBuilder::get_nodes() const {
  return inner_->get_nodes();
}

std::vector<core::PoseEdge> LoopClosureBuilder::get_edges() const {
  auto edges = inner_->get_edges();
  edges.insert(edges.end(), loop_edges_.begin(), loop_edges_.end());
  return edges;
}

void LoopClosureBuilder::reset() {
  inner_->reset();
  loop_edges_.clear();
  loop_failure_streak_ = 0;
  loop_just_closed_flag_ = false;
  loop_attempt_count_ = 0;
  loop_success_count_ = 0;
  all_nodes_cache_.clear();
}

std::vector<core::PoseEdge> LoopClosureBuilder::get_loop_edges() const {
  return loop_edges_;
}

bool LoopClosureBuilder::loop_just_closed() {
  bool result = loop_just_closed_flag_;
  loop_just_closed_flag_ = false;
  return result;
}

void LoopClosureBuilder::replace_nodes(const std::vector<core::PoseNode>& nodes) {
  inner_->replace_nodes(nodes);
  std::unordered_map<int, core::PoseNode> node_map;
  for (const auto& n : nodes) {
    node_map[n.index] = n;
  }
  for (auto& cached_node : all_nodes_cache_) {
    auto it = node_map.find(cached_node.index);
    if (it != node_map.end()) {
      cached_node.x = it->second.x;
      cached_node.y = it->second.y;
      cached_node.yaw = it->second.yaw;
    }
  }
}

}  // namespace pose_graph
}  // namespace slam_gnss_2d
