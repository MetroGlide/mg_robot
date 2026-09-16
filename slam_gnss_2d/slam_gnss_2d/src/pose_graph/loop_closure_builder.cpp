#include "slam_gnss_2d/pose_graph/loop_closure_builder.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "slam_gnss_2d/core/geometry.hpp"

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
  if (submap_radius_ <= 0.0) {
    return candidate.scan ? core::scan_to_points(candidate.scan) : std::vector<Eigen::Vector2d>{};
  }

  std::vector<Eigen::Vector2d> world_pts;
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

    auto pts = core::scan_to_points(node.scan);
    if (pts.empty()) {
      continue;
    }

    auto w_pts = core::points_local_to_world(pts, node.x, node.y, node.yaw);
    world_pts.insert(world_pts.end(), w_pts.begin(), w_pts.end());
  }

  if (world_pts.empty()) {
    return candidate.scan ? core::scan_to_points(candidate.scan) : std::vector<Eigen::Vector2d>{};
  }

  return core::points_world_to_local(world_pts, candidate.x, candidate.y, candidate.yaw);
}

bool LoopClosureBuilder::try_add_loop_edge(
    const core::PoseNode& node,
    const core::PoseNode& candidate) {
  for (const auto& e : loop_edges_) {
    if (e.from_index == candidate.index) {
      return false;
    }
  }

  auto src_pts = build_candidate_submap(candidate);
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
  loop_matcher_->set_target_cloud(src_pts);
  auto result = loop_matcher_->match(node.scan, initial_guess);

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

  if (max_score_ > 0.0 && result.score > max_score_) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.loop_closure_builder"),
        "Loop edge score check failed: node %d <- candidate %d (score=%.6f, limit=%.6f)",
        node.index, candidate.index, result.score, max_score_);
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
      "Loop edge added: %d -> %d (dx=%.3f, dy=%.3f, dyaw=%.1f deg, score=%.6f)",
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
