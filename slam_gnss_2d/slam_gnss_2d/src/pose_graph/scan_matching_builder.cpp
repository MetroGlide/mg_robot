#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

namespace {
Eigen::Matrix3d make_odom_information() {
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
  info(0, 0) = 100.0;
  info(1, 1) = 100.0;
  info(2, 2) = 50.0;
  return info;
}

Eigen::Matrix3d make_odom_fallback_information() {
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
  info(0, 0) = 10.0;
  info(1, 1) = 10.0;
  info(2, 2) = 5.0;
  return info;
}
}  // namespace

ScanMatchingBuilder::ScanMatchingBuilder(
    scan_matching::ScanMatcherPtr matcher,
    scan_matching::ReferenceProviderPtr provider,
    double min_translation,
    double min_rotation,
    int max_failure_streak,
    double max_translation_drift)
    : matcher_(matcher),
      provider_(provider),
      min_translation_(min_translation),
      min_rotation_(min_rotation),
      max_failure_streak_(max_failure_streak),
      max_translation_drift_(max_translation_drift) {}

std::optional<core::PoseNode> ScanMatchingBuilder::add_scan(
    const core::ScanDataPtr& scan,
    const core::OdomData& odom) {
  if (!scan) {
    return std::nullopt;
  }

  if (nodes_.empty()) {
    core::PoseNode node{
        0,
        scan->timestamp,
        odom.x,
        odom.y,
        odom.yaw,
        scan,
    };
    nodes_.push_back(node);
    provider_->update(node);
    last_odom_ = odom;
    return node;
  }

  if (!last_odom_.has_value()) {
    return std::nullopt;
  }

  double dx_w = odom.x - last_odom_->x;
  double dy_w = odom.y - last_odom_->y;
  double dist = std::hypot(dx_w, dy_w);
  double dyaw = std::abs(core::angle_diff(odom.yaw, last_odom_->yaw));
  if (dist < min_translation_ && dyaw < min_rotation_) {
    return std::nullopt;
  }

  const auto prev_node = nodes_.back();
  const int prev_index = prev_node.index;

  auto [dx_local, dy_local] = core::world_delta_to_local(
      dx_w, dy_w, last_odom_->yaw);
  double dyaw_delta = core::angle_diff(odom.yaw, last_odom_->yaw);

  core::OdomData initial_guess{
      scan->timestamp,
      dx_local,
      dy_local,
      dyaw_delta,
  };

  auto src_pts = provider_->get_reference_pts();
  double dx_icp = dx_local;
  double dy_icp = dy_local;
  double dyaw_icp = dyaw_delta;
  Eigen::Matrix3d edge_info;
  bool is_odom_fallback = false;
  double score = 0.0;

  if (src_pts.has_value()) {
    icp_attempt_count_++;
    matcher_->set_target_cloud(*src_pts);
    auto result = matcher_->match(scan, initial_guess);

    if (!result.converged) {
      failure_streak_++;
      RCLCPP_WARN(
          rclcpp::get_logger("slam_gnss_2d.scan_matching_builder"),
          "Matcher did not converge at node %zu (init dx=%.3f, dy=%.3f, dyaw=%.1f deg, streak=%d/%d); falling back to odometry",
          nodes_.size(), initial_guess.x, initial_guess.y,
          initial_guess.yaw * 180.0 / M_PI, failure_streak_, max_failure_streak_);

      dx_icp = dx_local;
      dy_icp = dy_local;
      dyaw_icp = dyaw_delta;
      edge_info = make_odom_fallback_information();
      odom_fallback_count_++;
      is_odom_fallback = true;
      score = 0.0;
    } else {
      failure_streak_ = 0;
      icp_success_count_++;
      dx_icp = result.dx;
      dy_icp = result.dy;
      dyaw_icp = result.dyaw;
      edge_info = result.information;
      is_odom_fallback = false;
      score = result.score;

      bool is_straight_motion = (std::abs(dyaw_delta) < 0.05 && std::abs(dyaw_icp) < 0.05);
      if (is_straight_motion && max_translation_drift_ > 0.0) {
        double translation_drift = std::abs(dx_icp - dx_local);
        if (translation_drift > max_translation_drift_) {
          RCLCPP_DEBUG(
              rclcpp::get_logger("slam_gnss_2d.scan_matching_builder"),
              "Degeneracy slip detected at node %zu: dx_match=%.3f vs dx_odom=%.3f (drift=%.3fm > %.3fm). Preserving odom translation.",
              nodes_.size(), dx_icp, dx_local, translation_drift, max_translation_drift_);
          dx_icp = dx_local;
          edge_info(0, 0) = std::min(edge_info(0, 0), 20.0);
        }
      }
    }
  } else {
    dx_icp = dx_local;
    dy_icp = dy_local;
    dyaw_icp = dyaw_delta;
    edge_info = make_odom_information();
    is_odom_fallback = true;
    score = 0.0;
  }

  auto [dx_world, dy_world] = core::local_delta_to_world(
      dx_icp, dy_icp, prev_node.yaw);
  double new_x = prev_node.x + dx_world;
  double new_y = prev_node.y + dy_world;
  double new_yaw = core::normalize_angle(prev_node.yaw + dyaw_icp);

  core::PoseNode node{
      static_cast<int>(nodes_.size()),
      scan->timestamp,
      new_x,
      new_y,
      new_yaw,
      scan,
  };
  nodes_.push_back(node);
  provider_->update(node);

  edges_.emplace_back(core::PoseEdge{
      prev_index,
      node.index,
      dx_icp,
      dy_icp,
      dyaw_icp,
      edge_info,
      score,
      is_odom_fallback,
  });

  last_odom_ = odom;
  return node;
}

std::vector<core::PoseNode> ScanMatchingBuilder::get_nodes() const {
  return nodes_;
}

std::vector<core::PoseEdge> ScanMatchingBuilder::get_edges() const {
  return edges_;
}

void ScanMatchingBuilder::reset() {
  nodes_.clear();
  edges_.clear();
  last_odom_.reset();
  failure_streak_ = 0;
}

void ScanMatchingBuilder::replace_nodes(const std::vector<core::PoseNode>& nodes) {
  nodes_ = nodes;
  provider_->sync_poses(nodes);
  provider_->invalidate_cache();
}

}  // namespace pose_graph
}  // namespace slam_gnss_2d
