#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/scan_matching/multi_res_csm_matcher.hpp"

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
    double max_translation_drift,
    bool enable_near_keyframe_links,
    int near_link_buffer_size,
    double near_link_max_distance,
    int near_link_min_index_diff,
    int near_link_max_links_per_node,
    double near_link_max_translation_drift,
    double near_link_max_rotation_drift_deg,
    double near_link_min_eigenvalue)
    : matcher_(matcher),
      provider_(provider),
      min_translation_(min_translation),
      min_rotation_(min_rotation),
      max_failure_streak_(max_failure_streak),
      max_translation_drift_(max_translation_drift),
      enable_near_keyframe_links_(enable_near_keyframe_links),
      near_link_buffer_size_(near_link_buffer_size),
      near_link_max_distance_(near_link_max_distance),
      near_link_min_index_diff_(near_link_min_index_diff),
      near_link_max_links_per_node_(near_link_max_links_per_node),
      near_link_max_translation_drift_(near_link_max_translation_drift),
      near_link_max_rotation_drift_rad_(near_link_max_rotation_drift_deg * M_PI / 180.0),
      near_link_min_eigenvalue_(near_link_min_eigenvalue) {}

std::optional<core::PoseNode> ScanMatchingBuilder::add_scan(
    const core::ScanDataPtr& scan,
    const core::OdomData& odom) {
  if (!scan) {
    return std::nullopt;
  }

  if (nodes_.empty()) {
    auto pair = core::scan_to_points_and_normals(scan);
    auto normals_ptr = std::make_shared<std::vector<Eigen::Vector2d>>(std::move(pair.second));
    core::PoseNode node{
        0,
        scan->timestamp,
        odom.x,
        odom.y,
        odom.yaw,
        scan,
        nullptr,
        normals_ptr,
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

  auto ref_res = provider_->get_reference_pts_and_normals();
  double dx_icp = dx_local;
  double dy_icp = dy_local;
  double dyaw_icp = dyaw_delta;
  Eigen::Matrix3d edge_info;
  bool is_odom_fallback = false;
  double score = 0.0;

  if (ref_res.has_value()) {
    icp_attempt_count_++;
    matcher_->set_target_cloud_with_normals(ref_res->first, ref_res->second);
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

  auto pair = core::scan_to_points_and_normals(scan);
  auto normals_ptr = std::make_shared<std::vector<Eigen::Vector2d>>(std::move(pair.second));

  core::PoseNode node{
      static_cast<int>(nodes_.size()),
      scan->timestamp,
      new_x,
      new_y,
      new_yaw,
      scan,
      nullptr,
      normals_ptr,
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

  if (enable_near_keyframe_links_ && nodes_.size() > static_cast<size_t>(near_link_min_index_diff_)) {
    add_near_keyframe_links(node);
  }

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
  near_link_success_count_ = 0;
}

void ScanMatchingBuilder::replace_nodes(const std::vector<core::PoseNode>& nodes) {
  nodes_ = nodes;
  provider_->sync_poses(nodes);
  provider_->invalidate_cache();
}

void ScanMatchingBuilder::add_near_keyframe_links(const core::PoseNode& current_node) {
  if (!current_node.scan || !matcher_) {
    return;
  }

  int curr_idx = current_node.index;
  int min_idx = std::max(0, curr_idx - near_link_buffer_size_);
  int max_idx = curr_idx - near_link_min_index_diff_;

  struct LinkCandidate {
    int index;
    double distance;
  };
  std::vector<LinkCandidate> candidates;

  for (int j = min_idx; j <= max_idx; ++j) {
    if (j < 0 || j >= static_cast<int>(nodes_.size())) {
      continue;
    }
    const auto& cand_node = nodes_[j];
    if (!cand_node.scan) {
      continue;
    }
    double dist = std::hypot(current_node.x - cand_node.x, current_node.y - cand_node.y);
    if (dist <= near_link_max_distance_) {
      candidates.push_back({j, dist});
    }
  }

  if (candidates.empty()) {
    return;
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const LinkCandidate& a, const LinkCandidate& b) {
              return a.distance < b.distance;
            });

  int added_count = 0;
  for (const auto& cand : candidates) {
    if (added_count >= near_link_max_links_per_node_) {
      break;
    }
    const auto& cand_node = nodes_[cand.index];

    std::vector<Eigen::Vector2d> target_pts;
    std::vector<Eigen::Vector2d> target_normals;
    if (cand_node.normals && !cand_node.normals->empty()) {
      target_pts = core::scan_to_points(cand_node.scan);
      target_normals = *(cand_node.normals);
    } else {
      auto pair = core::scan_to_points_and_normals(cand_node.scan);
      target_pts = std::move(pair.first);
      target_normals = std::move(pair.second);
    }

    if (target_pts.empty() || target_normals.empty()) {
      continue;
    }

    double dx_w = current_node.x - cand_node.x;
    double dy_w = current_node.y - cand_node.y;
    auto [dx_l, dy_l] = core::world_delta_to_local(dx_w, dy_w, cand_node.yaw);
    double dyaw_l = core::angle_diff(current_node.yaw, cand_node.yaw);

    core::OdomData initial_guess{
        current_node.scan->timestamp,
        dx_l,
        dy_l,
        dyaw_l,
    };

    core::MatchResult result;
    bool fast_success = false;

    // Level 1: 高速局所探索 (Point-to-Line ICP 直接実行: 1〜2ms)
    auto csm_matcher = std::dynamic_pointer_cast<scan_matching::MultiResCSMMatcher>(matcher_);
    std::shared_ptr<scan_matching::ICPMatcher> fine_matcher =
        csm_matcher ? csm_matcher->fine_matcher() : nullptr;

    if (fine_matcher) {
      fine_matcher->set_target_cloud_with_normals(target_pts, target_normals);
      auto fast_res = fine_matcher->match(current_node.scan, initial_guess);

      if (fast_res.converged) {
        double trans_drift = std::hypot(fast_res.dx - initial_guess.x, fast_res.dy - initial_guess.y);
        double rot_drift = std::abs(core::angle_diff(fast_res.dyaw, initial_guess.yaw));

        bool has_strong_constraint =
            (fast_res.information(1, 1) >= near_link_min_eigenvalue_ ||
             fast_res.information(2, 2) >= near_link_min_eigenvalue_);

        // 局所収束ゲート判定 (変位 0.15m以内、角度 5.0度以内、スコア良好、十分な幾何拘束)
        if (trans_drift <= 0.15 && rot_drift <= (5.0 * M_PI / 180.0) &&
            fast_res.score >= 0.25 && has_strong_constraint) {
          result = fast_res;
          fast_success = true;
        }
      }
    }

    // Level 2: 大域探索フォールバック (オドメトリ大ズレ・Level 1 不合格時のみ発動)
    if (!fast_success) {
      matcher_->set_target_cloud_with_normals(target_pts, target_normals);
      result = matcher_->match(current_node.scan, initial_guess);
    }

    if (!result.converged) {
      continue;
    }

    double trans_drift = std::hypot(result.dx - initial_guess.x, result.dy - initial_guess.y);
    if (trans_drift > near_link_max_translation_drift_) {
      continue;
    }
    double rot_drift = std::abs(core::angle_diff(result.dyaw, initial_guess.yaw));
    if (rot_drift > near_link_max_rotation_drift_rad_) {
      continue;
    }

    // 異方性判定: 直線路で前後(x)拘束が弱くても、横(y)または回転(yaw)の拘束が十分であれば採用
    double info_yy = result.information(1, 1);
    double info_tt = result.information(2, 2);
    if (info_yy < near_link_min_eigenvalue_ && info_tt < near_link_min_eigenvalue_) {
      continue;
    }

    // 前後方向(x)が極小値の場合も正定値を保証
    result.information(0, 0) = std::max(result.information(0, 0), 10.0);

    edges_.emplace_back(core::PoseEdge{
        cand_node.index,
        current_node.index,
        result.dx,
        result.dy,
        result.dyaw,
        result.information,
        result.score,
        false,
    });
    near_link_success_count_++;
    added_count++;

    RCLCPP_DEBUG(
        rclcpp::get_logger("slam_gnss_2d.scan_matching_builder"),
        "Near-keyframe mesh link added: %d -> %d (dx=%.3f, dy=%.3f, dyaw=%.1f deg, score=%.4f)",
        cand_node.index, current_node.index, result.dx, result.dy,
        result.dyaw * 180.0 / M_PI, result.score);
  }
}

}  // namespace pose_graph
}  // namespace slam_gnss_2d
