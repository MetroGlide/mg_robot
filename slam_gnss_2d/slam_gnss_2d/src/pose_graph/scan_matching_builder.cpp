#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/scan_matching/multi_res_csm_matcher.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

namespace {

double elapsed_sec(std::chrono::steady_clock::time_point since) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}


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
    double near_link_min_eigenvalue,
    core::OdomFusionConfig odom_fusion)
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
      near_link_min_eigenvalue_(near_link_min_eigenvalue),
      odom_fusion_(odom_fusion) {}

void ScanMatchingBuilder::fuse_with_odometry(
    double odom_dx, double odom_dy, double odom_dyaw,
    double& dx, double& dy, double& dyaw,
    Eigen::Matrix3d& information) const {
  // 軸ごとのガウス積: 情報量は加算し、平均は情報量で重み付けする。
  // マッチングの拘束が弱い (情報量が小さい) 軸ほどオドメトリの寄与が大きくなる。
  const double odom_info[3] = {
      odom_fusion_.information_x, odom_fusion_.information_y, odom_fusion_.information_yaw};
  const double odom_value[3] = {odom_dx, odom_dy, odom_dyaw};
  double value[3] = {dx, dy, dyaw};
  for (int i = 0; i < 3; ++i) {
    if (odom_info[i] <= 0.0) {
      continue;
    }
    const double match_info = information(i, i);
    const double total = match_info + odom_info[i];
    // 回転は角度差で扱い、±π をまたいでも正しく融合する
    const double delta = (i == 2) ? core::angle_diff(odom_value[i], value[i])
                                  : odom_value[i] - value[i];
    value[i] += odom_info[i] / total * delta;
    information(i, i) = total;
  }
  dx = value[0];
  dy = value[1];
  dyaw = (odom_fusion_.information_yaw > 0.0) ? core::normalize_angle(value[2]) : dyaw;
}

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

  auto t_ref = std::chrono::steady_clock::now();
  auto ref_res = provider_->get_reference_pts_and_normals();
  stage_times_.reference_sec += elapsed_sec(t_ref);
  double dx_icp = dx_local;
  double dy_icp = dy_local;
  double dyaw_icp = dyaw_delta;
  Eigen::Matrix3d edge_info;
  bool is_odom_fallback = false;
  double score = 0.0;

  if (ref_res.has_value()) {
    icp_attempt_count_++;
    auto t_set = std::chrono::steady_clock::now();
    matcher_->set_target_cloud_with_normals(ref_res->first, ref_res->second);
    stage_times_.set_target_sec += elapsed_sec(t_set);
    auto t_match = std::chrono::steady_clock::now();
    auto result = matcher_->match(scan, initial_guess);
    stage_times_.match_sec += elapsed_sec(t_match);

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

      fuse_with_odometry(
          dx_local, dy_local, dyaw_delta, dx_icp, dy_icp, dyaw_icp, edge_info);
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
    auto t_near = std::chrono::steady_clock::now();
    add_near_keyframe_links(node);
    stage_times_.near_link_sec += elapsed_sec(t_near);
  }

  last_odom_ = odom;
  return node;
}

std::string ScanMatchingBuilder::timing_summary() const {
  char buf[384];
  std::snprintf(
      buf, sizeof(buf),
      "scan_matching: reference=%.2fs set_target=%.2fs match=%.2fs near_link=%.2fs "
      "(target=%.2fs match=%.2fs) (attempts=%d, success=%d, odom_fallback=%d, near_link_added=%d)",
      stage_times_.reference_sec, stage_times_.set_target_sec, stage_times_.match_sec,
      stage_times_.near_link_sec, stage_times_.near_target_sec, stage_times_.near_match_sec,
      icp_attempt_count_, icp_success_count_,
      odom_fallback_count_, near_link_success_count_);
  return buf;
}

std::vector<core::PoseNode> ScanMatchingBuilder::get_nodes() const {
  return nodes_;
}

std::vector<core::PoseEdge> ScanMatchingBuilder::get_edges() const {
  return edges_;
}

void ScanMatchingBuilder::reset() {
  if (matcher_) {
    matcher_->clear_reusable_targets();
  }
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

  // 候補スキャンの点群と法線を取得する。空の場合は false
  auto build_target_cloud = [&](const core::PoseNode& cand_node,
                                std::vector<Eigen::Vector2d>& target_pts,
                                std::vector<Eigen::Vector2d>& target_normals) {
    if (cand_node.normals && !cand_node.normals->empty()) {
      target_pts = core::scan_to_points(cand_node.scan);
      target_normals = *(cand_node.normals);
    } else {
      auto pair = core::scan_to_points_and_normals(cand_node.scan);
      target_pts = std::move(pair.first);
      target_normals = std::move(pair.second);
    }
    return !target_pts.empty() && !target_normals.empty();
  };

  // 候補の再利用ターゲットを登録する (登録済みなら何もしない)。空の点群を持つ候補は false
  auto ensure_reusable_target = [&](const core::PoseNode& cand_node) {
    if (matcher_->try_use_reusable_target(cand_node.index)) {
      return true;
    }
    std::vector<Eigen::Vector2d> target_pts;
    std::vector<Eigen::Vector2d> target_normals;
    if (!build_target_cloud(cand_node, target_pts, target_normals)) {
      return false;
    }
    matcher_->set_reusable_target_cloud_with_normals(cand_node.index, target_pts, target_normals);
    return true;
  };

  // 候補は距離順に、必要本数ぶんずつまとめて (並列に) マッチングし、結果は距離順に採否判定する。
  // 採否判定の順序と打ち切りは逐次実行と同じなので、採用されるリンクは変わらない。
  // 再利用ターゲットに対応しないマッチャは 1 候補ずつ通常のターゲット設定で逐次マッチングする。
  const bool reusable = matcher_->supports_reusable_targets();
  constexpr size_t kMaxBatch = 6;  // マッチャの再利用ターゲットキャッシュに収まる数
  int added_count = 0;
  size_t next = 0;
  while (added_count < near_link_max_links_per_node_ && next < candidates.size()) {
    const size_t want = static_cast<size_t>(near_link_max_links_per_node_ - added_count);
    const size_t batch_size = reusable
        ? std::min({want, kMaxBatch, candidates.size() - next})
        : 1;

    auto t_target = std::chrono::steady_clock::now();
    std::vector<size_t> batch;
    std::vector<scan_matching::ReusableMatchRequest> requests;
    std::vector<core::MatchResult> results;
    for (size_t k = next; k < next + batch_size; ++k) {
      const auto& cand_node = nodes_[candidates[k].index];
      std::vector<Eigen::Vector2d> target_pts;
      std::vector<Eigen::Vector2d> target_normals;
      if (reusable) {
        if (!ensure_reusable_target(cand_node)) {
          continue;
        }
      } else {
        if (!build_target_cloud(cand_node, target_pts, target_normals)) {
          continue;
        }
        matcher_->set_target_cloud_with_normals(target_pts, target_normals);
      }
      double dx_w = current_node.x - cand_node.x;
      double dy_w = current_node.y - cand_node.y;
      auto [dx_l, dy_l] = core::world_delta_to_local(dx_w, dy_w, cand_node.yaw);
      double dyaw_l = core::angle_diff(current_node.yaw, cand_node.yaw);
      batch.push_back(k);
      requests.push_back({
          cand_node.index,
          core::OdomData{current_node.scan->timestamp, dx_l, dy_l, dyaw_l},
      });
    }
    stage_times_.near_target_sec += elapsed_sec(t_target);
    next += batch_size;

    auto t_near_match = std::chrono::steady_clock::now();
    if (reusable) {
      results = matcher_->match_reusable_targets(current_node.scan, requests);
    } else if (!requests.empty()) {
      results.push_back(matcher_->match(current_node.scan, requests.front().initial_guess));
    }
    stage_times_.near_match_sec += elapsed_sec(t_near_match);

    for (size_t r = 0; r < batch.size() && added_count < near_link_max_links_per_node_; ++r) {
      const auto& cand_node = nodes_[candidates[batch[r]].index];
      const auto& initial_guess = requests[r].initial_guess;
      core::MatchResult result = results[r];

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
}

}  // namespace pose_graph
}  // namespace slam_gnss_2d
