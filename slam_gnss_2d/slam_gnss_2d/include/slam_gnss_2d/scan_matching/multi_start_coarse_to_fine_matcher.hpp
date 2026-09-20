#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/scan_matching/base.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"
#include "slam_gnss_2d/scan_matching/ndt_matcher.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class MultiStartCoarseToFineMatcher : public ScanMatcherBase {
 public:
  MultiStartCoarseToFineMatcher(
      std::shared_ptr<NDTMatcher> coarse_matcher,
      std::shared_ptr<ICPMatcher> fine_matcher,
      double angular_search_window_deg = 20.0,
      double angular_step_deg = 2.5,
      bool enable_straight_hypothesis = true,
      bool enable_const_vel_hypothesis = true)
      : coarse_matcher_(std::move(coarse_matcher)),
        fine_matcher_(std::move(fine_matcher)),
        angular_search_window_rad_(angular_search_window_deg * M_PI / 180.0),
        angular_step_rad_(std::max(0.1 * M_PI / 180.0, angular_step_deg * M_PI / 180.0)),
        enable_straight_hypothesis_(enable_straight_hypothesis),
        enable_const_vel_hypothesis_(enable_const_vel_hypothesis) {}

  ~MultiStartCoarseToFineMatcher() override = default;

  void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) override {
    if (coarse_matcher_) coarse_matcher_->set_target_cloud(src_pts);
    if (fine_matcher_) fine_matcher_->set_target_cloud(src_pts);
  }

  void set_target_cloud_with_normals(
      const std::vector<Eigen::Vector2d>& src_pts,
      const std::vector<Eigen::Vector2d>& src_normals) override {
    if (coarse_matcher_) coarse_matcher_->set_target_cloud(src_pts);
    if (fine_matcher_) fine_matcher_->set_target_cloud_with_normals(src_pts, src_normals);
  }

  core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) override {
    if (!coarse_matcher_ && !fine_matcher_) {
      return core::MatchResult{
          initial_guess.x, initial_guess.y, initial_guess.yaw,
          false, Eigen::Matrix3d::Zero(), 0.0};
    }
    if (!coarse_matcher_) {
      return fine_matcher_->match(dst, initial_guess);
    }

    // Phase 1: 平常時の直接精密マッチング (Fine-first fast tracking)
    // 連続トラッキング時は変位が極小なため、まずは運動正則化ICPで直接吸着を試みる。
    // これにより、平常時の不要なNDT角度探索による微小な角度跳ねを完全根絶する。
    if (fine_matcher_) {
      auto direct_res = fine_matcher_->match(dst, initial_guess);
      if (direct_res.converged && direct_res.score < 0.15) {
        last_matched_dyaw_ = direct_res.dyaw;
        has_last_yaw_ = true;
        return direct_res;
      }
    }

    // Phase 2: セーフティネット (大スリップ・急激な旋回時のNDTマルチスタート粗探索)
    // 直接ICPが非収束、またはスコアが悪化した場合のみNDT大域探索を発動
    std::vector<core::OdomData> hypotheses;

    // 1. オドメトリ仮説
    hypotheses.push_back(initial_guess);

    // 2. 直進仮説 (dyaw = 0.0)
    if (enable_straight_hypothesis_) {
      core::OdomData h_straight = initial_guess;
      h_straight.yaw = 0.0;
      hypotheses.push_back(h_straight);
    }

    // 3. 一定角速度仮説
    if (enable_const_vel_hypothesis_ && has_last_yaw_) {
      core::OdomData h_const_vel = initial_guess;
      h_const_vel.yaw = last_matched_dyaw_;
      hypotheses.push_back(h_const_vel);
    }

    // 4. 回転グリッド探索仮説
    for (double offset = -angular_search_window_rad_;
         offset <= angular_search_window_rad_ + 1e-6;
         offset += angular_step_rad_) {
      if (std::abs(offset) < 1e-4) continue;
      core::OdomData h = initial_guess;
      h.yaw = core::normalize_angle(initial_guess.yaw + offset);
      hypotheses.push_back(h);
    }

    core::MatchResult best_coarse_res;
    best_coarse_res.converged = false;
    double best_coarse_score = std::numeric_limits<double>::max();

    for (const auto& h : hypotheses) {
      auto res = coarse_matcher_->match(dst, h);
      if (res.converged) {
        if (res.score < best_coarse_score) {
          best_coarse_score = res.score;
          best_coarse_res = res;
        }
      }
    }

    core::OdomData fine_seed = initial_guess;
    if (best_coarse_res.converged) {
      fine_seed.x = best_coarse_res.dx;
      fine_seed.y = best_coarse_res.dy;
      fine_seed.yaw = best_coarse_res.dyaw;
    }

    // NDT最良解からの精密壁面吸着
    if (fine_matcher_) {
      auto fine_res = fine_matcher_->match(dst, fine_seed);
      if (fine_res.converged) {
        last_matched_dyaw_ = fine_res.dyaw;
        has_last_yaw_ = true;
        return fine_res;
      }
    }

    // ICP非収束時はNDT最良解を採用
    if (best_coarse_res.converged) {
      last_matched_dyaw_ = best_coarse_res.dyaw;
      has_last_yaw_ = true;
      return best_coarse_res;
    }

    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }

 private:
  std::shared_ptr<NDTMatcher> coarse_matcher_;
  std::shared_ptr<ICPMatcher> fine_matcher_;
  double angular_search_window_rad_{20.0 * M_PI / 180.0};
  double angular_step_rad_{2.5 * M_PI / 180.0};
  bool enable_straight_hypothesis_{true};
  bool enable_const_vel_hypothesis_{true};
  double last_matched_dyaw_{0.0};
  bool has_last_yaw_{false};
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
