#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/scan_matching/base.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class MultiStartICPMatcher : public ScanMatcherBase {
 public:
  MultiStartICPMatcher(
      int max_iterations,
      double tolerance,
      double max_correspondence_dist,
      const std::string& robust_kernel,
      double robust_kernel_scale,
      double yaw_information_multiplier,
      const std::vector<double>& angle_offsets_deg = {-4.0, -2.0, 2.0, 4.0},
      bool enable_straight_hypothesis = true)
      : base_matcher_(std::make_unique<ICPMatcher>(
            max_iterations,
            tolerance,
            max_correspondence_dist,
            robust_kernel,
            robust_kernel_scale,
            yaw_information_multiplier)),
        enable_straight_hypothesis_(enable_straight_hypothesis) {
    for (double deg : angle_offsets_deg) {
      angle_offsets_rad_.push_back(deg * M_PI / 180.0);
    }
  }

  ~MultiStartICPMatcher() override = default;

  void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) override {
    base_matcher_->set_target_cloud(src_pts);
  }

  core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) override {
    if (!base_matcher_) {
      return core::MatchResult{
          initial_guess.x, initial_guess.y, initial_guess.yaw,
          false, Eigen::Matrix3d::Zero(), 0.0};
    }

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
    if (has_last_yaw_) {
      core::OdomData h_const_vel = initial_guess;
      h_const_vel.yaw = last_matched_dyaw_;
      hypotheses.push_back(h_const_vel);
    }

    // 4. 角度オフセット仮説
    for (double offset : angle_offsets_rad_) {
      core::OdomData h = initial_guess;
      h.yaw = core::normalize_angle(initial_guess.yaw + offset);
      hypotheses.push_back(h);
    }

    core::MatchResult best_res;
    best_res.converged = false;
    double best_score = std::numeric_limits<double>::max();

    for (const auto& h : hypotheses) {
      auto res = base_matcher_->match(dst, h);
      if (res.converged) {
        if (res.score < best_score) {
          best_score = res.score;
          best_res = res;
        }
      }
    }

    if (best_res.converged) {
      last_matched_dyaw_ = best_res.dyaw;
      has_last_yaw_ = true;
      return best_res;
    }

    // すべて非収束の場合はオドメトリ初期値での結果をフォールバックとして返す
    return base_matcher_->match(dst, initial_guess);
  }

 private:
  std::unique_ptr<ICPMatcher> base_matcher_;
  std::vector<double> angle_offsets_rad_;
  bool enable_straight_hypothesis_{true};
  double last_matched_dyaw_{0.0};
  bool has_last_yaw_{false};
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
