#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/scan_matching/base.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class CoarseToFineMatcher : public ScanMatcherBase {
 public:
  CoarseToFineMatcher(
      ScanMatcherPtr coarse_matcher,
      ScanMatcherPtr fine_matcher)
      : coarse_matcher_(coarse_matcher),
        fine_matcher_(fine_matcher) {}

  ~CoarseToFineMatcher() override = default;

  void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) override {
    if (coarse_matcher_) {
      coarse_matcher_->set_target_cloud(src_pts);
    }
    if (fine_matcher_) {
      fine_matcher_->set_target_cloud(src_pts);
    }
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
    if (!fine_matcher_) {
      return coarse_matcher_->match(dst, initial_guess);
    }

    // 1. Coarse アライメント (NDT 等) で広域引き込み
    auto coarse_res = coarse_matcher_->match(dst, initial_guess);

    core::OdomData fine_guess = initial_guess;
    if (coarse_res.converged) {
      fine_guess.x = coarse_res.dx;
      fine_guess.y = coarse_res.dy;
      fine_guess.yaw = coarse_res.dyaw;
    }

    // 2. Fine アライメント (Point-to-Line ICP) で壁面に精密吸着
    auto fine_res = fine_matcher_->match(dst, fine_guess);

    // Fine が収束した場合は高精度な結果を採用
    if (fine_res.converged) {
      return fine_res;
    }

    // Fine が非収束だが Coarse が収束していた場合は Coarse の結果で救済
    if (coarse_res.converged) {
      return coarse_res;
    }

    return fine_res;
  }

 private:
  ScanMatcherPtr coarse_matcher_;
  ScanMatcherPtr fine_matcher_;
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
