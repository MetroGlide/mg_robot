#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/scan_matching/base.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class MultiResCSMMatcher : public ScanMatcherBase {
 public:
  MultiResCSMMatcher(
      std::shared_ptr<ICPMatcher> fine_matcher = nullptr,
      double linear_search_window = 0.4,
      double angular_search_window_deg = 15.0,
      double linear_step = 0.02,
      double angular_step_deg = 0.5,
      double grid_resolution = 0.03,
      double score_threshold = 0.1,
      bool enable_variance_penalty = true,
      double distance_variance_penalty = 0.5,
      double angle_variance_penalty = 1.0,
      double minimum_distance_penalty = 0.5,
      double minimum_angle_penalty = 0.9,
      int num_threads = 0);

  ~MultiResCSMMatcher() override;

  void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) override;

  void set_target_cloud_with_normals(
      const std::vector<Eigen::Vector2d>& src_pts,
      const std::vector<Eigen::Vector2d>& src_normals) override;

  bool supports_reusable_targets() const override { return true; }
  bool try_use_reusable_target(int key) override;
  void set_reusable_target_cloud_with_normals(
      int key,
      const std::vector<Eigen::Vector2d>& src_pts,
      const std::vector<Eigen::Vector2d>& src_normals) override;
  void clear_reusable_targets() override;
  std::vector<core::MatchResult> match_reusable_targets(
      const core::ConstScanDataPtr& dst,
      const std::vector<ReusableMatchRequest>& requests) override;

  core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) override;

  core::MatchResult match(
      const std::vector<Eigen::Vector2d>& dst_pts,
      const core::OdomData& initial_guess);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
