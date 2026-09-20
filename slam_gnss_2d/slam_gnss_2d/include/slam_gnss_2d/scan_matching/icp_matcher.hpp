#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/scan_matching/base.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class ICPMatcher : public ScanMatcherBase {
 public:
  ICPMatcher(
      int max_iterations,
      double tolerance,
      double max_correspondence_dist,
      const std::string& robust_kernel,
      double robust_kernel_scale,
      double yaw_information_multiplier,
      double motion_prior_weight_x = 10.0,
      double motion_prior_weight_y = 500.0,
      double motion_prior_weight_yaw = 100.0);

  ~ICPMatcher() override;

  void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) override;

  core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) override;

 private:
  int max_iterations_;
  double tolerance_;
  double max_correspondence_dist_;
  std::string robust_kernel_;
  double robust_kernel_scale_;
  double yaw_information_multiplier_;
  double motion_prior_weight_x_;
  double motion_prior_weight_y_;
  double motion_prior_weight_yaw_;

  std::vector<Eigen::Vector2d> src_pts_;
  std::vector<Eigen::Vector2d> src_normals_;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
