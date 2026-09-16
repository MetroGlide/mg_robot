#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/scan_matching/base.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class CSMMatcher : public ScanMatcherBase {
 public:
  CSMMatcher(
      double linear_search_window,
      double angular_search_window,
      double linear_step,
      double angular_step,
      double yaw_information_multiplier);

  ~CSMMatcher() override;

  void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) override;

  core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) override;

 private:
  double linear_search_window_;
  double angular_search_window_;
  double linear_step_;
  double angular_step_;
  double yaw_information_multiplier_;

  std::vector<Eigen::Vector2d> src_pts_;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
