#pragma once

#include <map>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/scan_matching/base.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

struct NdtCell {
  Eigen::Vector2d mean;
  Eigen::Matrix2d sigma_inv;
};

class NDTMatcher : public ScanMatcherBase {
 public:
  NDTMatcher(
      int max_iterations,
      double tolerance,
      const std::vector<double>& cell_sizes,
      bool use_bilinear,
      double yaw_information_multiplier);

  void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) override;

  core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) override;

 private:
  int max_iterations_;
  double tolerance_;
  std::vector<double> cell_sizes_;
  bool use_bilinear_;
  double yaw_information_multiplier_;

  std::vector<Eigen::Vector2d> src_pts_;

  struct PyramidLevel {
    double cell_size;
    std::unordered_map<int64_t, NdtCell> cells;
  };
  std::vector<PyramidLevel> pyramids_;

  core::MatchResult match_single_resolution(
      const std::vector<Eigen::Vector2d>& dst_pts,
      double init_tx, double init_ty, double init_theta,
      const PyramidLevel& level);
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
