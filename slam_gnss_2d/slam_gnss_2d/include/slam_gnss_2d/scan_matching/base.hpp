#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class ScanMatcherBase {
 public:
  virtual ~ScanMatcherBase() = default;

  virtual void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) = 0;
  virtual void set_target_cloud_with_normals(
      const std::vector<Eigen::Vector2d>& src_pts,
      const std::vector<Eigen::Vector2d>& src_normals) {
    (void)src_normals;
    set_target_cloud(src_pts);
  }

  virtual core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) = 0;

  core::MatchResult match(
      const core::ScanDataPtr& dst,
      const core::OdomData& initial_guess) {
    return match(std::static_pointer_cast<const core::ScanData>(dst), initial_guess);
  }
};

using ScanMatcherPtr = std::shared_ptr<ScanMatcherBase>;

}  // namespace scan_matching
}  // namespace slam_gnss_2d
