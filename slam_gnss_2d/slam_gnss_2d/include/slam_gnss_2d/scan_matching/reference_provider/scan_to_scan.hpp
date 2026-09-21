#pragma once

#include <optional>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/scan_matching/reference_provider/base.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class ScanToScanProvider : public ReferenceProviderBase {
 public:
  ScanToScanProvider() = default;

  void update(const core::PoseNode& node) override;
  std::optional<std::vector<Eigen::Vector2d>> get_reference_pts() override;

 private:
  std::optional<std::vector<Eigen::Vector2d>> last_pts_;
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
