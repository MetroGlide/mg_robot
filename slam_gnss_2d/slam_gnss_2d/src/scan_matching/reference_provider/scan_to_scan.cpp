#include "slam_gnss_2d/scan_matching/reference_provider/scan_to_scan.hpp"

#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

void ScanToScanProvider::update(const core::PoseNode& node) {
  if (node.scan) {
    last_pts_ = core::scan_to_points(node.scan);
  }
}

std::optional<std::vector<Eigen::Vector2d>> ScanToScanProvider::get_reference_pts() {
  return last_pts_;
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
