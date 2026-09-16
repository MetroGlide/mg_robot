#include "slam_gnss_2d/tools/reoptimize_geometry.hpp"

#include <cmath>
#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace tools {

std::optional<std::vector<Eigen::Vector2d>> build_submap_points(
    int center_node_idx,
    const std::vector<core::PoseNode>& nodes,
    const std::unordered_map<int, core::ScanDataPtr>& node_scans,
    double radius) {
  if (center_node_idx < 0 || center_node_idx >= static_cast<int>(nodes.size())) {
    return std::nullopt;
  }

  const auto& center_node = nodes[center_node_idx];
  std::vector<Eigen::Vector2d> world_pts;
  double radius_sq = radius * radius;

  for (const auto& node : nodes) {
    auto it = node_scans.find(node.index);
    if (it != node_scans.end() && it->second) {
      double dx = node.x - center_node.x;
      double dy = node.y - center_node.y;
      if (dx * dx + dy * dy <= radius_sq) {
        auto local_pts = core::scan_to_points(it->second);
        auto w_pts = core::points_local_to_world(local_pts, node.x, node.y, node.yaw);
        world_pts.insert(world_pts.end(), w_pts.begin(), w_pts.end());
      }
    }
  }

  if (world_pts.empty()) {
    return std::nullopt;
  }

  return core::points_world_to_local(
      world_pts, center_node.x, center_node.y, center_node.yaw);
}

core::ScanDataPtr find_nearest_scan(
    const std::vector<core::ScanDataPtr>& scans,
    double timestamp,
    double max_diff) {
  if (scans.empty()) {
    return nullptr;
  }

  core::ScanDataPtr best = nullptr;
  double min_dt = max_diff + 1.0;

  for (const auto& scan : scans) {
    if (!scan) continue;
    double dt = std::abs(scan->timestamp - timestamp);
    if (dt < min_dt) {
      min_dt = dt;
      best = scan;
    }
  }

  if (min_dt <= max_diff) {
    return best;
  }
  return nullptr;
}

double sigma_from_covariance_or_status(
    const core::GnssData& gnss,
    const core::SlamConfig& config) {
  double cov_xx = gnss.covariance(0, 0);
  if (cov_xx > 0.0) {
    return std::sqrt(cov_xx);
  }
  if (gnss.fix_status >= 2) {
    return config.gnss.sigma.fix_m;
  }
  if (gnss.fix_status >= 0) {
    return config.gnss.sigma.float_m;
  }
  return -1.0;
}

}  // namespace tools
}  // namespace slam_gnss_2d
