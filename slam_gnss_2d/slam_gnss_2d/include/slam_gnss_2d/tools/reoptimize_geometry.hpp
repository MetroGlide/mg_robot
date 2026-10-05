#pragma once

#include <optional>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/config.hpp"
#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace tools {

std::optional<std::vector<Eigen::Vector2d>> build_submap_points(
    int center_node_idx,
    const std::vector<core::PoseNode>& nodes,
    const std::unordered_map<int, core::ScanDataPtr>& node_scans,
    double radius);

core::ScanDataPtr find_nearest_scan(
    const std::vector<core::ScanDataPtr>& scans,
    double timestamp,
    double max_diff);

double sigma_from_covariance_or_status(
    const core::GnssData& gnss,
    const core::SlamConfig& config);

}  // namespace tools
}  // namespace slam_gnss_2d
