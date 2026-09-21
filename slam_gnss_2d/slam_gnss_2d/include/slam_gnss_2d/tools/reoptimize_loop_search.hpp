#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/scan_matching/base.hpp"

namespace slam_gnss_2d {
namespace tools {

std::vector<core::PoseEdge> search_new_loop_edges(
    const std::vector<core::PoseNode>& optimized_nodes,
    const std::unordered_map<int, core::ScanDataPtr>& node_scans,
    const std::vector<core::PoseEdge>& existing_loop_edges,
    scan_matching::ScanMatcherBase& loop_matcher,
    double search_radius,
    int min_node_gap,
    double submap_radius,
    double max_score,
    double max_dyaw_deg,
    double crossing_reject_deg,
    std::function<void(const std::string&)> logger_info);

}  // namespace tools
}  // namespace slam_gnss_2d
