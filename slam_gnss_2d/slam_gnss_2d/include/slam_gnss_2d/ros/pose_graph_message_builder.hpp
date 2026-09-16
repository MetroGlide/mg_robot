#pragma once

#include <vector>
#include <slam_gnss_2d_msgs/msg/pose_graph_diff.hpp>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace ros {

std::pair<std::vector<core::PoseEdge>, std::vector<core::PoseEdge>> split_edges(
    const std::vector<core::PoseEdge>& all_edges,
    const std::vector<core::PoseEdge>& loop_edges);

slam_gnss_2d_msgs::msg::PoseGraphDiff build_pose_graph_diff(
    const std::vector<core::PoseNode>& nodes,
    const std::vector<core::PoseEdge>& seq_edges = {},
    const std::vector<core::PoseEdge>& loop_edges = {},
    const std::vector<core::GnssPrior>& priors = {},
    bool loop_closed = false,
    bool full_refresh_needed = false);

}  // namespace ros
}  // namespace slam_gnss_2d
