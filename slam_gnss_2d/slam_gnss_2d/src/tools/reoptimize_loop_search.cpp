#include "slam_gnss_2d/tools/reoptimize_loop_search.hpp"

#include <cmath>
#include <set>
#include <utility>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/tools/reoptimize_geometry.hpp"

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
    std::function<void(const std::string&)> logger_info) {
  std::set<std::pair<int, int>> existing_pairs;
  for (const auto& e : existing_loop_edges) {
    existing_pairs.insert({std::min(e.from_index, e.to_index), std::max(e.from_index, e.to_index)});
  }

  std::vector<core::PoseNode> valid_nodes;
  for (const auto& n : optimized_nodes) {
    if (node_scans.find(n.index) != node_scans.end()) {
      valid_nodes.push_back(n);
    }
  }

  if (valid_nodes.empty()) {
    return {};
  }

  double search_radius_sq = search_radius * search_radius;
  double max_loop_dyaw_rad = max_dyaw_deg * M_PI / 180.0;
  double crossing_rad = crossing_reject_deg * M_PI / 180.0;

  std::vector<core::PoseEdge> new_edges;

  for (size_t i = 0; i < valid_nodes.size(); ++i) {
    const auto& node = valid_nodes[i];
    for (size_t j = 0; j < valid_nodes.size(); ++j) {
      if (i == j) continue;
      const auto& candidate = valid_nodes[j];
      if (std::abs(node.index - candidate.index) < min_node_gap) {
        continue;
      }

      auto pair = std::make_pair(
          std::min(node.index, candidate.index),
          std::max(node.index, candidate.index));
      if (existing_pairs.find(pair) != existing_pairs.end()) {
        continue;
      }

      double dx = node.x - candidate.x;
      double dy = node.y - candidate.y;
      if (dx * dx + dy * dy > search_radius_sq) {
        continue;
      }

      std::vector<Eigen::Vector2d> src_pts;
      if (submap_radius > 0.0) {
        auto sub = build_submap_points(
            candidate.index, optimized_nodes, node_scans, submap_radius);
        if (sub.has_value()) {
          src_pts = *sub;
        }
      } else {
        auto it = node_scans.find(candidate.index);
        if (it != node_scans.end() && it->second) {
          src_pts = core::scan_to_points(it->second);
        }
      }

      if (src_pts.empty()) {
        continue;
      }

      auto [dx_local, dy_local] = core::world_delta_to_local(dx, dy, candidate.yaw);
      core::OdomData initial_guess{
          node.timestamp,
          dx_local,
          dy_local,
          core::angle_diff(node.yaw, candidate.yaw),
      };

      loop_matcher.set_target_cloud(src_pts);
      auto node_scan = node_scans.at(node.index);
      auto result = loop_matcher.match(node_scan, initial_guess);

      if (!result.converged) {
        continue;
      }
      if (max_score > 0.0 && result.score > max_score) {
        continue;
      }

      double abs_dyaw = std::abs(result.dyaw);
      if (abs_dyaw > max_loop_dyaw_rad) {
        continue;
      }

      if (crossing_rad > 0.0 && crossing_rad <= abs_dyaw && abs_dyaw <= (M_PI - crossing_rad)) {
        continue;
      }

      core::PoseEdge edge{
          candidate.index,
          node.index,
          result.dx,
          result.dy,
          result.dyaw,
          result.information,
          result.score,
          false,
      };

      new_edges.push_back(edge);
      existing_pairs.insert(pair);

      if (logger_info) {
        char buf[256];
        std::snprintf(
            buf, sizeof(buf),
            "New loop edge found: %d -> %d (score=%.4f)",
            candidate.index, node.index, result.score);
        logger_info(std::string(buf));
      }
    }
  }

  return new_edges;
}

}  // namespace tools
}  // namespace slam_gnss_2d
