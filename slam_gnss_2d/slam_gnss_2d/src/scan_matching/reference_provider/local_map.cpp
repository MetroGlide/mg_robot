#include "slam_gnss_2d/scan_matching/reference_provider/local_map.hpp"

#include <cmath>
#include <unordered_map>
#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

LocalMapProvider::LocalMapProvider(int window, double radius)
    : window_(window), radius_(radius) {}

void LocalMapProvider::update(const core::PoseNode& node) {
  if (node.scan) {
    if (static_cast<int>(nodes_.size()) >= window_) {
      nodes_.pop_front();
    }
    nodes_.emplace_back(node, core::scan_to_points(node.scan));
  }
  last_node_ = node;
}

void LocalMapProvider::invalidate_cache() {
  // ポーズ更新時も局所参照点群の連続性を維持するため全消去は行わない
}

void LocalMapProvider::sync_poses(const std::vector<core::PoseNode>& nodes) {
  if (nodes.empty() || nodes_.empty()) {
    return;
  }

  bool is_sequential = (!nodes.empty() && nodes.front().index == 0 &&
                        nodes.back().index == static_cast<int>(nodes.size() - 1));

  if (is_sequential) {
    for (auto& pair : nodes_) {
      int idx = pair.first.index;
      if (idx >= 0 && idx < static_cast<int>(nodes.size())) {
        pair.first.x = nodes[idx].x;
        pair.first.y = nodes[idx].y;
        pair.first.yaw = nodes[idx].yaw;
      }
    }
    if (last_node_.has_value()) {
      int idx = last_node_->index;
      if (idx >= 0 && idx < static_cast<int>(nodes.size())) {
        last_node_ = nodes[idx];
      }
    }
  } else {
    std::unordered_map<int, const core::PoseNode*> node_map;
    for (const auto& n : nodes) {
      node_map[n.index] = &n;
    }
    for (auto& pair : nodes_) {
      auto it = node_map.find(pair.first.index);
      if (it != node_map.end()) {
        pair.first.x = it->second->x;
        pair.first.y = it->second->y;
        pair.first.yaw = it->second->yaw;
      }
    }
    if (last_node_.has_value()) {
      auto it = node_map.find(last_node_->index);
      if (it != node_map.end()) {
        last_node_ = *(it->second);
      }
    }
  }
}

std::optional<std::vector<Eigen::Vector2d>> LocalMapProvider::get_reference_pts() {
  if (nodes_.empty() || !last_node_.has_value()) {
    return std::nullopt;
  }

  const auto& last = *last_node_;
  std::vector<Eigen::Vector2d> world_pts;

  double radius_sq = radius_ * radius_;
  for (const auto& pair : nodes_) {
    const auto& n = pair.first;
    const auto& local_pts = pair.second;
    auto w_pts = core::points_local_to_world(local_pts, n.x, n.y, n.yaw);
    for (const auto& pt : w_pts) {
      double dx = pt.x() - last.x;
      double dy = pt.y() - last.y;
      if (dx * dx + dy * dy <= radius_sq) {
        world_pts.push_back(pt);
      }
    }
  }

  if (world_pts.empty()) {
    return std::nullopt;
  }

  return core::points_world_to_local(world_pts, last.x, last.y, last.yaw);
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
