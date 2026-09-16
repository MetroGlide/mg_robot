#include "slam_gnss_2d/scan_matching/reference_provider/local_map.hpp"

#include <cmath>
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
  nodes_.clear();
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
