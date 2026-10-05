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
    std::vector<Eigen::Vector2d> pts;
    std::vector<Eigen::Vector2d> normals;
    if (node.normals && !node.normals->empty()) {
      pts = core::scan_to_points(node.scan);
      normals = *(node.normals);
    } else {
      auto pair = core::scan_to_points_and_normals(node.scan);
      pts = std::move(pair.first);
      normals = std::move(pair.second);
    }
    nodes_.push_back(CachedKeyframe{node, std::move(pts), std::move(normals)});
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
    for (auto& kf : nodes_) {
      int idx = kf.node.index;
      if (idx >= 0 && idx < static_cast<int>(nodes.size())) {
        kf.node.x = nodes[idx].x;
        kf.node.y = nodes[idx].y;
        kf.node.yaw = nodes[idx].yaw;
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
    for (auto& kf : nodes_) {
      auto it = node_map.find(kf.node.index);
      if (it != node_map.end()) {
        kf.node.x = it->second->x;
        kf.node.y = it->second->y;
        kf.node.yaw = it->second->yaw;
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
  auto res = get_reference_pts_and_normals();
  if (!res.has_value()) {
    return std::nullopt;
  }
  return res->first;
}

std::optional<std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>>
LocalMapProvider::get_reference_pts_and_normals() {
  if (nodes_.empty() || !last_node_.has_value()) {
    return std::nullopt;
  }

  const auto& last = *last_node_;
  std::vector<Eigen::Vector2d> world_pts;
  std::vector<Eigen::Vector2d> world_normals;

  double radius_sq = radius_ * radius_;
  for (const auto& kf : nodes_) {
    const auto& n = kf.node;
    auto w_pts = core::points_local_to_world(kf.pts, n.x, n.y, n.yaw);
    auto w_normals = core::normals_local_to_world(kf.normals, n.yaw);

    for (size_t i = 0; i < w_pts.size(); ++i) {
      double dx = w_pts[i].x() - last.x;
      double dy = w_pts[i].y() - last.y;
      if (dx * dx + dy * dy <= radius_sq) {
        world_pts.push_back(w_pts[i]);
        if (i < w_normals.size()) {
          world_normals.push_back(w_normals[i]);
        } else {
          world_normals.push_back(Eigen::Vector2d(0.0, 1.0));
        }
      }
    }
  }

  if (world_pts.empty()) {
    return std::nullopt;
  }

  auto local_pts = core::points_world_to_local(world_pts, last.x, last.y, last.yaw);
  auto local_normals = core::normals_world_to_local(world_normals, last.yaw);
  return std::make_pair(std::move(local_pts), std::move(local_normals));
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
