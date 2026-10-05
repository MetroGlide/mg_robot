#include "slam_gnss_2d/pose_graph/odom_builder.hpp"

#include <cmath>
#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

namespace {
Eigen::Matrix3d make_odom_information() {
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
  info(0, 0) = 100.0;
  info(1, 1) = 100.0;
  info(2, 2) = 50.0;
  return info;
}
}  // namespace

OdomOnlyBuilder::OdomOnlyBuilder(double min_translation, double min_rotation)
    : min_translation_(min_translation), min_rotation_(min_rotation) {}

std::optional<core::PoseNode> OdomOnlyBuilder::add_scan(
    const core::ScanDataPtr& scan,
    const core::OdomData& odom) {
  if (!scan) {
    return std::nullopt;
  }

  if (nodes_.empty()) {
    core::PoseNode node{
        0,
        scan->timestamp,
        odom.x,
        odom.y,
        odom.yaw,
        scan,
    };
    nodes_.push_back(node);
    last_odom_ = odom;
    return node;
  }

  if (!last_odom_.has_value()) {
    return std::nullopt;
  }

  double dx_w = odom.x - last_odom_->x;
  double dy_w = odom.y - last_odom_->y;
  double dist = std::hypot(dx_w, dy_w);
  double dyaw = std::abs(core::angle_diff(odom.yaw, last_odom_->yaw));
  if (dist < min_translation_ && dyaw < min_rotation_) {
    return std::nullopt;
  }

  const auto prev_node = nodes_.back();

  auto [dx_local, dy_local] = core::world_delta_to_local(
      dx_w, dy_w, last_odom_->yaw);
  double dyaw_delta = core::angle_diff(odom.yaw, last_odom_->yaw);

  auto [dx_world, dy_world] = core::local_delta_to_world(
      dx_local, dy_local, prev_node.yaw);
  double new_x = prev_node.x + dx_world;
  double new_y = prev_node.y + dy_world;
  double new_yaw = core::normalize_angle(prev_node.yaw + dyaw_delta);

  core::PoseNode node{
      static_cast<int>(nodes_.size()),
      scan->timestamp,
      new_x,
      new_y,
      new_yaw,
      scan,
  };
  nodes_.push_back(node);

  edges_.emplace_back(core::PoseEdge{
      prev_node.index,
      node.index,
      dx_local,
      dy_local,
      dyaw_delta,
      make_odom_information(),
      0.0,
      false,
  });

  last_odom_ = odom;
  return node;
}

std::vector<core::PoseNode> OdomOnlyBuilder::get_nodes() const {
  return nodes_;
}

std::vector<core::PoseEdge> OdomOnlyBuilder::get_edges() const {
  return edges_;
}

void OdomOnlyBuilder::reset() {
  nodes_.clear();
  edges_.clear();
  last_odom_.reset();
}

void OdomOnlyBuilder::replace_nodes(const std::vector<core::PoseNode>& nodes) {
  nodes_ = nodes;
}

}  // namespace pose_graph
}  // namespace slam_gnss_2d
