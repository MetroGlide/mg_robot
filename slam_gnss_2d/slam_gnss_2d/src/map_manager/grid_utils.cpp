#include "slam_gnss_2d/map_manager/grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace slam_gnss_2d {
namespace map_manager {

std::tuple<double, double, int, int> compute_bounds(
    const std::vector<core::PoseNode>& nodes,
    double resolution,
    double expansion_margin) {
  if (nodes.empty()) {
    return {0.0, 0.0, 1, 1};
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto& n : nodes) {
    min_x = std::min(min_x, n.x);
    min_y = std::min(min_y, n.y);
    max_x = std::max(max_x, n.x);
    max_y = std::max(max_y, n.y);
  }

  double origin_x = min_x - expansion_margin;
  double origin_y = min_y - expansion_margin;
  double end_x = max_x + expansion_margin;
  double end_y = max_y + expansion_margin;

  int width = static_cast<int>(std::ceil((end_x - origin_x) / resolution));
  int height = static_cast<int>(std::ceil((end_y - origin_y) / resolution));
  width = std::max(1, width);
  height = std::max(1, height);

  return {origin_x, origin_y, width, height};
}

std::tuple<double, double, int> compute_square_bounds(
    const std::vector<core::PoseNode>& nodes,
    double resolution,
    double expansion_margin) {
  auto [ox, oy, w, h] = compute_bounds(nodes, resolution, expansion_margin);
  return {ox, oy, std::max(w, h)};
}

std::pair<int, int> world_to_pixel(
    double wx, double wy,
    double origin_x, double origin_y,
    double resolution) {
  int px = static_cast<int>((wx - origin_x) / resolution);
  int py = static_cast<int>((wy - origin_y) / resolution);
  return {px, py};
}

bool in_bounds(int px, int py, int width, int height) {
  return px >= 0 && px < width && py >= 0 && py < height;
}

bool in_bounds(int px, int py, int map_size) {
  return in_bounds(px, py, map_size, map_size);
}

ScanHitsPixels scan_hits_to_pixels(
    const core::PoseNode& node,
    double origin_x,
    double origin_y,
    double resolution,
    int width,
    int height) {
  ScanHitsPixels result;
  const auto& scan = node.scan;
  double cos_yaw = std::cos(node.yaw);
  double sin_yaw = std::sin(node.yaw);
  double lidar_x = scan ? scan->lidar_x : 0.0;
  double lidar_y = scan ? scan->lidar_y : 0.0;

  double lidar_wx = node.x + cos_yaw * lidar_x - sin_yaw * lidar_y;
  double lidar_wy = node.y + sin_yaw * lidar_x + cos_yaw * lidar_y;
  auto [r_px, r_py] = world_to_pixel(lidar_wx, lidar_wy, origin_x, origin_y, resolution);
  result.robot_px = r_px;
  result.robot_py = r_py;

  if (!scan) {
    return result;
  }

  size_t n_ranges = scan->ranges.size();
  for (size_t i = 0; i < n_ranges; ++i) {
    double r = scan->ranges[i];
    if (r <= scan->range_min || r >= scan->range_max) {
      continue;
    }
    double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    double lx = r * std::cos(angle) + lidar_x;
    double ly = r * std::sin(angle) + lidar_y;
    double wx = node.x + cos_yaw * lx - sin_yaw * ly;
    double wy = node.y + sin_yaw * lx + cos_yaw * ly;

    auto [h_px, h_py] = world_to_pixel(wx, wy, origin_x, origin_y, resolution);
    if (in_bounds(h_px, h_py, width, height)) {
      result.hit_px.push_back(h_px);
      result.hit_py.push_back(h_py);
    } else {
      result.out_of_bounds_hits++;
    }
  }

  return result;
}

ScanHitsPixels scan_hits_to_pixels(
    const core::PoseNode& node,
    double origin_x,
    double origin_y,
    double resolution,
    int map_size) {
  return scan_hits_to_pixels(node, origin_x, origin_y, resolution, map_size, map_size);
}

cv::Mat build_trajectory_mask(
    int rows, int cols,
    const std::vector<core::PoseNode>& nodes,
    double radius_m,
    double origin_x,
    double origin_y,
    double resolution) {
  cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
  if (nodes.empty()) {
    return mask;
  }

  int radius_px = std::max(1, static_cast<int>(radius_m / resolution));
  std::vector<cv::Point> pts;
  pts.reserve(nodes.size());
  for (const auto& n : nodes) {
    auto [px, py] = world_to_pixel(n.x, n.y, origin_x, origin_y, resolution);
    pts.emplace_back(px, py);
  }

  const cv::Point* curve[1] = {pts.data()};
  int npts[1] = {static_cast<int>(pts.size())};
  cv::polylines(mask, curve, npts, 1, false, cv::Scalar(1), radius_px * 2);

  for (const auto& p : pts) {
    cv::circle(mask, p, radius_px, cv::Scalar(1), -1);
  }

  return mask;
}

}  // namespace map_manager
}  // namespace slam_gnss_2d
