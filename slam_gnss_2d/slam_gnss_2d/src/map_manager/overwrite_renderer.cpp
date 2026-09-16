#include "slam_gnss_2d/map_manager/overwrite_renderer.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/map_manager/grid_utils.hpp"

namespace slam_gnss_2d {
namespace map_manager {

OverwriteRenderer::OverwriteRenderer(double resolution, double expansion_margin)
    : resolution_(resolution),
      expansion_margin_(expansion_margin),
      map_(cv::Mat(1, 1, CV_8UC1, cv::Scalar(128))) {}

bool OverwriteRenderer::add_node(const core::PoseNode& node) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!node.scan) {
    return true;
  }
  if (map_size_ <= 1) {
    return false;
  }
  auto [r_px, r_py] = world_to_pixel(node.x, node.y, origin_x_, origin_y_, resolution_);
  if (!in_bounds(r_px, r_py, map_size_)) {
    return false;
  }
  return render_node(node);
}

void OverwriteRenderer::rerender_all(const std::vector<core::PoseNode>& nodes) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (nodes.empty()) {
    return;
  }

  auto [new_ox, new_oy, new_size] = compute_square_bounds(
      nodes, resolution_, expansion_margin_);
  origin_x_ = new_ox;
  origin_y_ = new_oy;
  map_size_ = new_size;
  map_ = cv::Mat(new_size, new_size, CV_8UC1, cv::Scalar(128));
  render_count_ = 0;

  for (const auto& node : nodes) {
    if (node.scan) {
      render_node(node);
    }
  }
}

OccupancyGridData OverwriteRenderer::to_occupancy_array() {
  std::lock_guard<std::mutex> lock(mutex_);
  cv::Mat occ(map_.rows, map_.cols, CV_8SC1);
  for (int r = 0; r < map_.rows; ++r) {
    const uint8_t* src = map_.ptr<uint8_t>(r);
    int8_t* dst = occ.ptr<int8_t>(r);
    for (int c = 0; c < map_.cols; ++c) {
      uint8_t val = src[c];
      if (val == 128) {
        dst[c] = -1;
      } else if (val == 255) {
        dst[c] = 0;
      } else {
        dst[c] = 100;
      }
    }
  }
  return OccupancyGridData{occ, origin_x_, origin_y_, resolution_};
}

void OverwriteRenderer::apply_trajectory_mask(
    const std::vector<core::PoseNode>& nodes,
    double radius_m,
    [[maybe_unused]] const std::string& filter_type) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (nodes.empty()) {
    return;
  }
  cv::Mat mask = build_trajectory_mask(
      map_.rows, map_.cols, nodes, radius_m, origin_x_, origin_y_, resolution_);
  map_.setTo(cv::Scalar(255), mask > 0);
}

bool OverwriteRenderer::render_node(const core::PoseNode& node) {
  auto scan_pixels = scan_hits_to_pixels(
      node, origin_x_, origin_y_, resolution_, map_size_);
  if (!in_bounds(scan_pixels.robot_px, scan_pixels.robot_py, map_size_)) {
    return false;
  }

  render_count_++;
  size_t n_hits = scan_pixels.hit_px.size();
  if (n_hits > 0) {
    cv::Point start(scan_pixels.robot_px, scan_pixels.robot_py);
    for (size_t i = 0; i < n_hits; ++i) {
      cv::Point end(scan_pixels.hit_px[i], scan_pixels.hit_py[i]);
      cv::line(map_, start, end, cv::Scalar(255), 1);
    }
    for (size_t i = 0; i < n_hits; ++i) {
      map_.at<uint8_t>(scan_pixels.hit_py[i], scan_pixels.hit_px[i]) = 0;
    }
  }

  return true;
}

}  // namespace map_manager
}  // namespace slam_gnss_2d
