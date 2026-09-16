#include "slam_gnss_2d/map_manager/counting_renderer.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/map_manager/grid_utils.hpp"

namespace slam_gnss_2d {
namespace map_manager {

CountingRenderer::CountingRenderer(
    double resolution,
    double expansion_margin,
    double hit_threshold,
    int min_hits)
    : resolution_(resolution),
      expansion_margin_(expansion_margin),
      hit_threshold_(hit_threshold),
      min_hits_(min_hits),
      hit_map_(cv::Mat::zeros(1, 1, CV_32SC1)),
      miss_map_(cv::Mat::zeros(1, 1, CV_32SC1)) {
  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.counting_renderer"),
      "CountingRenderer initialized: resolution=%.3f, expansion_margin=%.1f, "
      "hit_threshold=%.2f, min_hits=%d",
      resolution_, expansion_margin_, hit_threshold_, min_hits_);
}

bool CountingRenderer::add_node(const core::PoseNode& node) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!node.scan) {
    return true;
  }
  if (!initialized_ || map_size_ <= 1) {
    return false;
  }
  auto [r_px, r_py] = world_to_pixel(node.x, node.y, origin_x_, origin_y_, resolution_);
  if (!in_bounds(r_px, r_py, map_size_)) {
    return false;
  }
  return render_node(node);
}

void CountingRenderer::rerender_all(const std::vector<core::PoseNode>& nodes) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (nodes.empty()) {
    return;
  }

  auto [new_ox, new_oy, new_size] = compute_square_bounds(
      nodes, resolution_, expansion_margin_);
  origin_x_ = new_ox;
  origin_y_ = new_oy;
  map_size_ = new_size;
  hit_map_ = cv::Mat::zeros(new_size, new_size, CV_32SC1);
  miss_map_ = cv::Mat::zeros(new_size, new_size, CV_32SC1);
  render_count_ = 0;
  initialized_ = true;

  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.counting_renderer"),
      "Map recomputed (Counting): size=%dpx (%.0fm), origin=(%.1f, %.1f), nodes=%zu",
      new_size, new_size * resolution_, new_ox, new_oy, nodes.size());

  for (const auto& node : nodes) {
    if (node.scan) {
      render_node(node);
    }
  }
}

OccupancyGridData CountingRenderer::to_occupancy_array() {
  std::lock_guard<std::mutex> lock(mutex_);
  cv::Mat occ(hit_map_.rows, hit_map_.cols, CV_8SC1);

  int occupied_count = 0;
  int free_count = 0;

  for (int r = 0; r < hit_map_.rows; ++r) {
    const int32_t* hit_row = hit_map_.ptr<int32_t>(r);
    const int32_t* miss_row = miss_map_.ptr<int32_t>(r);
    int8_t* occ_row = occ.ptr<int8_t>(r);

    for (int c = 0; c < hit_map_.cols; ++c) {
      int32_t hits = hit_row[c];
      int32_t misses = miss_row[c];
      int32_t total = hits + misses;

      if (total == 0) {
        occ_row[c] = -1;
      } else {
        double ratio = static_cast<double>(hits) / static_cast<double>(total);
        if (hits >= min_hits_ && ratio >= hit_threshold_) {
          occ_row[c] = 100;
          occupied_count++;
        } else {
          occ_row[c] = 0;
          free_count++;
        }
      }
    }
  }

  if (render_count_ == 1 || render_count_ % 50 == 0) {
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.counting_renderer"),
        "Map stats (Counting): rendered=%d, occupied=%d px, free=%d px",
        render_count_, occupied_count, free_count);
  }

  return OccupancyGridData{occ, origin_x_, origin_y_, resolution_};
}

void CountingRenderer::apply_trajectory_mask(
    const std::vector<core::PoseNode>& nodes,
    double radius_m,
    const std::string& filter_type) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (nodes.empty()) {
    return;
  }

  cv::Mat mask = build_trajectory_mask(
      hit_map_.rows, hit_map_.cols, nodes, radius_m, origin_x_, origin_y_, resolution_);

  for (int r = 0; r < hit_map_.rows; ++r) {
    const uint8_t* mask_row = mask.ptr<uint8_t>(r);
    int32_t* hit_row = hit_map_.ptr<int32_t>(r);
    int32_t* miss_row = miss_map_.ptr<int32_t>(r);

    for (int c = 0; c < hit_map_.cols; ++c) {
      if (mask_row[c] > 0) {
        if (filter_type == "clear") {
          hit_row[c] = 0;
          miss_row[c] += 1;
        } else if (filter_type == "attenuate") {
          hit_row[c] = std::max(0, hit_row[c] - 2) / 2;
          miss_row[c] += 2;
        }
      }
    }
  }
}

bool CountingRenderer::render_node(const core::PoseNode& node) {
  auto scan_pixels = scan_hits_to_pixels(
      node, origin_x_, origin_y_, resolution_, map_size_);
  if (!in_bounds(scan_pixels.robot_px, scan_pixels.robot_py, map_size_)) {
    return false;
  }

  render_count_++;
  size_t n_hits = scan_pixels.hit_px.size();
  if (n_hits == 0) {
    return true;
  }

  int min_px = scan_pixels.robot_px;
  int max_px = scan_pixels.robot_px;
  int min_py = scan_pixels.robot_py;
  int max_py = scan_pixels.robot_py;

  for (size_t i = 0; i < n_hits; ++i) {
    min_px = std::min(min_px, scan_pixels.hit_px[i]);
    max_px = std::max(max_px, scan_pixels.hit_px[i]);
    min_py = std::min(min_py, scan_pixels.hit_py[i]);
    max_py = std::max(max_py, scan_pixels.hit_py[i]);
  }

  int h = max_py - min_py + 1;
  int w = max_px - min_px + 1;
  cv::Mat local_mask = cv::Mat::zeros(h, w, CV_8UC1);

  cv::Point start(scan_pixels.robot_px - min_px, scan_pixels.robot_py - min_py);
  for (size_t i = 0; i < n_hits; ++i) {
    cv::Point end(scan_pixels.hit_px[i] - min_px, scan_pixels.hit_py[i] - min_py);
    cv::line(local_mask, start, end, cv::Scalar(1), 1);
  }

  for (size_t i = 0; i < n_hits; ++i) {
    int lx = scan_pixels.hit_px[i] - min_px;
    int ly = scan_pixels.hit_py[i] - min_py;
    local_mask.at<uint8_t>(ly, lx) = 0;
  }

  for (int r = 0; r < h; ++r) {
    const uint8_t* lmask_row = local_mask.ptr<uint8_t>(r);
    int32_t* miss_row = miss_map_.ptr<int32_t>(min_py + r);
    for (int c = 0; c < w; ++c) {
      if (lmask_row[c] > 0) {
        miss_row[min_px + c] += 1;
      }
    }
  }

  for (size_t i = 0; i < n_hits; ++i) {
    hit_map_.at<int32_t>(scan_pixels.hit_py[i], scan_pixels.hit_px[i]) += 1;
  }

  if (render_count_ == 1 || render_count_ % 50 == 0) {
    RCLCPP_DEBUG(
        rclcpp::get_logger("slam_gnss_2d.counting_renderer"),
        "Render #%d (Counting): robot=(%.2f, %.2f), hits=%zu",
        render_count_, node.x, node.y, n_hits);
  }

  return true;
}

}  // namespace map_manager
}  // namespace slam_gnss_2d
