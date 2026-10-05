#pragma once

#include <tuple>
#include <vector>
#include <opencv2/core.hpp>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace map_manager {

std::tuple<double, double, int, int> compute_bounds(
    const std::vector<core::PoseNode>& nodes,
    double resolution,
    double expansion_margin);

std::tuple<double, double, int> compute_square_bounds(
    const std::vector<core::PoseNode>& nodes,
    double resolution,
    double expansion_margin);

std::pair<int, int> world_to_pixel(
    double wx, double wy,
    double origin_x, double origin_y,
    double resolution);

bool in_bounds(int px, int py, int width, int height);
bool in_bounds(int px, int py, int map_size);

struct ScanHitsPixels {
  int robot_px{0};
  int robot_py{0};
  std::vector<int> hit_px;
  std::vector<int> hit_py;
  size_t out_of_bounds_hits{0};
};

ScanHitsPixels scan_hits_to_pixels(
    const core::PoseNode& node,
    double origin_x,
    double origin_y,
    double resolution,
    int width,
    int height);

ScanHitsPixels scan_hits_to_pixels(
    const core::PoseNode& node,
    double origin_x,
    double origin_y,
    double resolution,
    int map_size);

cv::Mat build_trajectory_mask(
    int rows, int cols,
    const std::vector<core::PoseNode>& nodes,
    double radius_m,
    double origin_x,
    double origin_y,
    double resolution);

}  // namespace map_manager
}  // namespace slam_gnss_2d
