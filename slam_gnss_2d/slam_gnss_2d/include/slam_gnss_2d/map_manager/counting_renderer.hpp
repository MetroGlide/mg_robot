#pragma once

#include <mutex>
#include <vector>
#include <opencv2/core.hpp>

#include "slam_gnss_2d/map_manager/base.hpp"

namespace slam_gnss_2d {
namespace map_manager {

class CountingRenderer : public MapRendererBase {
 public:
  CountingRenderer(
      double resolution,
      double expansion_margin,
      double hit_threshold,
      int min_hits = 1,
      double hit_weight = 1.0,
      double miss_weight = 0.25,
      double miss_clearance_margin = 0.05,
      double max_miss_ratio = 2.0);

  bool add_node(const core::PoseNode& node) override;
  void rerender_all(const std::vector<core::PoseNode>& nodes) override;
  OccupancyGridData to_occupancy_array() override;
  void apply_trajectory_mask(
      const std::vector<core::PoseNode>& nodes,
      double radius_m,
      const std::string& filter_type = "clear") override;

 private:
  double resolution_;
  double expansion_margin_;
  double hit_threshold_;
  int min_hits_{1};
  double hit_weight_{1.0};
  double miss_weight_{0.25};
  double miss_clearance_margin_{0.05};
  double max_miss_ratio_{2.0};
  int map_width_{1};
  int map_height_{1};
  double origin_x_{0.0};
  double origin_y_{0.0};
  bool initialized_{false};
  cv::Mat hit_map_;
  cv::Mat miss_map_;
  int render_count_{0};
  std::mutex mutex_;

  bool render_node(const core::PoseNode& node);
  bool render_node_impl(
      const core::PoseNode& node,
      cv::Mat& target_hit,
      cv::Mat& target_miss);
};

}  // namespace map_manager
}  // namespace slam_gnss_2d
