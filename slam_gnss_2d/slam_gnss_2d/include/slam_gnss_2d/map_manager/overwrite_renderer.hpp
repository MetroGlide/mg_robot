#pragma once

#include <mutex>
#include <vector>
#include <opencv2/core.hpp>

#include "slam_gnss_2d/map_manager/base.hpp"

namespace slam_gnss_2d {
namespace map_manager {

class OverwriteRenderer : public MapRendererBase {
 public:
  OverwriteRenderer(double resolution, double expansion_margin);

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
  int map_size_{1};
  double origin_x_{0.0};
  double origin_y_{0.0};
  cv::Mat map_;
  int render_count_{0};
  std::mutex mutex_;

  bool render_node(const core::PoseNode& node);
};

}  // namespace map_manager
}  // namespace slam_gnss_2d
