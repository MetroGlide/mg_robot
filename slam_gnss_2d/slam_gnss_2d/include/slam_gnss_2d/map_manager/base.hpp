#pragma once

#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace map_manager {

struct OccupancyGridData {
  cv::Mat data;  // CV_8SC1 (-1, 0, 100)
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.05};
};

class MapRendererBase {
 public:
  virtual ~MapRendererBase() = default;

  virtual bool add_node(const core::PoseNode& node) = 0;
  virtual void rerender_all(const std::vector<core::PoseNode>& nodes) = 0;
  virtual OccupancyGridData to_occupancy_array() = 0;
  virtual void apply_trajectory_mask(
      const std::vector<core::PoseNode>& nodes,
      double radius_m,
      const std::string& filter_type = "clear") = 0;
};

using MapRendererPtr = std::shared_ptr<MapRendererBase>;

}  // namespace map_manager
}  // namespace slam_gnss_2d
