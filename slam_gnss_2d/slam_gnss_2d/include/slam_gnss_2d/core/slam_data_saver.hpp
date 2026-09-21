#pragma once

#include <optional>
#include <string>
#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace core {

class SlamDataSaver {
 public:
  static std::string save_gnss_transform(
      const std::string& output_dir,
      double anchor_lat,
      double anchor_lon,
      double anchor_utm_easting,
      double anchor_utm_northing,
      int utm_zone,
      const std::string& utm_hemisphere,
      double rotation_rad,
      const std::string& backend_name = "unknown");

  static std::string save_pose_graph(
      const std::string& output_dir,
      const std::vector<PoseNode>& nodes,
      const std::vector<PoseEdge>& edges,
      const std::optional<std::string>& bag_path = std::nullopt);
};

}  // namespace core
}  // namespace slam_gnss_2d
