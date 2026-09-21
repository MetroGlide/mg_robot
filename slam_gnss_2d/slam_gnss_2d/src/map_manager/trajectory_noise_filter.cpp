#include "slam_gnss_2d/map_manager/trajectory_noise_filter.hpp"

#include <rclcpp/rclcpp.hpp>

namespace slam_gnss_2d {
namespace map_manager {

TrajectoryNoiseFilter::TrajectoryNoiseFilter(
    const core::TrajectoryNoiseFilterConfig& config)
    : config_(config) {}

void TrajectoryNoiseFilter::apply(
    MapRendererBase& renderer,
    const std::vector<core::PoseNode>& nodes) {
  if (!config_.enabled || nodes.empty()) {
    return;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("slam_gnss_2d.trajectory_noise_filter"),
      "Applying trajectory noise filter (type=%s, radius=%.2fm)",
      config_.type.c_str(), config_.radius_m);

  renderer.apply_trajectory_mask(nodes, config_.radius_m, config_.type);
}

}  // namespace map_manager
}  // namespace slam_gnss_2d
