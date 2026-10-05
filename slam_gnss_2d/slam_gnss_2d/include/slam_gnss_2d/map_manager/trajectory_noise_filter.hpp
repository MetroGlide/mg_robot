#pragma once

#include <memory>
#include <vector>

#include "slam_gnss_2d/core/config.hpp"
#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/map_manager/base.hpp"

namespace slam_gnss_2d {
namespace map_manager {

class TrajectoryNoiseFilterBase {
 public:
  virtual ~TrajectoryNoiseFilterBase() = default;
  virtual void apply(
      MapRendererBase& renderer,
      const std::vector<core::PoseNode>& nodes) = 0;
};

class TrajectoryNoiseFilter : public TrajectoryNoiseFilterBase {
 public:
  explicit TrajectoryNoiseFilter(const core::TrajectoryNoiseFilterConfig& config);

  void apply(
      MapRendererBase& renderer,
      const std::vector<core::PoseNode>& nodes) override;

 private:
  core::TrajectoryNoiseFilterConfig config_;
};

using TrajectoryNoiseFilterPtr = std::shared_ptr<TrajectoryNoiseFilterBase>;

}  // namespace map_manager
}  // namespace slam_gnss_2d
