#pragma once

#include <memory>
#include <optional>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class ReferenceProviderBase {
 public:
  virtual ~ReferenceProviderBase() = default;

  virtual void update(const core::PoseNode& node) = 0;

  virtual std::optional<std::vector<Eigen::Vector2d>> get_reference_pts() = 0;

  virtual void invalidate_cache() {}
  virtual void sync_poses(const std::vector<core::PoseNode>& nodes) { (void)nodes; }
};

using ReferenceProviderPtr = std::shared_ptr<ReferenceProviderBase>;

}  // namespace scan_matching
}  // namespace slam_gnss_2d
