#pragma once

#include <deque>
#include <optional>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/scan_matching/reference_provider/base.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

class LocalMapProvider : public ReferenceProviderBase {
 public:
  LocalMapProvider(int window, double radius);

  void update(const core::PoseNode& node) override;
  void invalidate_cache() override;
  void sync_poses(const std::vector<core::PoseNode>& nodes) override;
  std::optional<std::vector<Eigen::Vector2d>> get_reference_pts() override;
  std::optional<std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>>
  get_reference_pts_and_normals() override;

 private:
  struct CachedKeyframe {
    core::PoseNode node;
    std::vector<Eigen::Vector2d> pts;
    std::vector<Eigen::Vector2d> normals;
  };

  int window_;
  double radius_;
  std::deque<CachedKeyframe> nodes_;
  std::optional<core::PoseNode> last_node_;
};

}  // namespace scan_matching
}  // namespace slam_gnss_2d
