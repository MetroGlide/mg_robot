#pragma once

#include <optional>
#include <vector>

#include "slam_gnss_2d/pose_graph/base.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

class OdomOnlyBuilder : public PoseGraphBuilderBase {
 public:
  OdomOnlyBuilder(double min_translation, double min_rotation);

  std::optional<core::PoseNode> add_scan(
      const core::ScanDataPtr& scan,
      const core::OdomData& odom) override;

  std::vector<core::PoseNode> get_nodes() const override;
  std::vector<core::PoseEdge> get_edges() const override;
  void reset() override;

  bool loop_just_closed() override { return false; }
  void replace_nodes(const std::vector<core::PoseNode>& nodes) override;

 private:
  double min_translation_;
  double min_rotation_;
  std::vector<core::PoseNode> nodes_;
  std::vector<core::PoseEdge> edges_;
  std::optional<core::OdomData> last_odom_;
};

}  // namespace pose_graph
}  // namespace slam_gnss_2d
