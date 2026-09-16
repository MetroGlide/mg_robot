#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

class PoseGraphBuilderBase {
 public:
  virtual ~PoseGraphBuilderBase() = default;

  virtual std::optional<core::PoseNode> add_scan(
      const core::ScanDataPtr& scan,
      const core::OdomData& odom) = 0;

  virtual std::vector<core::PoseNode> get_nodes() const = 0;
  virtual std::vector<core::PoseEdge> get_edges() const = 0;
  virtual void reset() = 0;

  virtual bool loop_just_closed() = 0;

  virtual void replace_nodes(const std::vector<core::PoseNode>& nodes) {
    (void)nodes;
  }
};

using PoseGraphBuilderPtr = std::shared_ptr<PoseGraphBuilderBase>;

}  // namespace pose_graph
}  // namespace slam_gnss_2d
