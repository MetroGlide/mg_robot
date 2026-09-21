#pragma once

#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace optimizer {

class GTSAMOptimizer {
 public:
  GTSAMOptimizer() = default;

  std::vector<core::PoseNode> optimize(
      const std::vector<core::PoseNode>& nodes,
      const std::vector<core::PoseEdge>& edges,
      const std::vector<core::GnssPrior>& gnss_priors = {});
};

}  // namespace optimizer
}  // namespace slam_gnss_2d
