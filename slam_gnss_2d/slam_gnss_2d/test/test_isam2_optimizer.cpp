#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/optimizer/isam2_optimizer.hpp"
#include "slam_gnss_2d/optimizer/gtsam_optimizer.hpp"

namespace slam_gnss_2d {
namespace optimizer {

TEST(OptimizerTest, ISAM2AddGnssPrior) {
  ISAM2Optimizer optimizer(0.1);
  optimizer.initialize(0, 0.0, 0.0, 0.0, 0.1, 0.1);

  // Add node 1 with initial estimate (1.0, 0.0, 0.0)
  optimizer.add_initial_estimate(1, 1.0, 0.0, 0.0);

  // Odometry between factor (0 -> 1): dx=1.0, dy=0.0, dyaw=0.0
  Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 100.0;
  optimizer.add_between_factor(0, 1, 1.0, 0.0, 0.0, info);

  // GNSS prior at node 1 pull towards (1.5, 0.5)
  optimizer.add_gnss_prior(1, 1.5, 0.5, 0.1);
  optimizer.update();

  auto pose = optimizer.get_pose(1);
  ASSERT_TRUE(pose.has_value());
  auto [x, y, yaw] = *pose;

  // Node 1 should be pulled toward the GNSS prior
  EXPECT_GT(x, 1.0);
  EXPECT_GT(y, 0.0);
}

TEST(OptimizerTest, ISAM2AddGnssPriorCauchy) {
  ISAM2Optimizer optimizer(0.1);
  optimizer.initialize(0, 0.0, 0.0, 0.0, 0.1, 0.1);

  optimizer.add_initial_estimate(1, 1.0, 0.0, 0.0);
  Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 100.0;
  optimizer.add_between_factor(0, 1, 1.0, 0.0, 0.0, info);

  // Cauchy robust kernel
  optimizer.add_gnss_prior(1, 1.5, 0.5, 0.1, 0.0, "cauchy", 1.5);
  optimizer.update();

  auto pose = optimizer.get_pose(1);
  ASSERT_TRUE(pose.has_value());
  auto [x, y, yaw] = *pose;

  EXPECT_GT(x, 1.0);
  EXPECT_GT(y, 0.0);
}

TEST(OptimizerTest, GTSAMOptimizerWithPriors) {
  GTSAMOptimizer optimizer;

  std::vector<core::PoseNode> nodes = {
      {0, 1.0, 0.0, 0.0, 0.0, nullptr},
      {1, 2.0, 1.0, 0.0, 0.0, nullptr},
  };

  Eigen::Matrix3d edge_info = Eigen::Matrix3d::Identity() * 100.0;
  std::vector<core::PoseEdge> edges = {
      {0, 1, 1.0, 0.0, 0.0, edge_info, 0.0, false},
  };

  Eigen::Matrix2d prior_info = Eigen::Matrix2d::Identity() * 25.0;
  std::vector<core::GnssPrior> priors = {
      {1, 1.2, 0.2, prior_info},
  };

  auto result = optimizer.optimize(nodes, edges, priors);
  ASSERT_EQ(result.size(), 2u);
  EXPECT_GT(result[1].x, 1.0);
  EXPECT_GT(result[1].y, 0.0);
}

}  // namespace optimizer
}  // namespace slam_gnss_2d
