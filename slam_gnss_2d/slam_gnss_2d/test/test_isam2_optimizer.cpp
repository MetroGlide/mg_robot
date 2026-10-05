#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/geometry.hpp"
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

TEST(OptimizerTest, GnssPriorWithLeverArmPlacesAntennaOnMeasurement) {
  // ロボットは (1, 2) で yaw = 90 deg。アンテナは base_link の (0.26, -0.13) にあるので、
  // 世界座標では (1.13, 2.26) に観測される。
  const double true_x = 1.0;
  const double true_y = 2.0;
  const double true_yaw = M_PI / 2.0;
  const double lever_x = 0.26;
  const double lever_y = -0.13;
  const auto [antenna_dx, antenna_dy] = local_delta_to_world(lever_x, lever_y, true_yaw);
  const double antenna_x = true_x + antenna_dx;
  const double antenna_y = true_y + antenna_dy;

  // 方位は強く拘束し、位置は弱く拘束した初期化
  auto run = [&](double lx, double ly) {
    ISAM2Optimizer optimizer(0.1);
    optimizer.initialize(0, true_x + 0.2, true_y - 0.1, true_yaw, 10.0, 0.001);
    optimizer.add_gnss_prior(0, antenna_x, antenna_y, 0.01, 0.0, "huber", 1.5, gtsam::Point2(lx, ly));
    optimizer.update();
    return *optimizer.get_pose(0);
  };

  auto [x_lever, y_lever, yaw_lever] = run(lever_x, lever_y);
  EXPECT_NEAR(x_lever, true_x, 0.02);
  EXPECT_NEAR(y_lever, true_y, 0.02);
  EXPECT_NEAR(yaw_lever, true_yaw, 0.01);

  // レバーアームなしでは、ロボット位置がアンテナ位置に引かれて 0.29 m ずれる
  auto [x_plain, y_plain, yaw_plain] = run(0.0, 0.0);
  EXPECT_NEAR(std::hypot(x_plain - true_x, y_plain - true_y), std::hypot(lever_x, lever_y), 0.05);
}

TEST(OptimizerTest, RobustBetweenFactorDownWeightsOutlierEdge) {
  // 0 -> 1 -> 2 の逐次エッジ (各 1.0 m) に、0 -> 2 の外れ値エッジ (5.0 m) を追加する
  auto solve = [](const std::string& kernel) {
    ISAM2Optimizer optimizer(0.1);
    optimizer.set_between_robust_kernel(kernel, 1.345);
    optimizer.initialize(0, 0.0, 0.0, 0.0, 0.001, 0.001);
    optimizer.add_initial_estimate(1, 1.0, 0.0, 0.0);
    optimizer.add_initial_estimate(2, 2.0, 0.0, 0.0);
    Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 10000.0;
    optimizer.add_between_factor(0, 1, 1.0, 0.0, 0.0, info);
    optimizer.add_between_factor(1, 2, 1.0, 0.0, 0.0, info);
    optimizer.add_between_factor(0, 2, 5.0, 0.0, 0.0, info);
    optimizer.update();
    optimizer.run_batch_optimization(50);
    return std::get<0>(*optimizer.get_pose(2));
  };
  const double gaussian_x = solve("none");
  const double robust_x = solve("cauchy");
  // ガウスは外れ値に引かれて 2.0 から大きくずれ、ロバストは 2.0 に近い
  EXPECT_GT(std::abs(gaussian_x - 2.0), 0.5);
  EXPECT_LT(std::abs(robust_x - 2.0), std::abs(gaussian_x - 2.0));
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

TEST(OptimizerTest, ISAM2TwoStageBatchOptimizationPreservesLinearity) {
  ISAM2Optimizer optimizer(0.1);
  optimizer.initialize(0, 0.0, 0.0, 0.0, 0.01, 0.01);

  // 0 -> 1 -> 2 -> 3 -> 4 直線軌跡
  Eigen::Matrix3d stiff_edge = Eigen::Matrix3d::Zero();
  stiff_edge(0, 0) = 500.0;
  stiff_edge(1, 1) = 2000.0;
  stiff_edge(2, 2) = 3000.0;

  for (int i = 1; i <= 4; ++i) {
    optimizer.add_initial_estimate(i, static_cast<double>(i), 0.0, 0.0);
    optimizer.add_between_factor(i - 1, i, 1.0, 0.0, 0.0, stiff_edge);
  }

  // ノード2にGNSS Prior (少しy方向にオフセット)
  optimizer.add_gnss_prior(2, 2.0, 0.2, 0.1, 0.0, "cauchy", 1.5);
  optimizer.update();

  // 二段階バッチ最適化を実行
  optimizer.run_batch_optimization(100);

  auto p0 = optimizer.get_pose(0);
  auto p2 = optimizer.get_pose(2);
  auto p4 = optimizer.get_pose(4);
  ASSERT_TRUE(p0.has_value() && p2.has_value() && p4.has_value());

  // 直線性が維持されているか（相対ステップのdyawがほぼ0であること）
  for (int i = 1; i <= 4; ++i) {
    auto p_prev = optimizer.get_pose(i - 1);
    auto p_curr = optimizer.get_pose(i);
    ASSERT_TRUE(p_prev.has_value() && p_curr.has_value());
    EXPECT_NEAR(std::get<2>(*p_curr), std::get<2>(*p_prev), 0.05);
  }
}

}  // namespace optimizer
}  // namespace slam_gnss_2d
