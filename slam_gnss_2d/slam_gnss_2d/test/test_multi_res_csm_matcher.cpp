#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/scan_matching/multi_res_csm_matcher.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

namespace {

core::ScanDataPtr make_corner_scan() {
  auto scan = std::make_shared<core::ScanData>();
  scan->timestamp = 1.0;
  scan->angle_min = -M_PI;
  scan->angle_increment = 2.0 * M_PI / 360.0;
  scan->range_min = 0.1;
  scan->range_max = 30.0;
  scan->lidar_x = 0.0;
  scan->lidar_y = 0.0;
  scan->lidar_yaw = 0.0;
  scan->ranges.assign(360, 10.0f);

  for (int i = 0; i < 360; ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double ca = std::cos(angle);
    double sa = std::sin(angle);
    if (ca > 0.1 && std::abs(sa / ca) <= 1.0) {
      double r = 2.0 / ca;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
    if (sa > 0.1 && std::abs(ca / sa) <= 1.0) {
      double r = 2.0 / sa;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
  }
  return scan;
}

std::vector<Eigen::Vector2d> make_corner_target_points() {
  std::vector<Eigen::Vector2d> target;
  for (double y = -2.0; y <= 2.0; y += 0.03) {
    target.emplace_back(2.0, y);
  }
  for (double x = -2.0; x <= 2.0; x += 0.03) {
    target.emplace_back(x, 2.0);
  }
  return target;
}

}  // namespace

TEST(MultiResCSMMatcherTest, RecoversFromLargeInitialOffset) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      100, 1e-4, 1.0, "huber", 0.15, 1.0,
      10.0, 500.0, 0.0, 1e-4, 1e-4);

  MultiResCSMMatcher matcher(
      fine_matcher,
      0.3,    // linear_search_window [m]
      12.0,   // angular_search_window_deg [deg]
      0.02,   // linear_step [m]
      1.0,    // angular_step_deg [deg]
      0.03,   // grid_resolution [m]
      0.1);   // score_threshold

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  auto scan = make_corner_scan();

  // オドメトリ初期値に大きなバイアスを与える（並進 +0.15m, -0.10m, 回転 +6.0度）
  core::OdomData initial_guess{
      1.0,
      0.15,
      -0.10,
      6.0 * M_PI / 180.0,
  };

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.03);
  EXPECT_NEAR(result.dy, 0.0, 0.03);
  EXPECT_NEAR(result.dyaw, 0.0, 0.03);
  EXPECT_GT(result.score, 0.3);
}

TEST(MultiResCSMMatcherTest, HandlesPureRotationBias) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      100, 1e-4, 1.0, "huber", 0.15, 1.0,
      10.0, 500.0, 0.0, 1e-4, 1e-4);

  MultiResCSMMatcher matcher(
      fine_matcher,
      0.2,    // linear_search_window
      15.0,   // angular_search_window_deg
      0.02,   // linear_step
      0.5,    // angular_step_deg
      0.03,   // grid_resolution
      0.1);

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  auto scan = make_corner_scan();

  // 回転のみ 8.0度 の大きなバイアス
  core::OdomData initial_guess{
      1.0,
      0.0,
      0.0,
      8.0 * M_PI / 180.0,
  };

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.03);
  EXPECT_NEAR(result.dy, 0.0, 0.03);
  EXPECT_NEAR(result.dyaw, 0.0, 0.02);
}

TEST(MultiResCSMMatcherTest, DegeneracyHandlingEmptyScan) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      50, 1e-3, 1.0, "huber", 0.15, 1.0);

  MultiResCSMMatcher matcher(fine_matcher);

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  core::OdomData initial_guess{1.0, 0.0, 0.0, 0.0};
  auto empty_scan = std::make_shared<core::ScanData>();
  auto result = matcher.match(empty_scan, initial_guess);

  EXPECT_FALSE(result.converged);
}

TEST(MultiResCSMMatcherTest, VariancePenaltyPreservesStraightHypothesis) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      100, 1e-4, 1.0, "huber", 0.15, 1.0,
      10.0, 500.0, 0.0, 1e-4, 1e-4);

  // Variance penalty を有効化 (distance=0.5, angle=1.0)
  MultiResCSMMatcher matcher(
      fine_matcher,
      0.3,    // linear_search_window
      15.0,   // angular_search_window_deg
      0.02,   // linear_step
      0.5,    // angular_step_deg
      0.03,   // grid_resolution
      0.1,    // score_threshold
      true,   // enable_variance_penalty
      0.5,    // distance_variance_penalty
      1.0,    // angle_variance_penalty
      0.5,    // minimum_distance_penalty
      0.9);   // minimum_angle_penalty

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  auto scan = make_corner_scan();

  // オドメトリ直進 (0.0, 0.0, 0.0)
  core::OdomData initial_guess{1.0, 0.0, 0.0, 0.0};
  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.02);
  EXPECT_NEAR(result.dy, 0.0, 0.02);
  EXPECT_NEAR(result.dyaw, 0.0, 0.01);
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
