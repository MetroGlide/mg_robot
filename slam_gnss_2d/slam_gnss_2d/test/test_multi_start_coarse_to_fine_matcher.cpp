#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"
#include "slam_gnss_2d/scan_matching/multi_start_coarse_to_fine_matcher.hpp"
#include "slam_gnss_2d/scan_matching/ndt_matcher.hpp"
#include "slam_gnss_2d/scan_matching/reference_provider/local_map.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

static core::ScanDataPtr make_corridor_scan(double sensor_x, double sensor_y, double sensor_yaw) {
  auto scan = std::make_shared<core::ScanData>();
  scan->timestamp = 100.0;
  scan->angle_min = -M_PI;
  scan->angle_increment = 2.0 * M_PI / 720.0;
  scan->range_min = 0.1;
  scan->range_max = 30.0;
  scan->lidar_x = 0.0;
  scan->lidar_y = 0.0;
  scan->lidar_yaw = 0.0;
  scan->ranges.assign(720, 30.0f);

  for (int i = 0; i < 720; ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double global_beam_angle = sensor_yaw + angle;
    double ca = std::cos(global_beam_angle);
    double sa = std::sin(global_beam_angle);

    if (sa > 1e-4) {
      double r = (1.5 - sensor_y) / sa;
      double hit_x = sensor_x + r * ca;
      if (r > 0.1 && r < scan->ranges[i] && hit_x >= -10.0 && hit_x <= 10.0) {
        scan->ranges[i] = static_cast<float>(r);
      }
    }
    if (sa < -1e-4) {
      double r = (-1.5 - sensor_y) / sa;
      double hit_x = sensor_x + r * ca;
      if (r > 0.1 && r < scan->ranges[i] && hit_x >= -10.0 && hit_x <= 10.0) {
        scan->ranges[i] = static_cast<float>(r);
      }
    }
  }
  return scan;
}

static std::vector<Eigen::Vector2d> make_corridor_target() {
  std::vector<Eigen::Vector2d> pts;
  for (double x = -10.0; x <= 10.0; x += 0.1) {
    pts.emplace_back(x, 1.5);
    pts.emplace_back(x, -1.5);
  }
  return pts;
}

TEST(MultiStartCoarseToFineMatcherTest, RecoversFromLargeAngularSlip) {
  auto coarse = std::make_shared<NDTMatcher>(
      30, 1e-3, std::vector<double>{1.0, 0.5}, true, 1.0);
  auto fine = std::make_shared<ICPMatcher>(
      50, 1e-3, 2.0, "huber", 0.15, 1.0);

  MultiStartCoarseToFineMatcher matcher(
      coarse, fine, 20.0, 2.5, true, true);

  auto target_pts = make_corridor_target();
  matcher.set_target_cloud(target_pts);

  // ロボットの真の位置: (x=0.23, y=0.0, yaw=0.0) - 直進
  auto scan = make_corridor_scan(0.23, 0.0, 0.0);

  // オドメトリに片輪スリップによる大角度旋回角 -15度 (-0.2618 rad) が生じたケース
  double large_slip_yaw = -15.0 * M_PI / 180.0;
  core::OdomData initial_guess{100.0, 0.23, -0.01, large_slip_yaw};

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dyaw, 0.0, 0.02);
  EXPECT_NEAR(result.dy, 0.0, 0.05);
}

TEST(MultiStartCoarseToFineMatcherTest, AccurateTrackingOnCurvedMotion) {
  auto coarse = std::make_shared<NDTMatcher>(
      30, 1e-3, std::vector<double>{1.0, 0.5}, true, 1.0);
  auto fine = std::make_shared<ICPMatcher>(
      50, 1e-3, 2.0, "huber", 0.15, 1.0);

  MultiStartCoarseToFineMatcher matcher(
      coarse, fine, 20.0, 2.5, true, true);

  auto target_pts = make_corridor_target();
  matcher.set_target_cloud(target_pts);

  // ロボットが実際にカーブして 12度 (0.209 rad) 旋回したケース
  double true_yaw = 12.0 * M_PI / 180.0;
  auto scan = make_corridor_scan(0.22, 0.03, true_yaw);

  // オドメトリも正しくカーブを指示
  core::OdomData initial_guess{100.0, 0.20, 0.02, true_yaw + 0.02};

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  // 直進仮説 (0度) に誤って引きずられず、真の旋回角に正しく収束すること！
  EXPECT_NEAR(result.dyaw, true_yaw, 0.02);
  EXPECT_NEAR(result.dy, 0.03, 0.05);
}

// 縮退すべりを模倣するモックマッチャー
class MockDegenerateSlipMatcher : public ScanMatcherBase {
 public:
  void set_target_cloud(const std::vector<Eigen::Vector2d>&) override {}
  core::MatchResult match(
      const core::ConstScanDataPtr&,
      [[maybe_unused]] const core::OdomData& initial_guess) override {
    Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 500.0;
    return core::MatchResult{
        0.05, -0.08, -0.01, true, info, 0.02};
  }
};

TEST(ScanMatchingBuilderDegeneracyTest, PreservesOdomTranslationOnDegeneracySlip) {
  auto mock_matcher = std::make_shared<MockDegenerateSlipMatcher>();
  auto provider = std::make_shared<LocalMapProvider>(10, 20.0);

  pose_graph::ScanMatchingBuilder builder(
      mock_matcher, provider, 0.2, 0.1, 5, 0.08);

  auto scan1 = make_corridor_scan(0.0, 0.0, 0.0);
  core::OdomData odom1{100.0, 0.0, 0.0, 0.0};
  auto node1 = builder.add_scan(scan1, odom1);
  ASSERT_TRUE(node1.has_value());
  EXPECT_NEAR(node1->x, 0.0, 1e-4);

  // 直進で 0.24m 前進
  auto scan2 = make_corridor_scan(0.24, 0.0, 0.0);
  core::OdomData odom2{100.1, 0.24, 0.0, 0.0};
  auto node2 = builder.add_scan(scan2, odom2);
  ASSERT_TRUE(node2.has_value());

  // 直進時は縮退保護が作動し、0.24m が維持されること
  EXPECT_NEAR(node2->x, 0.24, 0.02);
  auto edges = builder.get_edges();
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_NEAR(edges[0].dx, 0.24, 0.02);
  EXPECT_LE(edges[0].information(0, 0), 20.0 + 1e-4);
}

// 旋回中に dx が乖離したモックマッチャー
class MockTurningMatcher : public ScanMatcherBase {
 public:
  void set_target_cloud(const std::vector<Eigen::Vector2d>&) override {}
  core::MatchResult match(
      const core::ConstScanDataPtr&,
      [[maybe_unused]] const core::OdomData& initial_guess) override {
    Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 500.0;
    // 旋回中: dyaw = 0.15 rad, dx = 0.15m
    return core::MatchResult{
        0.15, 0.05, 0.15, true, info, 0.02};
  }
};

TEST(ScanMatchingBuilderDegeneracyTest, BypassesProtectionDuringTurning) {
  auto mock_matcher = std::make_shared<MockTurningMatcher>();
  auto provider = std::make_shared<LocalMapProvider>(10, 20.0);

  pose_graph::ScanMatchingBuilder builder(
      mock_matcher, provider, 0.2, 0.1, 5, 0.08);

  auto scan1 = make_corridor_scan(0.0, 0.0, 0.0);
  core::OdomData odom1{100.0, 0.0, 0.0, 0.0};
  builder.add_scan(scan1, odom1);

  // 旋回移動: dyaw = 0.15 rad (> 0.05 rad), odom dx = 0.25m
  // マッチャー dx = 0.15m (乖離 0.10m > 0.08m)
  auto scan2 = make_corridor_scan(0.25, 0.05, 0.15);
  core::OdomData odom2{100.1, 0.25, 0.05, 0.15};
  auto node2 = builder.add_scan(scan2, odom2);
  ASSERT_TRUE(node2.has_value());

  // 旋回中は縮退保護がバイパスされ、マッチャーの dx=0.15m がそのまま採用されること！
  auto edges = builder.get_edges();
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_NEAR(edges[0].dx, 0.15, 0.02);
  // 情報行列も縮小されず 500.0 が維持されること
  EXPECT_GT(edges[0].information(0, 0), 100.0);
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
