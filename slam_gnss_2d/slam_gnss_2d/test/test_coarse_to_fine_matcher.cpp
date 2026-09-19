#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/scan_matching/coarse_to_fine_matcher.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"
#include "slam_gnss_2d/scan_matching/ndt_matcher.hpp"
#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"
#include "slam_gnss_2d/scan_matching/reference_provider/local_map.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

static core::ScanDataPtr make_box_scan() {
  auto scan = std::make_shared<core::ScanData>();
  scan->timestamp = 100.0;
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
      double r = 1.0 / ca;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
    if (sa > 0.1 && std::abs(ca / sa) <= 1.0) {
      double r = 1.0 / sa;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
  }
  return scan;
}

TEST(CoarseToFineMatcherTest, ConvergesAccurately) {
  auto coarse = std::make_shared<NDTMatcher>(
      50, 1e-3, std::vector<double>{1.0, 0.5}, true, 1.0);
  auto fine = std::make_shared<ICPMatcher>(
      50, 1e-3, 1.5, "huber", 0.15, 1.0);

  CoarseToFineMatcher matcher(coarse, fine);

  std::vector<Eigen::Vector2d> target_pts;
  for (double y = -1.0; y <= 1.0; y += 0.05) {
    target_pts.emplace_back(1.0, y);
  }
  for (double x = 0.0; x <= 1.0; x += 0.05) {
    target_pts.emplace_back(x, 1.0);
  }
  matcher.set_target_cloud(target_pts);

  auto scan = make_box_scan();
  core::OdomData initial_guess{100.0, 0.15, -0.10, 0.05};

  auto result = matcher.match(scan, initial_guess);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.08);
  EXPECT_NEAR(result.dy, 0.0, 0.08);
  EXPECT_NEAR(result.dyaw, 0.0, 0.05);
}

// 常に非収束を返すモックマッチャー
class MockFailMatcher : public ScanMatcherBase {
 public:
  void set_target_cloud(const std::vector<Eigen::Vector2d>&) override {}
  core::MatchResult match(
      const core::ConstScanDataPtr&,
      const core::OdomData& initial_guess) override {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }
};

TEST(ScanMatchingBuilderTest, NonConvergenceFallsBackToOdometryWithoutDropping) {
  auto mock_matcher = std::make_shared<MockFailMatcher>();
  auto provider = std::make_shared<LocalMapProvider>(10, 20.0);

  pose_graph::ScanMatchingBuilder builder(mock_matcher, provider, 0.2, 0.1, 5);

  auto scan1 = make_box_scan();
  core::OdomData odom1{100.0, 0.0, 0.0, 0.0};
  auto node1 = builder.add_scan(scan1, odom1);
  ASSERT_TRUE(node1.has_value());
  EXPECT_EQ(node1->index, 0);

  // 0.25m 移動（キーフレーム閾値超え）
  auto scan2 = make_box_scan();
  core::OdomData odom2{100.1, 0.25, 0.0, 0.0};
  auto node2 = builder.add_scan(scan2, odom2);

  // マッチャーが非収束でも、オドメトリフォールバックによりノードが必ず作成される！
  ASSERT_TRUE(node2.has_value());
  EXPECT_EQ(node2->index, 1);
  EXPECT_NEAR(node2->x, 0.25, 1e-4);
  EXPECT_NEAR(node2->y, 0.0, 1e-4);

  // エッジがオドメトリフォールバックとして記録されていることを確認
  auto edges = builder.get_edges();
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_TRUE(edges[0].is_odom_fallback);
  EXPECT_NEAR(edges[0].dx, 0.25, 1e-4);
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
