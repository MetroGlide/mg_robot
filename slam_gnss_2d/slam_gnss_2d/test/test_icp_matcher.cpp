#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"
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

  // Add walls at x=1.0 and y=1.0
  for (int i = 0; i < 360; ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double ca = std::cos(angle);
    double sa = std::sin(angle);
    // Ray to line x = 1.0 (ca > 0.1)
    if (ca > 0.1 && std::abs(sa / ca) <= 1.0) {
      double r = 1.0 / ca;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
    // Ray to line y = 1.0 (sa > 0.1)
    if (sa > 0.1 && std::abs(ca / sa) <= 1.0) {
      double r = 1.0 / sa;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
  }
  return scan;
}

TEST(ICPMatcherTest, MatchAndInformationMatrixScaling) {
  ICPMatcher matcher(
      50,      // max_iterations
      1e-3,    // tolerance
      1.0,     // max_correspondence_dist
      "none",  // robust_kernel
      0.1,     // robust_kernel_scale
      5.0      // yaw_information_multiplier
  );

  // Target cloud: perpendicular walls
  std::vector<Eigen::Vector2d> target_pts;
  for (double y = -1.0; y <= 1.0; y += 0.05) {
    target_pts.emplace_back(1.0, y);
  }
  for (double x = 0.0; x <= 1.0; x += 0.05) {
    target_pts.emplace_back(x, 1.0);
  }
  matcher.set_target_cloud(target_pts);

  auto scan = make_box_scan();
  core::OdomData initial_guess{100.0, 0.02, -0.02, 0.01};

  auto result = matcher.match(scan, initial_guess);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.05);
  EXPECT_NEAR(result.dy, 0.0, 0.05);
  EXPECT_NEAR(result.dyaw, 0.0, 0.05);

  // Information matrix scaling check: norm of 2x2 translation block
  double trans_info_norm = result.information.block<2, 2>(0, 0).norm();
  EXPECT_GE(trans_info_norm, 50.0);
  EXPECT_LE(trans_info_norm, 50000.0);
}

TEST(LocalMapProviderTest, SyncPoses) {
  LocalMapProvider provider(10, 20.0);

  auto scan = make_box_scan();
  core::PoseNode n0{0, 100.0, 0.0, 0.0, 0.0, scan};
  provider.update(n0);

  auto pts0 = provider.get_reference_pts();
  ASSERT_TRUE(pts0.has_value());
  EXPECT_FALSE(pts0->empty());

  // n0 を回転・移動
  core::PoseNode n0_new{0, 100.0, 10.0, 20.0, M_PI_2, scan};
  provider.sync_poses({n0_new});

  // 最新ノードとして回転後の座標で更新
  core::PoseNode n1{1, 100.1, 10.0, 20.0, M_PI_2, nullptr};
  provider.update(n1);

  auto pts1 = provider.get_reference_pts();
  ASSERT_TRUE(pts1.has_value());
  EXPECT_EQ(pts1->size(), pts0->size());

  // n0とn1は同じ相対位置なので、局所座標系での参照点群は pts0 と一致するはず
  for (size_t i = 0; i < std::min(pts0->size(), size_t(5)); ++i) {
    EXPECT_NEAR((*pts0)[i].x(), (*pts1)[i].x(), 1e-4);
    EXPECT_NEAR((*pts0)[i].y(), (*pts1)[i].y(), 1e-4);
  }
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
