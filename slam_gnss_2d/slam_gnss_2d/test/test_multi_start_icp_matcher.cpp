#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/scan_matching/multi_start_icp_matcher.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

// 直線廊下（幅3m, 長さ20m）の点群を生成するヘルパー
static core::ScanDataPtr make_corridor_scan(double sensor_x, double sensor_y, double sensor_yaw) {
  auto scan = std::make_shared<core::ScanData>();
  scan->timestamp = 100.0;
  scan->angle_min = -M_PI;
  scan->angle_increment = 2.0 * M_PI / 720.0;  // 0.5度刻み
  scan->range_min = 0.1;
  scan->range_max = 30.0;
  scan->lidar_x = 0.0;
  scan->lidar_y = 0.0;
  scan->lidar_yaw = 0.0;
  scan->ranges.assign(720, 30.0f);

  // 左右の壁: y = +1.5, y = -1.5 (x: -10 to +10)
  for (int i = 0; i < 720; ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double global_beam_angle = sensor_yaw + angle;
    double ca = std::cos(global_beam_angle);
    double sa = std::sin(global_beam_angle);

    // 上の壁 (y = 1.5) との交点
    if (sa > 1e-4) {
      double r = (1.5 - sensor_y) / sa;
      double hit_x = sensor_x + r * ca;
      if (r > 0.1 && r < scan->ranges[i] && hit_x >= -10.0 && hit_x <= 10.0) {
        scan->ranges[i] = static_cast<float>(r);
      }
    }
    // 下の壁 (y = -1.5) との交点
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

// ターゲット点群（廊下の壁面点群）を生成
static std::vector<Eigen::Vector2d> make_corridor_target() {
  std::vector<Eigen::Vector2d> pts;
  for (double x = -10.0; x <= 10.0; x += 0.1) {
    pts.emplace_back(x, 1.5);
    pts.emplace_back(x, -1.5);
  }
  return pts;
}

TEST(MultiStartICPMatcherTest, RecoversFromAngularSlipInStraightCorridor) {
  // 廊下環境でロボットが直進しているが、オドメトリに片輪スリップ (-4.5度 = -0.0785 rad) が生じたケース
  MultiStartICPMatcher matcher(
      100, 1e-3, 2.0, "huber", 0.15, 1.0,
      {-4.0, -2.0, 2.0, 4.0}, true);

  auto target_pts = make_corridor_target();
  matcher.set_target_cloud(target_pts);

  // ロボットの真の位置: (x=0.2, y=0.0, yaw=0.0) - 直進
  auto scan = make_corridor_scan(0.2, 0.0, 0.0);

  // オドメトリ初期値: 片輪スリップによる誤った旋回角 -4.5度
  double slip_dyaw = -4.5 * M_PI / 180.0;  // -0.0785 rad
  core::OdomData initial_guess{100.0, 0.2, 0.0, slip_dyaw};

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  // スリップによる旋回角に引きずられず、真の直進角度 (約0.0) に収束すること
  EXPECT_NEAR(result.dx, 0.2, 0.05);
  EXPECT_NEAR(result.dy, 0.0, 0.05);
  EXPECT_NEAR(result.dyaw, 0.0, 0.02);  // 0.02 rad = 約 1.1度以内
}

TEST(MultiStartICPMatcherTest, NormalMotionConvergence) {
  MultiStartICPMatcher matcher(
      100, 1e-3, 2.0, "huber", 0.15, 1.0,
      {-4.0, -2.0, 2.0, 4.0}, true);

  auto target_pts = make_corridor_target();
  matcher.set_target_cloud(target_pts);

  // ロボットが実際にわずかに斜めに移動したケース (x=0.2, y=0.03, yaw=0.02)
  auto scan = make_corridor_scan(0.2, 0.03, 0.02);
  core::OdomData initial_guess{100.0, 0.18, 0.02, 0.015};

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.2, 0.05);
  EXPECT_NEAR(result.dy, 0.03, 0.05);
  EXPECT_NEAR(result.dyaw, 0.02, 0.02);
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
