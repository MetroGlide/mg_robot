#include <gtest/gtest.h>
#include <cmath>
#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {

TEST(GeometryTest, NormalizeAngle) {
  EXPECT_NEAR(normalize_angle(0.0), 0.0, 1e-6);
  EXPECT_NEAR(normalize_angle(M_PI), M_PI, 1e-6);
  EXPECT_NEAR(normalize_angle(-M_PI), -M_PI, 1e-6);
  EXPECT_NEAR(normalize_angle(3.0 * M_PI), M_PI, 1e-6);
  EXPECT_NEAR(normalize_angle(-3.0 * M_PI), -M_PI, 1e-6);
  EXPECT_NEAR(normalize_angle(2.0 * M_PI), 0.0, 1e-6);
}

TEST(GeometryTest, ComposeAndDeltaPose) {
  // Pose 1: (1.0, 2.0, pi/2)
  // Relative: (0.5, 0.0, 0.0) -> in world: (1.0, 2.5, pi/2)
  auto [x2, y2, yaw2] = compose_pose(1.0, 2.0, M_PI_2, 0.5, 0.0, 0.0);
  EXPECT_NEAR(x2, 1.0, 1e-6);
  EXPECT_NEAR(y2, 2.5, 1e-6);
  EXPECT_NEAR(yaw2, M_PI_2, 1e-6);

  // Delta pose from (1, 2, pi/2) to (1, 2.5, pi/2)
  auto [dx, dy, dyaw] = delta_pose(1.0, 2.0, M_PI_2, 1.0, 2.5, M_PI_2);
  EXPECT_NEAR(dx, 0.5, 1e-6);
  EXPECT_NEAR(dy, 0.0, 1e-6);
  EXPECT_NEAR(dyaw, 0.0, 1e-6);
}

TEST(GeometryTest, QuaternionYawConversion) {
  double yaw = 0.75;
  auto q = yaw_to_quaternion(yaw);
  double recovered_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);
  EXPECT_NEAR(recovered_yaw, yaw, 1e-6);
}

TEST(GeometryTest, ScanToPointsWithLidarOffset) {
  ScanData scan;
  scan.timestamp = 100.0;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 360.0;
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.ranges.assign(360, 1.0f);
  scan.lidar_x = 0.23;
  scan.lidar_y = 0.0;
  scan.lidar_yaw = 0.0;

  auto pts = scan_to_points(scan);
  EXPECT_FALSE(pts.empty());

  // Check the point at angle ~ 0 (forward)
  // angle = 0 is index 180
  int idx_zero = 180;
  double angle = scan.angle_min + idx_zero * scan.angle_increment;
  EXPECT_NEAR(angle, 0.0, 1e-2);

  // Expected point in robot frame: 1.0 * cos(0) + 0.23 = 1.23
  EXPECT_NEAR(pts[idx_zero].x(), 1.23, 0.02);
  EXPECT_NEAR(pts[idx_zero].y(), 0.0, 0.02);
}

}  // namespace slam_gnss_2d
