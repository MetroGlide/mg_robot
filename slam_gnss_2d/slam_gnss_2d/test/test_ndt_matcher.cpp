#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/scan_matching/ndt_matcher.hpp"

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
  scan->ranges.assign(360, 50.0f);  // Beyond range_max -> ignored by default

  for (int i = 0; i < 360; ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double ca = std::cos(angle);
    double sa = std::sin(angle);
    // Wall at x = 2.0
    if (ca > 0.1 && std::abs(sa / ca) <= 1.0) {
      double r = 2.0 / ca;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
    // Wall at x = -2.0
    if (ca < -0.1 && std::abs(sa / ca) <= 1.0) {
      double r = -2.0 / ca;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
    // Wall at y = 2.0
    if (sa > 0.1 && std::abs(ca / sa) <= 1.0) {
      double r = 2.0 / sa;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
    // Wall at y = -2.0
    if (sa < -0.1 && std::abs(ca / sa) <= 1.0) {
      double r = -2.0 / sa;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
  }
  return scan;
}

TEST(NDTMatcherTest, MultiResolutionMatch) {
  NDTMatcher matcher(
      50,                 // max_iterations
      1e-3,               // tolerance
      {2.0, 1.0, 0.5},    // cell_sizes
      true,               // use_bilinear
      1.0                 // yaw_information_multiplier
  );

  auto scan = make_box_scan();
  auto target_pts = core::scan_to_points(scan);
  ASSERT_GE(target_pts.size(), 20u);
  matcher.set_target_cloud(target_pts);

  core::OdomData initial_guess{100.0, 0.05, -0.05, 0.02};

  auto result = matcher.match(scan, initial_guess);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.08);
  EXPECT_NEAR(result.dy, 0.0, 0.08);
  EXPECT_NEAR(result.dyaw, 0.0, 0.05);
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
