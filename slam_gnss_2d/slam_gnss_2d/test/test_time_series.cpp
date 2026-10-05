#include <gtest/gtest.h>
#include <vector>
#include <string>

#include "slam_gnss_2d/input/time_series.hpp"

namespace slam_gnss_2d {
namespace input {

TEST(TimeSeriesTest, NearestByTimestampWithMaxDt) {
  std::vector<std::string> items = {"a", "b", "c"};
  std::vector<double> timestamps = {10.0, 20.0, 30.0};

  // Within max_dt
  auto res1 = nearest_by_timestamp(items, timestamps, 20.5, 1.0);
  ASSERT_TRUE(res1.has_value());
  EXPECT_EQ(*res1, "b");

  auto res2 = nearest_by_timestamp(items, timestamps, 19.5, 1.0);
  ASSERT_TRUE(res2.has_value());
  EXPECT_EQ(*res2, "b");

  // Exceeding max_dt
  auto res3 = nearest_by_timestamp(items, timestamps, 25.0, 2.0);
  EXPECT_FALSE(res3.has_value());

  auto res4 = nearest_by_timestamp(items, timestamps, 50.0, 5.0);
  EXPECT_FALSE(res4.has_value());

  auto res5 = nearest_by_timestamp(items, timestamps, 0.0, 5.0);
  EXPECT_FALSE(res5.has_value());
}

TEST(TimeSeriesTest, InterpolateGnssWithMaxDt) {
  GnssData gnss0;
  gnss0.timestamp = 10.0;
  gnss0.x = 0.0;
  gnss0.y = 0.0;
  gnss0.covariance = Eigen::Matrix2d::Identity();
  gnss0.fix_status = 2;

  GnssData gnss1;
  gnss1.timestamp = 20.0;
  gnss1.x = 10.0;
  gnss1.y = 10.0;
  gnss1.covariance = Eigen::Matrix2d::Identity();
  gnss1.fix_status = 2;

  std::vector<GnssData> gnss_list = {gnss0, gnss1};
  std::vector<double> timestamps = {10.0, 20.0};

  // Normal interpolation
  auto interp = interpolate_gnss(gnss_list, timestamps, 15.0, 10.0);
  ASSERT_TRUE(interp.has_value());
  EXPECT_NEAR(interp->x, 5.0, 1e-6);
  EXPECT_NEAR(interp->y, 5.0, 1e-6);

  // Exceeding max_dt from query
  auto interp_too_far = interpolate_gnss(gnss_list, timestamps, 30.0, 5.0);
  EXPECT_FALSE(interp_too_far.has_value());
}

TEST(TimeSeriesTest, InterpolateGnssLargeGapRejected) {
  GnssData gnss0;
  gnss0.timestamp = 0.0;
  gnss0.x = 0.0;
  gnss0.y = 0.0;
  gnss0.fix_status = 2;

  GnssData gnss1;
  gnss1.timestamp = 20.0;
  gnss1.x = 20.0;
  gnss1.y = 20.0;
  gnss1.fix_status = 2;

  std::vector<GnssData> gnss_list = {gnss0, gnss1};
  std::vector<double> timestamps = {0.0, 20.0};

  // 20s gap between observations, max_dt=5.0s -> query at 3.0s should be rejected
  auto res = interpolate_gnss(gnss_list, timestamps, 3.0, 5.0);
  EXPECT_FALSE(res.has_value());
}

}  // namespace input
}  // namespace slam_gnss_2d
