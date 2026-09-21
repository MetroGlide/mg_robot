#include <gtest/gtest.h>

#include "slam_gnss_2d/input/time_utils.hpp"

using namespace slam_gnss_2d::input;

TEST(BagTimeUtilsTest, DefaultRangeUnspecified) {
  double bag_start = 1000.0;
  auto range = compute_bag_time_range(bag_start, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(range.bag_start_sec, 1000.0);
  EXPECT_DOUBLE_EQ(range.target_start_sec, 0.0);
  EXPECT_DOUBLE_EQ(range.target_end_sec, 0.0);

  EXPECT_FALSE(range.is_before_start(999.0));
  EXPECT_FALSE(range.is_past_end(2000.0));
  EXPECT_TRUE(range.is_in_range(500.0));
  EXPECT_TRUE(range.is_in_range(1500.0));
}

TEST(BagTimeUtilsTest, StartTimeOnly) {
  double bag_start = 1000.0;
  auto range = compute_bag_time_range(bag_start, 10.0, 0.0);

  EXPECT_DOUBLE_EQ(range.target_start_sec, 1010.0);
  EXPECT_DOUBLE_EQ(range.target_end_sec, 0.0);

  EXPECT_TRUE(range.is_before_start(1005.0));
  EXPECT_FALSE(range.is_before_start(1010.0));
  EXPECT_FALSE(range.is_before_start(1015.0));

  EXPECT_FALSE(range.is_past_end(2000.0));

  EXPECT_FALSE(range.is_in_range(1005.0));
  EXPECT_TRUE(range.is_in_range(1010.0));
  EXPECT_TRUE(range.is_in_range(2000.0));

  // Margin check
  EXPECT_TRUE(range.is_in_range(1007.0, 5.0));
  EXPECT_FALSE(range.is_in_range(1004.0, 5.0));
}

TEST(BagTimeUtilsTest, EndTimeOnly) {
  double bag_start = 1000.0;
  auto range = compute_bag_time_range(bag_start, 0.0, 50.0);

  EXPECT_DOUBLE_EQ(range.target_start_sec, 0.0);
  EXPECT_DOUBLE_EQ(range.target_end_sec, 1050.0);

  EXPECT_FALSE(range.is_before_start(900.0));

  EXPECT_FALSE(range.is_past_end(1049.0));
  EXPECT_FALSE(range.is_past_end(1050.0));
  EXPECT_TRUE(range.is_past_end(1050.001));

  EXPECT_TRUE(range.is_in_range(1020.0));
  EXPECT_FALSE(range.is_in_range(1055.0));
  EXPECT_TRUE(range.is_in_range(1053.0, 5.0));
}

TEST(BagTimeUtilsTest, BothStartAndEndTime) {
  double bag_start = 1000.0;
  auto range = compute_bag_time_range(bag_start, 10.0, 50.0);

  EXPECT_DOUBLE_EQ(range.target_start_sec, 1010.0);
  EXPECT_DOUBLE_EQ(range.target_end_sec, 1050.0);

  EXPECT_TRUE(range.is_before_start(1009.9));
  EXPECT_FALSE(range.is_before_start(1010.0));

  EXPECT_FALSE(range.is_past_end(1050.0));
  EXPECT_TRUE(range.is_past_end(1050.1));

  EXPECT_FALSE(range.is_in_range(1005.0));
  EXPECT_TRUE(range.is_in_range(1020.0));
  EXPECT_FALSE(range.is_in_range(1055.0));
}

TEST(BagTimeUtilsTest, InvalidEndTimeIgnored) {
  double bag_start = 1000.0;
  auto range = compute_bag_time_range(bag_start, 30.0, 20.0);

  EXPECT_DOUBLE_EQ(range.target_start_sec, 1030.0);
  EXPECT_DOUBLE_EQ(range.target_end_sec, 0.0);
}
