#include <gtest/gtest.h>

#include <cmath>

#include "slam_gnss_2d/gnss/nav_bridge_core.hpp"

namespace slam_gnss_2d {
namespace gnss {

TEST(NavBridgeCoreTest, CarrierFromFlags) {
  EXPECT_EQ(CarrierFromFlags(0x01), CarrierSolution::kNone);
  EXPECT_EQ(CarrierFromFlags(0x01 | (1 << 6)), CarrierSolution::kFloat);
  EXPECT_EQ(CarrierFromFlags(0x01 | (2 << 6)), CarrierSolution::kFixed);
  // 仕様にない値 (3) は使わない扱い
  EXPECT_EQ(CarrierFromFlags(0x01 | (3 << 6)), CarrierSolution::kNone);
}

TEST(NavBridgeCoreTest, DefaultVarianceIsHaccSquared) {
  PositionQualityConfig config;
  const auto fixed = PositionVariance(config, CarrierSolution::kFixed, 0.05);
  ASSERT_TRUE(fixed.has_value());
  EXPECT_NEAR(*fixed, 0.0025, 1e-12);
  const auto single = PositionVariance(config, CarrierSolution::kNone, 1.0);
  ASSERT_TRUE(single.has_value());
  EXPECT_NEAR(*single, 1.0, 1e-12);
}

TEST(NavBridgeCoreTest, FloorAndScaleApplyPerCarrier) {
  PositionQualityConfig config;
  config.fix_floor_m = 0.1;
  config.float_scale = 4.0;
  // Fix: hAcc が下限より小さいときは下限を使う
  EXPECT_NEAR(*PositionVariance(config, CarrierSolution::kFixed, 0.03), 0.01, 1e-12);
  // Float: 分散に係数がかかる
  EXPECT_NEAR(*PositionVariance(config, CarrierSolution::kFloat, 0.5), 1.0, 1e-12);
  // 単独測位には影響しない
  EXPECT_NEAR(*PositionVariance(config, CarrierSolution::kNone, 0.5), 0.25, 1e-12);
}

TEST(NavBridgeCoreTest, RejectsSingleWhenDisabledAndPoorAccuracy) {
  PositionQualityConfig config;
  config.accept_single = false;
  EXPECT_FALSE(PositionVariance(config, CarrierSolution::kNone, 0.5).has_value());
  EXPECT_TRUE(PositionVariance(config, CarrierSolution::kFloat, 0.5).has_value());
  config.max_covariance_threshold = 49.0;
  // 分散の 2 倍が 49 を超える (hAcc > 4.95 m) ものは使わない
  EXPECT_FALSE(PositionVariance(config, CarrierSolution::kFloat, 5.0).has_value());
  EXPECT_TRUE(PositionVariance(config, CarrierSolution::kFloat, 4.9).has_value());
}

TEST(NavBridgeCoreTest, AntennaToBaseRemovesLeverArm) {
  // 車体が原点で東向きなら、アンテナは (0.26, -0.13)。車体はそこから戻した位置
  const Point2 base = AntennaToBase({0.26, -0.13}, 0.0, 0.26, -0.13);
  EXPECT_NEAR(base.x, 0.0, 1e-12);
  EXPECT_NEAR(base.y, 0.0, 1e-12);
}

TEST(NavBridgeCoreTest, AntennaToBaseRotatesWithYaw) {
  const double yaw = M_PI / 2.0;  // 北向き
  // 北向きの車体の (0.26, -0.13) は、地図では (+0.13, +0.26)
  const Point2 antenna{10.0 + 0.13, 20.0 + 0.26};
  const Point2 base = AntennaToBase(antenna, yaw, 0.26, -0.13);
  EXPECT_NEAR(base.x, 10.0, 1e-12);
  EXPECT_NEAR(base.y, 20.0, 1e-12);
}

TEST(NavBridgeCoreTest, LeverArmUncertaintyVariance) {
  EXPECT_NEAR(LeverArmUncertaintyVariance(0.26, -0.13), 0.26 * 0.26 + 0.13 * 0.13, 1e-12);
}

}  // namespace gnss
}  // namespace slam_gnss_2d
