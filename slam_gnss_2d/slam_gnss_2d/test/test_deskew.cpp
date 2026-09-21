#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "slam_gnss_2d/core/deskew.hpp"
#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace core {

namespace {

constexpr double kHalfSize = 5.0;
constexpr double kDuration = 0.1;
constexpr double kYawRate = 2.0;  // [rad/s]
constexpr size_t kNumBeams = 720;

// 一辺 2*kHalfSize の正方形の部屋の壁までの距離 (原点から方向 phi)
double wall_range(double phi) {
  return kHalfSize / std::max(std::abs(std::cos(phi)), std::abs(std::sin(phi)));
}

// ロボットがその場で一定角速度 kYawRate で回転している間に走査したスキャンを作る。
// direction: +1 はビーム番号が増えるほど後に計測、-1 は先に計測
ScanData make_rotating_scan(int direction) {
  ScanData scan;
  scan.timestamp = 100.0;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / static_cast<double>(kNumBeams);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.scan_duration = kDuration;
  scan.ranges.resize(kNumBeams);
  for (size_t i = 0; i < kNumBeams; ++i) {
    const double frac = direction > 0
        ? static_cast<double>(i) / static_cast<double>(kNumBeams - 1)
        : static_cast<double>(kNumBeams - 1 - i) / static_cast<double>(kNumBeams - 1);
    const double body_yaw = kYawRate * kDuration * frac;
    const double beam = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    scan.ranges[i] = static_cast<float>(wall_range(beam + body_yaw));
  }
  return scan;
}

OdomLookup rotating_odom(double stamp) {
  return [stamp](double t) -> std::optional<OdomData> {
    return OdomData{t, 0.0, 0.0, kYawRate * (t - stamp)};
  };
}

// 全点が壁 (正方形の境界) からどれだけずれているかの最大値
double max_wall_error(const ScanData& scan) {
  double worst = 0.0;
  for (const auto& p : scan_to_points(scan)) {
    worst = std::max(worst, std::abs(std::max(std::abs(p.x()), std::abs(p.y())) - kHalfSize));
  }
  return worst;
}

}  // namespace

TEST(DeskewTest, RemovesRotationDistortionWithMatchingDirection) {
  for (int direction : {1, -1}) {
    ScanData scan = make_rotating_scan(direction);
    EXPECT_GT(max_wall_error(scan), 0.3) << "補正前は歪んでいるはず direction=" << direction;

    DeskewConfig config;
    config.enabled = true;
    config.direction = direction;
    config.start_offset_s = 0.0;
    ASSERT_TRUE(deskew_scan(scan, rotating_odom(scan.timestamp), config));
    EXPECT_LT(max_wall_error(scan), 1e-3) << "direction=" << direction;
  }
}

TEST(DeskewTest, WrongDirectionDoesNotRemoveDistortion) {
  ScanData scan = make_rotating_scan(1);
  DeskewConfig config;
  config.enabled = true;
  config.direction = -1;
  config.start_offset_s = 0.0;
  ASSERT_TRUE(deskew_scan(scan, rotating_odom(scan.timestamp), config));
  EXPECT_GT(max_wall_error(scan), 0.3);
}

TEST(DeskewTest, CorrectsTranslationOfMovingRobot) {
  // 前進 (x 方向 1.0 m/s) しながら走査。壁は正方形の部屋 (ロボットは走査開始時に原点)
  ScanData scan;
  scan.timestamp = 100.0;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / static_cast<double>(kNumBeams);
  scan.range_min = 0.1;
  scan.range_max = 30.0;
  scan.scan_duration = kDuration;
  scan.ranges.resize(kNumBeams);
  const double speed = 1.0;
  for (size_t i = 0; i < kNumBeams; ++i) {
    const double frac = static_cast<double>(i) / static_cast<double>(kNumBeams - 1);
    const double x0 = speed * kDuration * frac;  // 計測時のロボット位置
    const double beam = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    // 位置 (x0, 0) から方向 beam への壁までの距離
    const double dx = std::cos(beam);
    const double dy = std::sin(beam);
    double t = 1e9;
    if (dx > 1e-9) t = std::min(t, (kHalfSize - x0) / dx);
    if (dx < -1e-9) t = std::min(t, (-kHalfSize - x0) / dx);
    if (dy > 1e-9) t = std::min(t, kHalfSize / dy);
    if (dy < -1e-9) t = std::min(t, -kHalfSize / dy);
    scan.ranges[i] = static_cast<float>(t);
  }
  auto odom = [&](double t) -> std::optional<OdomData> {
    return OdomData{t, speed * (t - scan.timestamp), 0.0, 0.0};
  };

  DeskewConfig config;
  config.enabled = true;
  config.direction = 1;
  config.start_offset_s = 0.0;
  ASSERT_TRUE(deskew_scan(scan, odom, config));
  EXPECT_LT(max_wall_error(scan), 1e-3);
}

TEST(DeskewTest, StartOffsetShiftsBeamTimes) {
  // 走査が stamp より 0.02 s 早く始まる場合 (start_offset_s = -0.02) の補正
  ScanData scan = make_rotating_scan(1);
  const double offset = -0.02;
  auto shifted_odom = [&](double t) -> std::optional<OdomData> {
    // 実際の姿勢は、ビーム計測時刻 (stamp + offset + frac*T) の yaw = rate * (frac*T)
    return OdomData{t, 0.0, 0.0, kYawRate * (t - scan.timestamp - offset)};
  };
  DeskewConfig config;
  config.enabled = true;
  config.direction = 1;
  config.start_offset_s = offset;
  ASSERT_TRUE(deskew_scan(scan, shifted_odom, config));
  // 点群は stamp 時点のロボット座標系 (走査開始時の姿勢から yaw = -rate*offset だけ回転) で
  // 表現されるので、走査開始時の座標系 (壁が軸に揃う) へ戻して確認する
  const double ref_yaw = -kYawRate * offset;
  double worst = 0.0;
  for (const auto& p : scan_to_points(scan)) {
    const double c = std::cos(ref_yaw);
    const double s = std::sin(ref_yaw);
    const double x = c * p.x() - s * p.y();
    const double y = s * p.x() + c * p.y();
    worst = std::max(worst, std::abs(std::max(std::abs(x), std::abs(y)) - kHalfSize));
  }
  EXPECT_LT(worst, 1e-3);
}

TEST(DeskewTest, DisabledOrUnknownDurationLeavesScanUntouched) {
  ScanData scan = make_rotating_scan(1);
  const auto ranges_before = scan.ranges;

  DeskewConfig disabled;
  disabled.enabled = false;
  EXPECT_FALSE(deskew_scan(scan, rotating_odom(scan.timestamp), disabled));

  DeskewConfig enabled;
  enabled.enabled = true;
  scan.scan_duration = 0.0;
  EXPECT_FALSE(deskew_scan(scan, rotating_odom(scan.timestamp), enabled));
  enabled.duration_s = kDuration;
  auto no_odom = [](double) -> std::optional<OdomData> { return std::nullopt; };
  EXPECT_FALSE(deskew_scan(scan, no_odom, enabled));

  EXPECT_EQ(scan.ranges, ranges_before);
  EXPECT_TRUE(scan.angles.empty());
}

}  // namespace core
}  // namespace slam_gnss_2d
