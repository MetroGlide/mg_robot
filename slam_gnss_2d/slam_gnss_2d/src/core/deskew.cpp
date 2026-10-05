#include "slam_gnss_2d/core/deskew.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace core {

namespace {

// 走査時間内でオドメトリ姿勢を評価するサンプル数 (区間数 = kSamples - 1)
constexpr int kSamples = 11;

struct RelativePose {
  double dx;
  double dy;
  double dyaw;
};

}  // namespace

bool deskew_scan(ScanData& scan, const OdomLookup& odom_at, const DeskewConfig& config) {
  const size_t n = scan.ranges.size();
  const double duration = config.duration_s > 0.0 ? config.duration_s : scan.scan_duration;
  if (!config.enabled || config.direction == 0 || n < 2 || duration <= 0.0) {
    return false;
  }

  const auto reference = odom_at(scan.timestamp);
  if (!reference.has_value()) {
    return false;
  }

  // 走査時間内のオドメトリ姿勢を、header.stamp 時点のロボット座標系での相対姿勢に変換する
  std::array<RelativePose, kSamples> samples;
  for (int k = 0; k < kSamples; ++k) {
    const double t = scan.timestamp + config.start_offset_s +
                     duration * static_cast<double>(k) / static_cast<double>(kSamples - 1);
    const auto odom = odom_at(t);
    if (!odom.has_value()) {
      return false;
    }
    auto [dx, dy, dyaw] = delta_pose(
        reference->x, reference->y, reference->yaw, odom->x, odom->y, odom->yaw);
    samples[k] = RelativePose{dx, dy, dyaw};
  }

  std::vector<float> angles(n);
  for (size_t i = 0; i < n; ++i) {
    const double angle = beam_angle(scan, i);
    angles[i] = static_cast<float>(angle);

    const float r = scan.ranges[i];
    if (!(r >= scan.range_min && r <= scan.range_max)) {
      continue;
    }

    // ビームの計測時刻に対応する走査内の位置 (0: 最初に計測, 1: 最後に計測)
    const double frac = config.direction > 0
        ? static_cast<double>(i) / static_cast<double>(n - 1)
        : static_cast<double>(n - 1 - i) / static_cast<double>(n - 1);
    const double u = frac * static_cast<double>(kSamples - 1);
    const int k = std::min(static_cast<int>(u), kSamples - 2);
    const double w = u - static_cast<double>(k);

    const RelativePose& a = samples[k];
    const RelativePose& b = samples[k + 1];
    const double dx = a.dx + w * (b.dx - a.dx);
    const double dy = a.dy + w * (b.dy - a.dy);
    const double dyaw = a.dyaw + w * angle_diff(b.dyaw, a.dyaw);

    // 計測時刻のロボット座標系での終端点を、header.stamp 時点のロボット座標系へ変換する
    const double px = scan.lidar_x + r * std::cos(angle);
    const double py = scan.lidar_y + r * std::sin(angle);
    const double c = std::cos(dyaw);
    const double s = std::sin(dyaw);
    const double rx = c * px - s * py + dx - scan.lidar_x;
    const double ry = s * px + c * py + dy - scan.lidar_y;

    scan.ranges[i] = static_cast<float>(std::hypot(rx, ry));
    angles[i] = static_cast<float>(std::atan2(ry, rx));
  }

  scan.angles = std::move(angles);
  return true;
}

}  // namespace core
}  // namespace slam_gnss_2d
