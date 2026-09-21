#include "slam_gnss_2d/input/time_series.hpp"

namespace slam_gnss_2d {
namespace input {

std::optional<core::OdomData> interpolate_odom(
    const std::vector<core::OdomData>& odom_list,
    const std::vector<double>& timestamps,
    double timestamp) {
  if (odom_list.empty() || timestamps.empty() || odom_list.size() != timestamps.size()) {
    return std::nullopt;
  }
  auto it = std::lower_bound(timestamps.begin(), timestamps.end(), timestamp);
  size_t idx = std::distance(timestamps.begin(), it);

  if (idx == 0) {
    return odom_list[0];
  }
  if (idx >= odom_list.size()) {
    return odom_list.back();
  }

  const auto& prev = odom_list[idx - 1];
  const auto& next = odom_list[idx];
  double t_span = next.timestamp - prev.timestamp;
  if (t_span < 1e-9) {
    return prev;
  }

  double alpha = (timestamp - prev.timestamp) / t_span;
  double interp_x = prev.x + alpha * (next.x - prev.x);
  double interp_y = prev.y + alpha * (next.y - prev.y);
  double interp_yaw = prev.yaw + alpha * core::angle_diff(next.yaw, prev.yaw);

  return core::OdomData{timestamp, interp_x, interp_y, interp_yaw};
}

std::optional<core::GnssData> interpolate_gnss(
    const std::vector<core::GnssData>& gnss_list,
    const std::vector<double>& timestamps,
    double timestamp,
    std::optional<double> max_dt) {
  if (gnss_list.empty() || timestamps.empty() || gnss_list.size() != timestamps.size()) {
    return std::nullopt;
  }
  auto it = std::lower_bound(timestamps.begin(), timestamps.end(), timestamp);
  size_t idx = std::distance(timestamps.begin(), it);

  if (idx == 0) {
    if (max_dt.has_value() && std::abs(timestamp - timestamps[0]) > *max_dt) {
      return std::nullopt;
    }
    return gnss_list[0];
  }
  if (idx >= gnss_list.size()) {
    if (max_dt.has_value() && std::abs(timestamp - timestamps.back()) > *max_dt) {
      return std::nullopt;
    }
    return gnss_list.back();
  }

  const auto& prev = gnss_list[idx - 1];
  const auto& next = gnss_list[idx];
  double t_span = next.timestamp - prev.timestamp;
  if (t_span < 1e-9) {
    return prev;
  }

  if (max_dt.has_value() &&
      (t_span > *max_dt ||
       std::abs(timestamp - prev.timestamp) > *max_dt ||
       std::abs(timestamp - next.timestamp) > *max_dt)) {
    return std::nullopt;
  }

  double alpha = (timestamp - prev.timestamp) / t_span;
  core::GnssData result;
  result.timestamp = timestamp;
  result.x = prev.x + alpha * (next.x - prev.x);
  result.y = prev.y + alpha * (next.y - prev.y);
  result.covariance = prev.covariance + alpha * (next.covariance - prev.covariance);
  result.fix_status = prev.fix_status;
  result.latitude = prev.latitude + alpha * (next.latitude - prev.latitude);
  result.longitude = prev.longitude + alpha * (next.longitude - prev.longitude);

  return result;
}

}  // namespace input
}  // namespace slam_gnss_2d
