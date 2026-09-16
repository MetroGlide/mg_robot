#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace input {

template <typename T>
std::optional<T> nearest_by_timestamp(
    const std::vector<T>& items,
    const std::vector<double>& timestamps,
    double timestamp,
    std::optional<double> max_dt = std::nullopt) {
  if (items.empty() || timestamps.empty() || items.size() != timestamps.size()) {
    return std::nullopt;
  }
  auto it = std::lower_bound(timestamps.begin(), timestamps.end(), timestamp);
  size_t idx = std::distance(timestamps.begin(), it);
  T best;
  double best_dt = 0.0;
  if (idx == 0) {
    best = items[0];
    best_dt = std::abs(timestamp - timestamps[0]);
  } else if (idx >= timestamps.size()) {
    best = items.back();
    best_dt = std::abs(timestamp - timestamps.back());
  } else {
    double dt_prev = std::abs(timestamp - timestamps[idx - 1]);
    double dt_next = std::abs(timestamp - timestamps[idx]);
    if (dt_prev <= dt_next) {
      best = items[idx - 1];
      best_dt = dt_prev;
    } else {
      best = items[idx];
      best_dt = dt_next;
    }
  }
  if (max_dt.has_value() && best_dt > *max_dt) {
    return std::nullopt;
  }
  return best;
}

std::optional<core::OdomData> interpolate_odom(
    const std::vector<core::OdomData>& odom_list,
    const std::vector<double>& timestamps,
    double timestamp);

std::optional<core::GnssData> interpolate_gnss(
    const std::vector<core::GnssData>& gnss_list,
    const std::vector<double>& timestamps,
    double timestamp,
    std::optional<double> max_dt = std::nullopt);

}  // namespace input
}  // namespace slam_gnss_2d
