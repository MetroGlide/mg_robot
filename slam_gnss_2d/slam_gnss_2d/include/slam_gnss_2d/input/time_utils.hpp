#pragma once

#include <iostream>

namespace slam_gnss_2d {
namespace input {

struct BagTimeRange {
  double bag_start_sec{0.0};
  double target_start_sec{0.0};  // 0.0: 最初から
  double target_end_sec{0.0};    // 0.0: 最後まで

  bool is_before_start(double timestamp) const {
    return target_start_sec > 0.0 && timestamp < target_start_sec;
  }

  bool is_past_end(double timestamp) const {
    return target_end_sec > 0.0 && timestamp > target_end_sec;
  }

  bool is_in_range(double timestamp, double margin_sec = 0.0) const {
    if (target_start_sec > 0.0 && timestamp < (target_start_sec - margin_sec)) {
      return false;
    }
    if (target_end_sec > 0.0 && timestamp > (target_end_sec + margin_sec)) {
      return false;
    }
    return true;
  }
};

inline BagTimeRange compute_bag_time_range(
    double bag_start_sec,
    double start_time_sec,
    double end_time_sec) {
  BagTimeRange range;
  range.bag_start_sec = bag_start_sec;

  if (start_time_sec > 0.0) {
    range.target_start_sec = bag_start_sec + start_time_sec;
  }

  if (end_time_sec > 0.0) {
    if (start_time_sec > 0.0 && end_time_sec <= start_time_sec) {
      std::cerr << "[slam_gnss_2d.time_utils] Warning: end_time ("
                << end_time_sec << "s) <= start_time (" << start_time_sec
                << "s). Ignoring end_time limit." << std::endl;
      range.target_end_sec = 0.0;
    } else {
      range.target_end_sec = bag_start_sec + end_time_sec;
    }
  }

  return range;
}

}  // namespace input
}  // namespace slam_gnss_2d
