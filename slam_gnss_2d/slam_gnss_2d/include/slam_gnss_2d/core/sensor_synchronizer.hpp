#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/input/base.hpp"

namespace slam_gnss_2d {
namespace core {

struct SynchronizerStats {
  int scans{0};
  int odom_miss{0};
};

class SensorSynchronizer {
 public:
  SensorSynchronizer(
      std::shared_ptr<input::ScanSourceBase> scan_source,
      std::shared_ptr<input::OdomSourceBase> odom_source,
      std::shared_ptr<input::GnssSourceBase> gnss_source = nullptr);

  void set_frame_callback(std::function<void(const SensorFrame&)> callback);
  void start();
  void stop();
  SynchronizerStats get_stats() const;

 private:
  std::shared_ptr<input::ScanSourceBase> scan_source_;
  std::shared_ptr<input::OdomSourceBase> odom_source_;
  std::shared_ptr<input::GnssSourceBase> gnss_source_;
  std::function<void(const SensorFrame&)> frame_callback_;

  int scan_recv_count_{0};
  int odom_miss_count_{0};

  void on_scan(const ScanDataPtr& scan);
};

using SensorSynchronizerPtr = std::shared_ptr<SensorSynchronizer>;

}  // namespace core
}  // namespace slam_gnss_2d
