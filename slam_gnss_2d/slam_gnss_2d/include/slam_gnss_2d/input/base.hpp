#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace input {

class ScanSourceBase {
 public:
  virtual ~ScanSourceBase() = default;
  virtual void set_scan_callback(
      std::function<void(const core::ScanDataPtr&)> callback) = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
};

class OdomSourceBase {
 public:
  virtual ~OdomSourceBase() = default;
  virtual std::optional<core::OdomData> get_odom_at(double timestamp) = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
};

class GnssSourceBase {
 public:
  virtual ~GnssSourceBase() = default;
  virtual std::optional<core::GnssData> get_gnss_at(double timestamp) = 0;
  virtual std::vector<core::GnssData> get_all_gnss() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
};

}  // namespace input
}  // namespace slam_gnss_2d
