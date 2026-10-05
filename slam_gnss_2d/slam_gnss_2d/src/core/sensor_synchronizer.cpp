#include "slam_gnss_2d/core/sensor_synchronizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/core/deskew.hpp"

namespace slam_gnss_2d {
namespace core {

SensorSynchronizer::SensorSynchronizer(
    std::shared_ptr<input::ScanSourceBase> scan_source,
    std::shared_ptr<input::OdomSourceBase> odom_source,
    std::shared_ptr<input::GnssSourceBase> gnss_source,
    DeskewConfig deskew_config)
    : scan_source_(scan_source),
      odom_source_(odom_source),
      gnss_source_(gnss_source),
      deskew_config_(deskew_config) {
  scan_source_->set_scan_callback(
      [this](const ScanDataPtr& scan) { this->on_scan(scan); });
}

void SensorSynchronizer::set_frame_callback(
    std::function<void(const SensorFrame&)> callback) {
  frame_callback_ = callback;
}

void SensorSynchronizer::start() {
  odom_source_->start();
  if (gnss_source_) {
    gnss_source_->start();
  }
  scan_source_->start();
}

void SensorSynchronizer::stop() {
  if (gnss_source_) {
    gnss_source_->stop();
  }
  scan_source_->stop();
  odom_source_->stop();
}

SynchronizerStats SensorSynchronizer::get_stats() const {
  return SynchronizerStats{scan_recv_count_, odom_miss_count_};
}

void SensorSynchronizer::on_scan(const ScanDataPtr& scan) {
  scan_recv_count_++;
  auto odom = odom_source_->get_odom_at(scan->timestamp);
  if (!odom.has_value()) {
    odom_miss_count_++;
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.sensor_synchronizer"),
        "No odom for scan ts=%.3f (miss #%d)",
        scan->timestamp, odom_miss_count_);
    return;
  }

  deskew_scan(
      *scan,
      [this](double t) { return odom_source_->get_odom_at(t); },
      deskew_config_);

  std::optional<GnssData> gnss = std::nullopt;
  if (gnss_source_) {
    gnss = gnss_source_->get_gnss_at(scan->timestamp);
  }

  SensorFrame frame{scan, *odom, gnss};
  if (frame_callback_) {
    frame_callback_(frame);
  }
}

}  // namespace core
}  // namespace slam_gnss_2d
