#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/gnss/utm.hpp"
#include "slam_gnss_2d/input/base.hpp"
#include "slam_gnss_2d/input/time_utils.hpp"

namespace slam_gnss_2d {
namespace input {

class BagScanSource : public ScanSourceBase {
 public:
  BagScanSource(
      const std::string& bag_path,
      const std::string& scan_topic,
      double start_time = 0.0,
      double end_time = 0.0);

  void set_scan_callback(
      std::function<void(const core::ScanDataPtr&)> callback) override;
  void start() override;
  void stop() override;
  bool step();

 private:
  std::string bag_path_;
  std::string scan_topic_;
  double start_time_{0.0};
  double end_time_{0.0};
  BagTimeRange time_range_;
  std::function<void(const core::ScanDataPtr&)> callback_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;

  double lidar_yaw_{0.0};
  double lidar_x_{0.0};
  double lidar_y_{0.0};
  int step_count_{0};

  std::tuple<double, double, double> resolve_lidar_tf();
};

class BagOdomSource : public OdomSourceBase {
 public:
  BagOdomSource(
      const std::string& bag_path,
      const std::string& odom_topic,
      double start_time = 0.0,
      double end_time = 0.0);

  void start() override;
  void stop() override;
  std::optional<core::OdomData> get_odom_at(double timestamp) override;

 private:
  std::string bag_path_;
  std::string odom_topic_;
  double start_time_{0.0};
  double end_time_{0.0};
  std::vector<core::OdomData> odom_list_;
  std::vector<double> timestamps_;
};

class BagGnssSource : public GnssSourceBase {
 public:
  BagGnssSource(
      const std::string& bag_path,
      const std::string& gnss_topic,
      double start_time = 0.0,
      double end_time = 0.0);

  void start() override;
  void stop() override;
  std::optional<core::GnssData> get_gnss_at(double timestamp) override;
  std::vector<core::GnssData> get_all_gnss() override;

 private:
  std::string bag_path_;
  std::string gnss_topic_;
  double start_time_{0.0};
  double end_time_{0.0};
  std::vector<core::GnssData> gnss_list_;
  std::vector<double> timestamps_;
};

class BagNavPVTSource : public GnssSourceBase {
 public:
  BagNavPVTSource(
      const std::string& bag_path,
      const std::string& navpvt_topic,
      double hacc_scale = 1.0,
      double start_time = 0.0,
      double end_time = 0.0);

  void start() override;
  void stop() override;
  std::optional<core::GnssData> get_gnss_at(double timestamp) override;
  std::vector<core::GnssData> get_all_gnss() override;

 private:
  std::string bag_path_;
  std::string navpvt_topic_;
  double hacc_scale_;
  double start_time_{0.0};
  double end_time_{0.0};
  std::vector<core::GnssData> gnss_list_;
  std::vector<double> timestamps_;
};

}  // namespace input
}  // namespace slam_gnss_2d
