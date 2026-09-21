#include "slam_gnss_2d/input/ros2/bag_reader.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/buffer_core.h>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/input/time_series.hpp"

namespace slam_gnss_2d {
namespace input {

namespace {

std::unique_ptr<rosbag2_cpp::Reader> open_reader(
    const std::string& bag_path, const std::vector<std::string>& topics) {
  if (bag_path.empty()) {
    throw std::runtime_error("bag_path is empty. Please set ROSBAG_FILE environment variable or bag_path argument.");
  }
  if (!std::filesystem::exists(bag_path)) {
    throw std::runtime_error("Bag path does not exist: " + bag_path);
  }
  auto reader = std::make_unique<rosbag2_cpp::Reader>();
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader->open(storage_options, converter_options);

  if (!topics.empty()) {
    rosbag2_storage::StorageFilter filter;
    filter.topics = topics;
    reader->set_filter(filter);
  }
  return reader;
}

template <typename MsgT>
MsgT deserialize_bag_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_msg) {
  rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
  MsgT msg;
  rclcpp::Serialization<MsgT> serializer;
  serializer.deserialize_message(&serialized_msg, &msg);
  return msg;
}

double get_bag_start_time_sec(rosbag2_cpp::Reader& reader) {
  try {
    const auto& meta = reader.get_metadata();
    auto start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        meta.starting_time.time_since_epoch()).count();
    if (start_ns > 0) {
      return static_cast<double>(start_ns) * 1e-9;
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("slam_gnss_2d.bag_reader"),
                "Failed to get metadata starting_time: %s", e.what());
  }
  return 0.0;
}

}  // namespace

// -----------------------------------------------------------------------------
// BagScanSource
// -----------------------------------------------------------------------------
BagScanSource::BagScanSource(
    const std::string& bag_path,
    const std::string& scan_topic,
    double start_time,
    double end_time)
    : bag_path_(bag_path),
      scan_topic_(scan_topic),
      start_time_(start_time),
      end_time_(end_time) {}

void BagScanSource::set_scan_callback(
    std::function<void(const core::ScanDataPtr&)> callback) {
  callback_ = callback;
}

void BagScanSource::start() {
  std::tie(lidar_yaw_, lidar_x_, lidar_y_) = resolve_lidar_tf();
  reader_ = open_reader(bag_path_, {scan_topic_});

  double bag_start = get_bag_start_time_sec(*reader_);
  time_range_ = compute_bag_time_range(bag_start, start_time_, end_time_);

  if (time_range_.target_start_sec > 0.0) {
    double seek_target = std::max(time_range_.bag_start_sec, time_range_.target_start_sec - 1.0);
    int64_t seek_ns = static_cast<int64_t>(seek_target * 1e9);
    try {
      reader_->seek(seek_ns);
      RCLCPP_INFO(
          rclcpp::get_logger("slam_gnss_2d.bag_reader"),
          "Seeked bag to %.3f sec (start_time: +%.2fs)",
          seek_target, start_time_);
    } catch (const std::exception& ex) {
      RCLCPP_WARN(
          rclcpp::get_logger("slam_gnss_2d.bag_reader"),
          "Failed to seek bag: %s. Falling back to sequential skip.", ex.what());
    }
  }

  if (time_range_.target_end_sec > 0.0) {
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.bag_reader"),
        "Processing bag with time range: start=+%.2fs, end=+%.2fs",
        start_time_, end_time_);
  } else if (start_time_ > 0.0) {
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.bag_reader"),
        "Processing bag from start=+%.2fs to end of bag",
        start_time_);
  }
}

void BagScanSource::stop() {
  reader_.reset();
}

bool BagScanSource::step() {
  if (!reader_) {
    return false;
  }

  while (reader_->has_next()) {
    auto bag_msg = reader_->read_next();
    auto msg = deserialize_bag_message<sensor_msgs::msg::LaserScan>(bag_msg);
    double stamp = static_cast<double>(msg.header.stamp.sec) +
                   static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    if (time_range_.bag_start_sec <= 0.0) {
      time_range_ = compute_bag_time_range(stamp, start_time_, end_time_);
    }

    if (time_range_.is_before_start(stamp)) {
      continue;
    }

    if (time_range_.is_past_end(stamp)) {
      RCLCPP_INFO(
          rclcpp::get_logger("slam_gnss_2d.bag_reader"),
          "Reached bag end_time (+%.2fs, stamp=%.3f). Stopping scan stream.",
          end_time_, stamp);
      return false;
    }

    step_count_++;

    if (callback_) {
      auto scan = std::make_shared<core::ScanData>();
      scan->timestamp = stamp;
      scan->ranges = msg.ranges;
      scan->angle_min = msg.angle_min + lidar_yaw_;
      scan->angle_increment = msg.angle_increment;
      scan->range_min = msg.range_min;
      scan->range_max = msg.range_max;
      scan->lidar_x = lidar_x_;
      scan->lidar_y = lidar_y_;

      callback_(scan);
    }
    return true;
  }

  return false;
}

std::tuple<double, double, double> BagScanSource::resolve_lidar_tf() {
  auto reader = open_reader(bag_path_, {"/tf_static", scan_topic_});
  tf2::BufferCore tf_buffer;
  std::string scan_frame_id;
  int msg_count = 0;

  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    msg_count++;
    if (bag_msg->topic_name == "/tf_static") {
      auto tf_msg = deserialize_bag_message<tf2_msgs::msg::TFMessage>(bag_msg);
      for (const auto& tf : tf_msg.transforms) {
        tf_buffer.setTransform(tf, "bag_static", true);
      }
    } else if (bag_msg->topic_name == scan_topic_ && scan_frame_id.empty()) {
      auto scan_msg = deserialize_bag_message<sensor_msgs::msg::LaserScan>(bag_msg);
      scan_frame_id = scan_msg.header.frame_id;
    }

    if (!scan_frame_id.empty() && tf_buffer.canTransform("base_link", scan_frame_id, tf2::TimePointZero)) {
      break;
    }
    if (msg_count > 500 && !scan_frame_id.empty()) {
      break;
    }
  }

  if (scan_frame_id.empty()) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.bag_reader"),
        "No scan frame_id found in bag for topic '%s'. Fallback to (0,0,0)",
        scan_topic_.c_str());
    return {0.0, 0.0, 0.0};
  }

  try {
    auto tf = tf_buffer.lookupTransform("base_link", scan_frame_id, tf2::TimePointZero);
    double yaw = core::quaternion_to_yaw(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
    RCLCPP_INFO(
        rclcpp::get_logger("slam_gnss_2d.bag_reader"),
        "Resolved LiDAR TF for '%s' -> 'base_link': x=%.3f, y=%.3f, yaw=%.3f rad (%.1f deg)",
        scan_frame_id.c_str(), tf.transform.translation.x, tf.transform.translation.y,
        yaw, yaw * 180.0 / M_PI);
    return {yaw, tf.transform.translation.x, tf.transform.translation.y};
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(
        rclcpp::get_logger("slam_gnss_2d.bag_reader"),
        "Failed to lookup LiDAR TF for '%s' -> 'base_link': %s. Fallback to (0,0,0)",
        scan_frame_id.c_str(), ex.what());
    return {0.0, 0.0, 0.0};
  }
}

// -----------------------------------------------------------------------------
// BagOdomSource
// -----------------------------------------------------------------------------
BagOdomSource::BagOdomSource(
    const std::string& bag_path,
    const std::string& odom_topic,
    double start_time,
    double end_time)
    : bag_path_(bag_path),
      odom_topic_(odom_topic),
      start_time_(start_time),
      end_time_(end_time) {}

void BagOdomSource::start() {
  auto reader = open_reader(bag_path_, {odom_topic_});
  double bag_start = get_bag_start_time_sec(*reader);
  auto time_range = compute_bag_time_range(bag_start, start_time_, end_time_);

  if (time_range.target_start_sec > 0.0) {
    double seek_target = std::max(time_range.bag_start_sec, time_range.target_start_sec - 5.0);
    try {
      reader->seek(static_cast<int64_t>(seek_target * 1e9));
    } catch (...) {}
  }

  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    auto msg = deserialize_bag_message<nav_msgs::msg::Odometry>(bag_msg);
    double stamp = static_cast<double>(msg.header.stamp.sec) +
                   static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    if (time_range.bag_start_sec <= 0.0) {
      time_range = compute_bag_time_range(stamp, start_time_, end_time_);
    }

    if (!time_range.is_in_range(stamp, 5.0)) {
      if (time_range.is_past_end(stamp - 5.0)) {
        break;
      }
      continue;
    }

    double yaw = core::quaternion_to_yaw(
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    odom_list_.push_back(core::OdomData{
        stamp,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        yaw,
    });
  }
  std::sort(odom_list_.begin(), odom_list_.end(), [](const core::OdomData& a, const core::OdomData& b) {
    return a.timestamp < b.timestamp;
  });
  timestamps_.clear();
  timestamps_.reserve(odom_list_.size());
  for (const auto& o : odom_list_) {
    timestamps_.push_back(o.timestamp);
  }
}

void BagOdomSource::stop() {}

std::optional<core::OdomData> BagOdomSource::get_odom_at(double timestamp) {
  if (odom_list_.empty()) {
    return std::nullopt;
  }
  return interpolate_odom(odom_list_, timestamps_, timestamp);
}

// -----------------------------------------------------------------------------
// BagGnssSource
// -----------------------------------------------------------------------------
BagGnssSource::BagGnssSource(
    const std::string& bag_path,
    const std::string& gnss_topic,
    double start_time,
    double end_time)
    : bag_path_(bag_path),
      gnss_topic_(gnss_topic),
      start_time_(start_time),
      end_time_(end_time) {}

void BagGnssSource::start() {
  auto reader = open_reader(bag_path_, {gnss_topic_});
  double bag_start = get_bag_start_time_sec(*reader);
  auto time_range = compute_bag_time_range(bag_start, start_time_, end_time_);

  if (time_range.target_start_sec > 0.0) {
    double seek_target = std::max(time_range.bag_start_sec, time_range.target_start_sec - 5.0);
    try {
      reader->seek(static_cast<int64_t>(seek_target * 1e9));
    } catch (...) {}
  }

  std::vector<sensor_msgs::msg::NavSatFix> raw_fixes;

  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    auto msg = deserialize_bag_message<sensor_msgs::msg::NavSatFix>(bag_msg);
    double stamp = static_cast<double>(msg.header.stamp.sec) +
                   static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    if (time_range.bag_start_sec <= 0.0) {
      time_range = compute_bag_time_range(stamp, start_time_, end_time_);
    }

    if (!time_range.is_in_range(stamp, 5.0)) {
      if (time_range.is_past_end(stamp - 5.0)) {
        break;
      }
      continue;
    }

    if (msg.status.status >= 0) {
      raw_fixes.push_back(msg);
    }
  }

  if (raw_fixes.empty()) {
    return;
  }

  auto transformer = gnss::build_utm_transformer(raw_fixes[0].latitude, raw_fixes[0].longitude);

  for (const auto& msg : raw_fixes) {
    double stamp = static_cast<double>(msg.header.stamp.sec) +
                   static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    auto [x, y] = gnss::transform_latlon(transformer, msg.latitude, msg.longitude);

    Eigen::Matrix2d cov_2x2 = Eigen::Matrix2d::Zero();
    if (msg.position_covariance_type != 0) {
      cov_2x2(0, 0) = msg.position_covariance[0];
      cov_2x2(0, 1) = msg.position_covariance[1];
      cov_2x2(1, 0) = msg.position_covariance[3];
      cov_2x2(1, 1) = msg.position_covariance[4];
    }

    gnss_list_.push_back(core::GnssData{
        stamp,
        x,
        y,
        cov_2x2,
        static_cast<int>(msg.status.status),
        msg.latitude,
        msg.longitude,
    });
  }

  std::sort(gnss_list_.begin(), gnss_list_.end(), [](const core::GnssData& a, const core::GnssData& b) {
    return a.timestamp < b.timestamp;
  });
  timestamps_.clear();
  timestamps_.reserve(gnss_list_.size());
  for (const auto& g : gnss_list_) {
    timestamps_.push_back(g.timestamp);
  }
}

void BagGnssSource::stop() {}

std::optional<core::GnssData> BagGnssSource::get_gnss_at(double timestamp) {
  if (gnss_list_.empty()) {
    return std::nullopt;
  }
  return interpolate_gnss(gnss_list_, timestamps_, timestamp, 5.0);
}

std::vector<core::GnssData> BagGnssSource::get_all_gnss() {
  return gnss_list_;
}

// -----------------------------------------------------------------------------
// BagNavPVTSource
// -----------------------------------------------------------------------------
BagNavPVTSource::BagNavPVTSource(
    const std::string& bag_path,
    const std::string& navpvt_topic,
    double hacc_scale,
    double start_time,
    double end_time)
    : bag_path_(bag_path),
      navpvt_topic_(navpvt_topic),
      hacc_scale_(hacc_scale),
      start_time_(start_time),
      end_time_(end_time) {}

void BagNavPVTSource::start() {
  auto reader = open_reader(bag_path_, {navpvt_topic_});
  double bag_start = get_bag_start_time_sec(*reader);
  auto time_range = compute_bag_time_range(bag_start, start_time_, end_time_);

  if (time_range.target_start_sec > 0.0) {
    double seek_target = std::max(time_range.bag_start_sec, time_range.target_start_sec - 5.0);
    try {
      reader->seek(static_cast<int64_t>(seek_target * 1e9));
    } catch (...) {}
  }

  std::vector<std::pair<int64_t, ublox_msgs::msg::NavPVT>> raw_msgs;

  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    auto msg = deserialize_bag_message<ublox_msgs::msg::NavPVT>(bag_msg);
    double stamp = static_cast<double>(bag_msg->time_stamp) * 1e-9;

    if (time_range.bag_start_sec <= 0.0) {
      time_range = compute_bag_time_range(stamp, start_time_, end_time_);
    }

    if (!time_range.is_in_range(stamp, 5.0)) {
      if (time_range.is_past_end(stamp - 5.0)) {
        break;
      }
      continue;
    }

    if (!(msg.flags & 0x01) || msg.fix_type < 2) {
      continue;
    }
    raw_msgs.emplace_back(bag_msg->time_stamp, msg);
  }

  if (raw_msgs.empty()) {
    return;
  }

  double first_lat = static_cast<double>(raw_msgs[0].second.lat) * 1e-7;
  double first_lon = static_cast<double>(raw_msgs[0].second.lon) * 1e-7;
  auto transformer = gnss::build_utm_transformer(first_lat, first_lon);

  for (const auto& [t, msg] : raw_msgs) {
    double stamp = static_cast<double>(t) * 1e-9;
    double lat = static_cast<double>(msg.lat) * 1e-7;
    double lon = static_cast<double>(msg.lon) * 1e-7;
    auto [x, y] = gnss::transform_latlon(transformer, lat, lon);

    Eigen::Matrix2d cov_2x2 = Eigen::Matrix2d::Zero();
    if (msg.h_acc > 0) {
      double pos_std = (static_cast<double>(msg.h_acc) / 1000.0) * hacc_scale_;
      double pos_var = pos_std * pos_std;
      cov_2x2(0, 0) = pos_var;
      cov_2x2(1, 1) = pos_var;
    }

    int carr_soln = (msg.flags >> 6) & 0x03;

    gnss_list_.push_back(core::GnssData{
        stamp,
        x,
        y,
        cov_2x2,
        carr_soln,
        lat,
        lon,
    });
  }

  std::sort(gnss_list_.begin(), gnss_list_.end(), [](const core::GnssData& a, const core::GnssData& b) {
    return a.timestamp < b.timestamp;
  });
  timestamps_.clear();
  timestamps_.reserve(gnss_list_.size());
  for (const auto& g : gnss_list_) {
    timestamps_.push_back(g.timestamp);
  }
}

void BagNavPVTSource::stop() {}

std::optional<core::GnssData> BagNavPVTSource::get_gnss_at(double timestamp) {
  if (gnss_list_.empty()) {
    return std::nullopt;
  }
  return interpolate_gnss(gnss_list_, timestamps_, timestamp, 5.0);
}

std::vector<core::GnssData> BagNavPVTSource::get_all_gnss() {
  return gnss_list_;
}

}  // namespace input
}  // namespace slam_gnss_2d
