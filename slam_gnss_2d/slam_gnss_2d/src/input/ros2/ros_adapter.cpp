#include "slam_gnss_2d/input/ros2/ros_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/exceptions.h>

#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/input/time_series.hpp"

namespace slam_gnss_2d {
namespace input {

constexpr size_t kOdomBufferSize = 200;
constexpr size_t kGnssBufferSize = 1000;

// -----------------------------------------------------------------------------
// ROS2ScanSource
// -----------------------------------------------------------------------------
ROS2ScanSource::ROS2ScanSource(rclcpp::Node::SharedPtr node, const std::string& topic)
    : node_(node), topic_(topic) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ROS2ScanSource::set_scan_callback(
    std::function<void(const core::ScanDataPtr&)> callback) {
  callback_ = callback;
}

void ROS2ScanSource::start() {
  sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      topic_, 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->on_msg(msg);
      });
}

void ROS2ScanSource::stop() {
  sub_.reset();
}

void ROS2ScanSource::on_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!callback_) {
    return;
  }

  if (!lidar_tf_ready_) {
    try {
      auto tf = tf_buffer_->lookupTransform(
          "base_link", msg->header.frame_id, tf2::TimePointZero);
      lidar_yaw_ = core::quaternion_to_yaw(
          tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z, tf.transform.rotation.w);
      lidar_x_ = tf.transform.translation.x;
      lidar_y_ = tf.transform.translation.y;
      lidar_tf_ready_ = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "ScanSource: TF resolved [%s -> base_link]: x=%.3fm, y=%.3fm, yaw=%.1fdeg",
          msg->header.frame_id.c_str(), lidar_x_, lidar_y_, lidar_yaw_ * 180.0 / M_PI);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(
          node_->get_logger(),
          "ScanSource: TF not yet available, skipping scan (%s)", e.what());
      return;
    }
  }

  recv_count_++;
  if (recv_count_ == 1 || recv_count_ % 100 == 0) {
    RCLCPP_INFO(
        node_->get_logger(),
        "ScanSource [%s]: #%d, %zu ranges, range=[%.2f, %.2f]m",
        topic_.c_str(), recv_count_, msg->ranges.size(), msg->range_min, msg->range_max);
  }

  double stamp = static_cast<double>(msg->header.stamp.sec) +
                 static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

  auto scan = std::make_shared<core::ScanData>();
  scan->timestamp = stamp;
  scan->ranges = msg->ranges;
  scan->angle_min = msg->angle_min + lidar_yaw_;
  scan->angle_increment = msg->angle_increment;
  scan->range_min = msg->range_min;
  scan->range_max = msg->range_max;
  scan->lidar_x = lidar_x_;
  scan->lidar_y = lidar_y_;

  callback_(scan);
}

// -----------------------------------------------------------------------------
// ROS2OdomSource
// -----------------------------------------------------------------------------
ROS2OdomSource::ROS2OdomSource(rclcpp::Node::SharedPtr node, const std::string& topic)
    : node_(node), topic_(topic) {}

void ROS2OdomSource::start() {
  sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->on_msg(msg);
      });
}

void ROS2OdomSource::stop() {
  sub_.reset();
}

std::optional<core::OdomData> ROS2OdomSource::get_odom_at(double timestamp) {
  if (buffer_.empty()) {
    if (!empty_warned_) {
      RCLCPP_WARN(node_->get_logger(), "OdomSource [%s]: buffer is empty", topic_.c_str());
      empty_warned_ = true;
    }
    return std::nullopt;
  }

  std::vector<core::OdomData> buf(buffer_.begin(), buffer_.end());
  std::sort(buf.begin(), buf.end(), [](const core::OdomData& a, const core::OdomData& b) {
    return a.timestamp < b.timestamp;
  });

  std::vector<double> timestamps;
  timestamps.reserve(buf.size());
  for (const auto& o : buf) {
    timestamps.push_back(o.timestamp);
  }

  if (timestamp <= timestamps.front() || timestamp >= timestamps.back()) {
    const auto& best = (timestamp <= timestamps.front()) ? buf.front() : buf.back();
    double dt = std::abs(best.timestamp - timestamp);
    if (dt > 0.5) {
      RCLCPP_WARN(
          node_->get_logger(),
          "OdomSource: large time delta %.3fs (scan=%.3f, odom=%.3f)",
          dt, timestamp, best.timestamp);
    }
    return best;
  }

  return interpolate_odom(buf, timestamps, timestamp);
}

void ROS2OdomSource::on_msg(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double stamp = static_cast<double>(msg->header.stamp.sec) +
                 static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  double yaw = core::quaternion_to_yaw(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  recv_count_++;
  if (recv_count_ == 1 || recv_count_ % 100 == 0) {
    RCLCPP_INFO(
        node_->get_logger(),
        "OdomSource [%s]: #%d, x=%.2f y=%.2f yaw=%.1fdeg",
        topic_.c_str(), recv_count_,
        msg->pose.pose.position.x, msg->pose.pose.position.y, yaw * 180.0 / M_PI);
  }

  if (buffer_.size() >= kOdomBufferSize) {
    buffer_.pop_front();
  }
  buffer_.push_back(core::OdomData{
      stamp,
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      yaw,
  });
}

// -----------------------------------------------------------------------------
// ROS2GnssUtmSource
// -----------------------------------------------------------------------------
ROS2GnssUtmSource::ROS2GnssUtmSource(rclcpp::Node::SharedPtr node, const std::string& topic)
    : node_(node), topic_(topic) {}

void ROS2GnssUtmSource::start() {
  sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      topic_, 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        this->on_msg(msg);
      });
}

void ROS2GnssUtmSource::stop() {
  sub_.reset();
}

std::optional<core::GnssData> ROS2GnssUtmSource::get_gnss_at(double timestamp) {
  if (buffer_.empty()) {
    return std::nullopt;
  }
  std::vector<core::GnssData> buf(buffer_.begin(), buffer_.end());
  std::sort(buf.begin(), buf.end(), [](const core::GnssData& a, const core::GnssData& b) {
    return a.timestamp < b.timestamp;
  });
  std::vector<double> timestamps;
  timestamps.reserve(buf.size());
  for (const auto& g : buf) {
    timestamps.push_back(g.timestamp);
  }
  return nearest_by_timestamp(buf, timestamps, timestamp, 5.0);
}

std::vector<core::GnssData> ROS2GnssUtmSource::get_all_gnss() {
  return std::vector<core::GnssData>(buffer_.begin(), buffer_.end());
}

void ROS2GnssUtmSource::on_msg(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  if (msg->status.status < 0) {
    return;
  }

  double stamp = static_cast<double>(msg->header.stamp.sec) +
                 static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

  if (!transformer_.has_value()) {
    transformer_ = gnss::build_utm_transformer(msg->latitude, msg->longitude);
    RCLCPP_INFO(
        node_->get_logger(),
        "GNSS UTM transformer initialized: zone=%d north=%d",
        transformer_->zone, transformer_->northp);
  }

  auto [x, y] = gnss::transform_latlon(*transformer_, msg->latitude, msg->longitude);

  Eigen::Matrix2d cov_2x2 = Eigen::Matrix2d::Zero();
  if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    cov_2x2(0, 0) = msg->position_covariance[0];
    cov_2x2(0, 1) = msg->position_covariance[1];
    cov_2x2(1, 0) = msg->position_covariance[3];
    cov_2x2(1, 1) = msg->position_covariance[4];
  }

  recv_count_++;
  if (recv_count_ == 1 || recv_count_ % 100 == 0) {
    RCLCPP_INFO(
        node_->get_logger(),
        "GnssUtmSource [%s]: #%d, x=%.2f y=%.2f status=%d",
        topic_.c_str(), recv_count_, x, y, static_cast<int>(msg->status.status));
  }

  if (buffer_.size() >= kGnssBufferSize) {
    buffer_.pop_front();
  }
  buffer_.push_back(core::GnssData{
      stamp,
      x,
      y,
      cov_2x2,
      static_cast<int>(msg->status.status),
      msg->latitude,
      msg->longitude,
  });
}

// -----------------------------------------------------------------------------
// ROS2NavpvtSource
// -----------------------------------------------------------------------------
ROS2NavpvtSource::ROS2NavpvtSource(
    rclcpp::Node::SharedPtr node, const std::string& topic, double hacc_scale)
    : node_(node), topic_(topic), hacc_scale_(hacc_scale) {}

void ROS2NavpvtSource::start() {
  sub_ = node_->create_subscription<ublox_msgs::msg::NavPVT>(
      topic_, 10, [this](const ublox_msgs::msg::NavPVT::SharedPtr msg) {
        this->on_msg(msg);
      });
}

void ROS2NavpvtSource::stop() {
  sub_.reset();
}

std::optional<core::GnssData> ROS2NavpvtSource::get_gnss_at(double timestamp) {
  if (buffer_.empty()) {
    return std::nullopt;
  }
  std::vector<core::GnssData> buf(buffer_.begin(), buffer_.end());
  std::sort(buf.begin(), buf.end(), [](const core::GnssData& a, const core::GnssData& b) {
    return a.timestamp < b.timestamp;
  });
  std::vector<double> timestamps;
  timestamps.reserve(buf.size());
  for (const auto& g : buf) {
    timestamps.push_back(g.timestamp);
  }
  return nearest_by_timestamp(buf, timestamps, timestamp, 5.0);
}

std::vector<core::GnssData> ROS2NavpvtSource::get_all_gnss() {
  return std::vector<core::GnssData>(buffer_.begin(), buffer_.end());
}

void ROS2NavpvtSource::on_msg(const ublox_msgs::msg::NavPVT::SharedPtr msg) {
  constexpr uint8_t kFlagsGnssFixOk = 0x01;
  constexpr uint8_t kFixType2D = 2;

  if (!(msg->flags & kFlagsGnssFixOk)) {
    return;
  }
  if (msg->fix_type < kFixType2D) {
    return;
  }

  double stamp = 0.0;
  // ublox_msgs/NavPVT にはROS2 headerがない場合がある
  stamp = node_->get_clock()->now().seconds();

  double lon = static_cast<double>(msg->lon) * 1e-7;
  double lat = static_cast<double>(msg->lat) * 1e-7;

  if (!transformer_.has_value()) {
    transformer_ = gnss::build_utm_transformer(lat, lon);
    RCLCPP_INFO(
        node_->get_logger(),
        "GNSS UTM transformer initialized: zone=%d north=%d",
        transformer_->zone, transformer_->northp);
  }

  auto [x, y] = gnss::transform_latlon(*transformer_, lat, lon);

  Eigen::Matrix2d cov_2x2 = Eigen::Matrix2d::Zero();
  if (msg->h_acc > 0) {
    double pos_std = (static_cast<double>(msg->h_acc) / 1000.0) * hacc_scale_;
    double pos_var = pos_std * pos_std;
    cov_2x2(0, 0) = pos_var;
    cov_2x2(1, 1) = pos_var;
  }

  int carr_soln = (msg->flags >> 6) & 0x03;

  recv_count_++;
  if (recv_count_ == 1 || recv_count_ % 100 == 0) {
    RCLCPP_INFO(
        node_->get_logger(),
        "ROS2NavpvtSource [%s]: #%d, x=%.2f y=%.2f status=%d",
        topic_.c_str(), recv_count_, x, y, carr_soln);
  }

  if (buffer_.size() >= kGnssBufferSize) {
    buffer_.pop_front();
  }
  buffer_.push_back(core::GnssData{
      stamp,
      x,
      y,
      cov_2x2,
      carr_soln,
      lat,
      lon,
  });
}

}  // namespace input
}  // namespace slam_gnss_2d
