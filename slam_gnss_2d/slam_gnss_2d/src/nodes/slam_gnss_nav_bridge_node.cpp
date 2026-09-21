#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include "slam_gnss_2d/gnss/utm.hpp"

namespace slam_gnss_2d {

geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
  double qx = std::sin(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) -
              std::cos(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
  double qy = std::cos(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2) +
              std::sin(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2);
  double qz = std::cos(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2) -
              std::sin(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2);
  double qw = std::cos(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2) +
              std::sin(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2);
  geometry_msgs::msg::Quaternion q;
  q.x = qx; q.y = qy; q.z = qz; q.w = qw;
  return q;
}

class SlamGnssNavBridgeNode : public rclcpp::Node {
 public:
  SlamGnssNavBridgeNode() : rclcpp::Node("slam_gnss_nav_bridge_node") {
    declare_parameter("gnss_transform_file", "");
    declare_parameter("gnss_input", "navpvt");
    declare_parameter("gnss_topic", "/navpvt");
    declare_parameter("map_frame_id", "map");
    declare_parameter("gps_frame_id", "gps_link");
    declare_parameter("heading_source", "computed");
    declare_parameter("heading_min_distance", 0.6);
    declare_parameter("heading_smoothing_alpha", 0.6);
    declare_parameter("min_publish_distance", 1.0);
    declare_parameter("max_covariance_threshold", 49.0);

    transform_file_ = get_parameter("gnss_transform_file").as_string();
    gnss_input_ = get_parameter("gnss_input").as_string();
    gnss_topic_ = get_parameter("gnss_topic").as_string();
    map_frame_id_ = get_parameter("map_frame_id").as_string();
    gps_frame_id_ = get_parameter("gps_frame_id").as_string();
    heading_source_ = get_parameter("heading_source").as_string();
    heading_min_dist_ = get_parameter("heading_min_distance").as_double();
    heading_alpha_ = get_parameter("heading_smoothing_alpha").as_double();
    min_pub_dist_ = get_parameter("min_publish_distance").as_double();
    max_cov_thresh_ = get_parameter("max_covariance_threshold").as_double();

    if (!load_transform()) {
      RCLCPP_ERROR(get_logger(), "Failed to load GNSS transform. Node will not publish.");
      return;
    }

    transformer_ = gnss::UtmTransformer{utm_zone_, utm_hemisphere_ == "north"};

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/gps", 10);

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();
    anchor_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/slam_gnss_2d/anchor", map_qos);
    publish_anchor();

    if (gnss_input_ == "navpvt") {
      navpvt_sub_ = create_subscription<ublox_msgs::msg::NavPVT>(
          gnss_topic_, 10, [this](const ublox_msgs::msg::NavPVT::SharedPtr msg) {
            this->on_navpvt(msg);
          });
    } else {
      navsatfix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          gnss_topic_, 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            this->on_navsatfix(msg);
          });
    }

    RCLCPP_INFO(get_logger(), "SlamGnssNavBridgeNode initialized. Input: %s (%s)",
                gnss_input_.c_str(), gnss_topic_.c_str());
  }

 private:
  std::string transform_file_;
  std::string gnss_input_;
  std::string gnss_topic_;
  std::string map_frame_id_;
  std::string gps_frame_id_;
  std::string heading_source_;
  double heading_min_dist_;
  double heading_alpha_;
  double min_pub_dist_;
  double max_cov_thresh_;

  double anchor_lat_{0.0};
  double anchor_lon_{0.0};
  double anchor_easting_{0.0};
  double anchor_northing_{0.0};
  int utm_zone_{0};
  std::string utm_hemisphere_;
  double rotation_rad_{0.0};
  gnss::UtmTransformer transformer_;

  std::optional<double> last_map_x_;
  std::optional<double> last_map_y_;
  std::optional<double> last_heading_;
  std::optional<double> last_pub_x_;
  std::optional<double> last_pub_y_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr anchor_pub_;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_sub_;

  bool load_transform() {
    if (transform_file_.empty()) {
      return false;
    }
    try {
      YAML::Node data = YAML::LoadFile(transform_file_);
      anchor_lat_ = data["anchor"]["latitude"].as<double>();
      anchor_lon_ = data["anchor"]["longitude"].as<double>();
      anchor_easting_ = data["anchor_utm"]["easting"].as<double>();
      anchor_northing_ = data["anchor_utm"]["northing"].as<double>();
      utm_zone_ = data["anchor_utm"]["zone"].as<int>();
      utm_hemisphere_ = data["anchor_utm"]["hemisphere"].as<std::string>();
      double file_rot = 0.0;
      if (data["rotation_rad"]) {
        file_rot = data["rotation_rad"].as<double>();
      }
      // SLAMマップはすでにUTM座標系にアライメントされて生成されているため、
      // ナビゲーション時の座標変換における追加の回転は不要（0.0）とする
      rotation_rad_ = 0.0;
      RCLCPP_INFO(
          get_logger(),
          "Loaded transform: anchor=(%.7f, %.7f), file_rot=%.3frad, applied_rot=%.3frad",
          anchor_lat_, anchor_lon_, file_rot, rotation_rad_);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to read transform file: %s", e.what());
      return false;
    }
  }

  void publish_anchor() {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_id_;
    msg.latitude = anchor_lat_;
    msg.longitude = anchor_lon_;
    anchor_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published anchor: lat=%.7f, lon=%.7f", anchor_lat_, anchor_lon_);
  }

  std::pair<double, double> convert_to_map(double lat, double lon) {
    auto [utm_e, utm_n] = gnss::transform_latlon(transformer_, lat, lon);
    double local_x = utm_e - anchor_easting_;
    double local_y = utm_n - anchor_northing_;
    double cos_r = std::cos(rotation_rad_);
    double sin_r = std::sin(rotation_rad_);
    double map_x = cos_r * local_x - sin_r * local_y;
    double map_y = sin_r * local_x + cos_r * local_y;
    return {map_x, map_y};
  }

  std::optional<double> compute_heading(double map_x, double map_y) {
    if (!last_map_x_.has_value()) {
      last_map_x_ = map_x;
      last_map_y_ = map_y;
      return std::nullopt;
    }
    double dx = map_x - *last_map_x_;
    double dy = map_y - *last_map_y_;
    double dist = std::hypot(dx, dy);

    if (dist >= heading_min_dist_) {
      double new_heading = std::atan2(dy, dx);
      if (!last_heading_.has_value()) {
        last_heading_ = new_heading;
      } else {
        double diff = std::remainder(new_heading - *last_heading_, 2.0 * M_PI);
        *last_heading_ = std::remainder(*last_heading_ + heading_alpha_ * diff, 2.0 * M_PI);
      }
      last_map_x_ = map_x;
      last_map_y_ = map_y;
    }
    return last_heading_;
  }

  void on_navpvt(const ublox_msgs::msg::NavPVT::SharedPtr msg) {
    if (!(msg->flags & 0x01) || msg->fix_type < 2) {
      return;
    }

    double hacc = static_cast<double>(msg->h_acc) / 1000.0;
    double cov_xx = hacc * hacc;
    if (cov_xx * 2.0 > max_cov_thresh_) {
      return;
    }

    double lat = static_cast<double>(msg->lat) * 1e-7;
    double lon = static_cast<double>(msg->lon) * 1e-7;
    auto [map_x, map_y] = convert_to_map(lat, lon);

    if (last_pub_x_.has_value()) {
      double pub_dist = std::hypot(map_x - *last_pub_x_, map_y - *last_pub_y_);
      if (pub_dist < min_pub_dist_) {
        return;
      }
    }

    std::optional<double> heading;
    if (heading_source_ == "navpvt") {
      double heading_deg = static_cast<double>(msg->heading) * 1e-5;
      double h = heading_deg * M_PI / 180.0 + rotation_rad_;
      h = -h + (M_PI / 2.0);
      heading = std::remainder(h, 2.0 * M_PI);
    } else {
      heading = compute_heading(map_x, map_y);
    }

    if (!heading.has_value()) {
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = gps_frame_id_;

    publish_odom(header, map_x, map_y, *heading, cov_xx);
    last_pub_x_ = map_x;
    last_pub_y_ = map_y;
  }

  void on_navsatfix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < 0) {
      return;
    }

    double cov_xx = msg->position_covariance[0];
    if (cov_xx * 2.0 > max_cov_thresh_) {
      return;
    }

    auto [map_x, map_y] = convert_to_map(msg->latitude, msg->longitude);

    if (last_pub_x_.has_value()) {
      double pub_dist = std::hypot(map_x - *last_pub_x_, map_y - *last_pub_y_);
      if (pub_dist < min_pub_dist_) {
        return;
      }
    }

    auto heading = compute_heading(map_x, map_y);
    if (!heading.has_value()) {
      return;
    }

    publish_odom(msg->header, map_x, map_y, *heading, cov_xx);
    last_pub_x_ = map_x;
    last_pub_y_ = map_y;
  }

  void publish_odom(
      const std_msgs::msg::Header& header,
      double x, double y, double yaw, double pos_cov) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = map_frame_id_;
    odom.child_frame_id = gps_frame_id_;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = euler_to_quaternion(0, 0, yaw);

    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0] = pos_cov;
    odom.pose.covariance[7] = pos_cov;
    odom.pose.covariance[14] = 99999.0;
    odom.pose.covariance[21] = 99999.0;
    odom.pose.covariance[28] = 99999.0;
    odom.pose.covariance[35] = 0.1;

    odom_pub_->publish(odom);
  }
};

}  // namespace slam_gnss_2d

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam_gnss_2d::SlamGnssNavBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
