#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include "slam_gnss_2d/gnss/nav_bridge_core.hpp"
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

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class SlamGnssNavBridgeNode : public rclcpp::Node {
 public:
  SlamGnssNavBridgeNode() : rclcpp::Node("slam_gnss_nav_bridge_node") {
    declare_parameter("gnss_transform_file", "");
    declare_parameter("gnss_input", "navpvt");
    declare_parameter("gnss_topic", "/navpvt");
    declare_parameter("map_frame_id", "map");
    declare_parameter("base_frame_id", "base_footprint");
    declare_parameter("gps_frame_id", "gps_link");
    declare_parameter("heading_source", "computed");
    declare_parameter("heading_min_distance", 0.6);
    declare_parameter("heading_smoothing_alpha", 0.6);
    declare_parameter("min_publish_distance", 1.0);
    declare_parameter("max_covariance_threshold", 49.0);
    // アンテナ位置 (車体座標系)。URDF の base_footprint -> gps_link の x, y と合わせる
    declare_parameter("lever_arm_compensation", true);
    declare_parameter("lever_arm_x", 0.26);
    declare_parameter("lever_arm_y", -0.13);
    // 車体の向きに使う TF (map -> base) がこれより古いときは、向きが分からないものとして扱う
    declare_parameter("yaw_max_age_sec", 1.0);
    // 受信してから /odom/gps のスタンプを付けるまでの遅れの補正。スタンプは now() からこの秒数を引く
    declare_parameter("time_offset_sec", 0.0);
    // 解の種類ごとの分散の決め方 (hAcc の下限 [m] と、分散にかける係数)
    declare_parameter("fix_floor_m", 0.02);
    declare_parameter("fix_scale", 1.0);
    declare_parameter("float_floor_m", 0.02);
    declare_parameter("float_scale", 1.0);
    declare_parameter("single_floor_m", 0.02);
    declare_parameter("single_scale", 1.0);
    declare_parameter("accept_single", true);

    transform_file_ = get_parameter("gnss_transform_file").as_string();
    gnss_input_ = get_parameter("gnss_input").as_string();
    gnss_topic_ = get_parameter("gnss_topic").as_string();
    map_frame_id_ = get_parameter("map_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    gps_frame_id_ = get_parameter("gps_frame_id").as_string();
    heading_source_ = get_parameter("heading_source").as_string();
    heading_min_dist_ = get_parameter("heading_min_distance").as_double();
    heading_alpha_ = get_parameter("heading_smoothing_alpha").as_double();
    min_pub_dist_ = get_parameter("min_publish_distance").as_double();
    quality_.max_covariance_threshold = get_parameter("max_covariance_threshold").as_double();
    lever_arm_compensation_ = get_parameter("lever_arm_compensation").as_bool();
    lever_x_ = get_parameter("lever_arm_x").as_double();
    lever_y_ = get_parameter("lever_arm_y").as_double();
    yaw_max_age_sec_ = get_parameter("yaw_max_age_sec").as_double();
    time_offset_sec_ = get_parameter("time_offset_sec").as_double();
    quality_.fix_floor_m = get_parameter("fix_floor_m").as_double();
    quality_.fix_scale = get_parameter("fix_scale").as_double();
    quality_.float_floor_m = get_parameter("float_floor_m").as_double();
    quality_.float_scale = get_parameter("float_scale").as_double();
    quality_.single_floor_m = get_parameter("single_floor_m").as_double();
    quality_.single_scale = get_parameter("single_scale").as_double();
    quality_.accept_single = get_parameter("accept_single").as_bool();

    if (!load_transform()) {
      RCLCPP_ERROR(get_logger(), "Failed to load GNSS transform. Node will not publish.");
      return;
    }

    transformer_ = gnss::UtmTransformer{utm_zone_, utm_hemisphere_ == "north"};

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/gps", 10);

    // /odom/gps の配信の有効・無効 (ウェイポイントの gps_on / gps_off から使う)
    publish_service_ = create_service<std_srvs::srv::SetBool>(
        "~/change_publish_state",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          publish_enabled_ = request->data;
          response->success = true;
          response->message = publish_enabled_ ? "publishing /odom/gps" : "stopped /odom/gps";
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        });

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

    RCLCPP_INFO(get_logger(),
                "SlamGnssNavBridgeNode initialized. Input: %s (%s), lever_arm_compensation=%s "
                "(%.2f, %.2f), time_offset=%.3f s",
                gnss_input_.c_str(), gnss_topic_.c_str(),
                lever_arm_compensation_ ? "true" : "false", lever_x_, lever_y_, time_offset_sec_);
  }

 private:
  std::string transform_file_;
  std::string gnss_input_;
  std::string gnss_topic_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string gps_frame_id_;
  std::string heading_source_;
  double heading_min_dist_;
  double heading_alpha_;
  double min_pub_dist_;
  gnss::PositionQualityConfig quality_;
  bool lever_arm_compensation_;
  double lever_x_;
  double lever_y_;
  double yaw_max_age_sec_;
  double time_offset_sec_;

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

  bool publish_enabled_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr publish_service_;
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

  // EKF が出している map -> base の最新の向き。TF が無い、または古いときは std::nullopt。
  std::optional<double> lookup_base_yaw() {
    try {
      const auto transform =
          tf_buffer_->lookupTransform(map_frame_id_, base_frame_id_, tf2::TimePointZero);
      const double age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
      if (age > yaw_max_age_sec_) {
        return std::nullopt;
      }
      return quaternion_to_yaw(transform.transform.rotation);
    } catch (const tf2::TransformException&) {
      return std::nullopt;
    }
  }

  rclcpp::Time measurement_stamp() {
    return now() - rclcpp::Duration::from_seconds(time_offset_sec_);
  }

  void on_navpvt(const ublox_msgs::msg::NavPVT::SharedPtr msg) {
    if (!(msg->flags & 0x01) || msg->fix_type < 2) {
      return;
    }

    const double hacc = static_cast<double>(msg->h_acc) / 1000.0;
    const auto variance = gnss::PositionVariance(
        quality_, gnss::CarrierFromFlags(msg->flags), hacc);
    if (!variance.has_value()) {
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
    header.stamp = measurement_stamp();
    header.frame_id = gps_frame_id_;

    publish_odom(header, map_x, map_y, *heading, *variance);
    last_pub_x_ = map_x;
    last_pub_y_ = map_y;
  }

  void on_navsatfix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < 0) {
      return;
    }

    // NavSatFix には搬送波位相の解の種類がないため、RTK (GBAS_FIX) だけを Fix として扱う
    const auto carrier = msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX
                             ? gnss::CarrierSolution::kFixed
                             : gnss::CarrierSolution::kNone;
    const double h_acc = std::sqrt(msg->position_covariance[0]);
    const auto variance = gnss::PositionVariance(quality_, carrier, h_acc);
    if (!variance.has_value()) {
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

    std_msgs::msg::Header header = msg->header;
    header.stamp = rclcpp::Time(msg->header.stamp) - rclcpp::Duration::from_seconds(time_offset_sec_);
    publish_odom(header, map_x, map_y, *heading, *variance);
    last_pub_x_ = map_x;
    last_pub_y_ = map_y;
  }

  // antenna_x, antenna_y はアンテナの map 座標。アンテナ位置の補正が有効で車体の向きが分かれば、
  // 車体中心 (base_frame) の位置に直して出す。分からないときはアンテナ位置のまま、
  // レバーアームの分だけ分散を増やして出す。
  void publish_odom(
      const std_msgs::msg::Header& header,
      double antenna_x, double antenna_y, double yaw, double pos_var) {
    if (!publish_enabled_) {
      return;
    }
    gnss::Point2 position{antenna_x, antenna_y};
    std::string child_frame = gps_frame_id_;
    if (lever_arm_compensation_) {
      const auto base_yaw = lookup_base_yaw();
      if (base_yaw.has_value()) {
        position = gnss::AntennaToBase(position, *base_yaw, lever_x_, lever_y_);
        child_frame = base_frame_id_;
      } else {
        pos_var += gnss::LeverArmUncertaintyVariance(lever_x_, lever_y_);
      }
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = map_frame_id_;
    odom.child_frame_id = child_frame;

    odom.pose.pose.position.x = position.x;
    odom.pose.pose.position.y = position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = euler_to_quaternion(0, 0, yaw);

    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0] = pos_var;
    odom.pose.covariance[7] = pos_var;
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
