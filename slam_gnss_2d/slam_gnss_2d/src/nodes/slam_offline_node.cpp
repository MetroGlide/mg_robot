#include <cmath>
#include <execinfo.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "slam_gnss_2d/core/slam_node_base.hpp"
#include "slam_gnss_2d/input/ros2/bag_reader.hpp"

namespace slam_gnss_2d {

class SlamOfflineNode : public core::SlamNodeBase {
 public:
  SlamOfflineNode() : core::SlamNodeBase("slam_gnss_2d_offline_node") {
    declare_parameter("bag_path", "");
    declare_parameter("offline_step_hz", 30.0);
    declare_parameter("start_time", 0.0);
    declare_parameter("end_time", 0.0);
  }

  void init() override {
    core::SlamNodeBase::init();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    gps_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    double step_hz = get_parameter("offline_step_hz").as_double();
    double period_sec = (step_hz <= 0.0) ? 0.001 : (1.0 / step_hz);

    step_timer_ = create_wall_timer(
        std::chrono::duration<double>(period_sec),
        [this]() { this->process_step(); });
  }

 protected:
  std::pair<std::shared_ptr<input::ScanSourceBase>, std::shared_ptr<input::OdomSourceBase>>
  setup_io(const core::SlamConfig& cfg) override {
    std::string bag_path = get_parameter("bag_path").as_string();
    double start_time = get_parameter("start_time").as_double();
    double end_time = get_parameter("end_time").as_double();
    auto scan = std::make_shared<input::BagScanSource>(bag_path, cfg.topics.scan, start_time, end_time);
    auto odom = std::make_shared<input::BagOdomSource>(bag_path, cfg.topics.odom, start_time, end_time);
    bag_scan_source_ = scan;
    return {scan, odom};
  }

  std::shared_ptr<input::GnssSourceBase>
  setup_gnss_source(const core::SlamConfig& cfg) override {
    std::string bag_path = get_parameter("bag_path").as_string();
    double start_time = get_parameter("start_time").as_double();
    double end_time = get_parameter("end_time").as_double();
    if (cfg.gnss.source == "navpvt") {
      return std::make_shared<input::BagNavPVTSource>(
          bag_path, cfg.gnss.topics.navpvt, cfg.gnss.navpvt_hacc_scale, start_time, end_time);
    } else {
      return std::make_shared<input::BagGnssSource>(
          bag_path, cfg.gnss.topics.fix, start_time, end_time);
    }
  }

  void on_frame(const core::SensorFrame& frame) override {
    core::SlamNodeBase::on_frame(frame);

    const auto& odom = frame.odom;
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";
    msg.pose.pose.position.x = odom.x;
    msg.pose.pose.position.y = odom.y;
    msg.pose.pose.orientation.w = std::cos(odom.yaw / 2.0);
    msg.pose.pose.orientation.z = std::sin(odom.yaw / 2.0);
    odom_pub_->publish(msg);

    if (config_.gnss.enabled && frame.gnss.has_value()) {
      const auto& gnss = *frame.gnss;
      if (std::abs(gnss.timestamp - last_gps_ts_) > 1e-6) {
        last_gps_ts_ = gnss.timestamp;
        sensor_msgs::msg::NavSatFix fix_msg;
        fix_msg.header.stamp = now();
        fix_msg.header.frame_id = "gps";
        fix_msg.status.status = gnss.fix_status;
        fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        fix_msg.latitude = gnss.latitude;
        fix_msg.longitude = gnss.longitude;
        fix_msg.altitude = 0.0;
        fix_msg.position_covariance[0] = gnss.covariance(0, 0);
        fix_msg.position_covariance[1] = gnss.covariance(0, 1);
        fix_msg.position_covariance[3] = gnss.covariance(1, 0);
        fix_msg.position_covariance[4] = gnss.covariance(1, 1);
        fix_msg.position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gps_fix_pub_->publish(fix_msg);
      }
    }
  }

  void process_step() {
    if (!bag_scan_source_) {
      return;
    }
    double step_hz = get_parameter("offline_step_hz").as_double();
    int batch_size = (step_hz <= 0.0) ? (skip_intermediate_rendering_ ? 500 : 10) : 1;
    for (int b = 0; b < batch_size; ++b) {
      if (!rclcpp::ok() || !bag_scan_source_->step()) {
        if (step_timer_) {
          step_timer_->cancel();
        }
        RCLCPP_INFO(get_logger(), "Bag processing complete");
        finalize();
        break;
      }
    }

    if (!skip_intermediate_rendering_) {
      publish_tf_timer();
      publish_map_timer();
    }
  }

 private:
  std::shared_ptr<input::BagScanSource> bag_scan_source_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;
  double last_gps_ts_{-1.0};
};

}  // namespace slam_gnss_2d

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam_gnss_2d::SlamOfflineNode>();
  try {
    node->init();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error in slam_offline_node: %s", e.what());
    void* callstack[64];
    int frames = backtrace(callstack, 64);
    char** strs = backtrace_symbols(callstack, frames);
    for (int i = 0; i < frames; ++i) {
      RCLCPP_ERROR(node->get_logger(), "  #%d %s", i, strs[i]);
    }
    free(strs);
    rclcpp::shutdown();
    return 1;
  }

  node->finalize();
  rclcpp::shutdown();
  return 0;
}
