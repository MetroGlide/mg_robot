#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/core/slam_node_base.hpp"
#include "slam_gnss_2d/input/ros2/ros_adapter.hpp"

namespace slam_gnss_2d {

class SlamGnss2DNode : public core::SlamNodeBase {
 public:
  SlamGnss2DNode() : core::SlamNodeBase("slam_gnss_2d_node") {}

 protected:
  std::pair<std::shared_ptr<input::ScanSourceBase>, std::shared_ptr<input::OdomSourceBase>>
  setup_io(const core::SlamConfig& cfg) override {
    auto node_shared = shared_from_this();
    auto scan = std::make_shared<input::ROS2ScanSource>(node_shared, cfg.topics.scan);
    auto odom = std::make_shared<input::ROS2OdomSource>(node_shared, cfg.topics.odom);
    return {scan, odom};
  }

  std::shared_ptr<input::GnssSourceBase>
  setup_gnss_source(const core::SlamConfig& cfg) override {
    auto node_shared = shared_from_this();
    if (cfg.gnss.source == "navpvt") {
      return std::make_shared<input::ROS2NavpvtSource>(
          node_shared, cfg.gnss.topics.navpvt, cfg.gnss.navpvt_hacc_scale);
    } else {
      return std::make_shared<input::ROS2GnssUtmSource>(
          node_shared, cfg.gnss.topics.fix);
    }
  }
};

}  // namespace slam_gnss_2d

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam_gnss_2d::SlamGnss2DNode>();
  node->init();

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_INFO(node->get_logger(), "Interrupted: %s", e.what());
  }

  node->finalize();
  rclcpp::shutdown();
  return 0;
}
