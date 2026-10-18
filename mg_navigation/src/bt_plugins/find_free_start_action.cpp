#include "mg_navigation/bt_plugins/find_free_start_action.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "tf2_ros/buffer.h"

namespace mg_navigation
{
namespace bt_plugins
{

namespace
{

// 設定値の応答を tick の中で待つ上限 (BT のループを長く止めない)
constexpr std::chrono::milliseconds kParameterWait{200};

}  // namespace

FindFreeStartAction::FindFreeStartAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  monitor_ = std::make_unique<PathClearanceMonitor>(
    node_, config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"),
    PathClearanceMonitor::readOptions("FindFreeStart", *this));
}

BT::NodeStatus FindFreeStartAction::tick()
{
  monitor_->spinSome();
  monitor_->pollParametersWithin(kParameterWait);
  double max_distance = 3.0;
  double margin = 0.1;
  getInput("max_distance", max_distance);
  getInput("margin", margin);

  geometry_msgs::msg::PoseStamped start;
  switch (monitor_->findFreeStart(node_->now().seconds(), max_distance, margin, start)) {
    case PathClearanceMonitor::FreeStartStatus::kFound:
      setOutput("free_start", start);
      if (!reported_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "FindFreeStart: the robot is inside an obstacle of the global costmap; "
          "planning from (%.2f, %.2f)", start.pose.position.x, start.pose.position.y);
        reported_ = true;
      }
      return BT::NodeStatus::SUCCESS;
    case PathClearanceMonitor::FreeStartStatus::kNotFound:
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "FindFreeStart: no way out of the obstacle within %.1f m", max_distance);
      return BT::NodeStatus::FAILURE;
    case PathClearanceMonitor::FreeStartStatus::kNotNeeded:
      reported_ = false;
      return BT::NodeStatus::FAILURE;
    case PathClearanceMonitor::FreeStartStatus::kNoCostmap:
    case PathClearanceMonitor::FreeStartStatus::kUnavailable:
      return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_plugins
}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::bt_plugins::FindFreeStartAction>("FindFreeStart");
}
