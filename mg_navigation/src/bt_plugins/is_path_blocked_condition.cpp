#include "mg_navigation/bt_plugins/is_path_blocked_condition.hpp"

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace mg_navigation
{
namespace bt_plugins
{

IsPathBlockedCondition::IsPathBlockedCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  monitor_ = std::make_unique<PathClearanceMonitor>(
    node_, config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"),
    PathClearanceMonitor::readOptions("IsPathBlocked", *this));
}

BT::NodeStatus IsPathBlockedCondition::tick()
{
  monitor_->spinSome();
  monitor_->pollParameters();
  if (!monitor_->ready() || !monitor_->useCollisionDetection()) {
    // RPP が障害物で止まらない設定では、再計画を止めると障害物へ向かって進み続ける
    report(false);
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path path;
  getInput("path", path);
  switch (monitor_->evaluate(path, node_->now().seconds())) {
    case PathClearanceMonitor::Status::kEvaluated:
      blocked_ = monitor_->lastBlocked();
      break;
    case PathClearanceMonitor::Status::kUnchanged:
      break;
    case PathClearanceMonitor::Status::kNoCostmap:
    case PathClearanceMonitor::Status::kUnavailable:
      blocked_ = false;
      break;
  }
  report(blocked_);
  return blocked_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void IsPathBlockedCondition::report(bool blocked)
{
  if (blocked == reported_blocked_) {
    return;
  }
  if (blocked) {
    RCLCPP_INFO(
      node_->get_logger(), "IsPathBlocked: path ahead is blocked, holding periodic replanning");
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "IsPathBlocked: path ahead is not blocked, resuming periodic replanning");
  }
  reported_blocked_ = blocked;
}

}  // namespace bt_plugins
}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::bt_plugins::IsPathBlockedCondition>("IsPathBlocked");
}
