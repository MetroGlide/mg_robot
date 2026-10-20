#include "mg_navigation/bt_plugins/is_localization_healthy_condition.hpp"

#include <functional>
#include <string>

namespace mg_navigation
{
namespace bt_plugins
{

using localization_supervisor_msgs::msg::LocalizationStatus;

IsLocalizationHealthyCondition::IsLocalizationHealthyCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string topic = "/localization/status";
  getInput("topic", topic);
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  sub_ = node_->create_subscription<LocalizationStatus>(
    topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsLocalizationHealthyCondition::callback, this, std::placeholders::_1),
    sub_option);
}

void IsLocalizationHealthyCondition::callback(LocalizationStatus::SharedPtr msg)
{
  state_ = msg->state;
  received_ = node_->now().seconds();
}

BT::NodeStatus IsLocalizationHealthyCondition::tick()
{
  callback_group_executor_.spin_some();
  double max_age = 3.0;
  getInput("max_age", max_age);
  const bool fresh = received_ >= 0.0 && node_->now().seconds() - received_ <= max_age;
  const bool healthy = fresh && state_ == LocalizationStatus::NORMAL;
  if (healthy != reported_healthy_) {
    if (healthy) {
      RCLCPP_INFO(node_->get_logger(), "IsLocalizationHealthy: localization is healthy");
    } else {
      RCLCPP_INFO(
        node_->get_logger(), "IsLocalizationHealthy: localization is not healthy (%s)",
        fresh ? "state is not NORMAL" : "no recent status");
    }
    reported_healthy_ = healthy;
  }
  return healthy ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace bt_plugins
}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::bt_plugins::IsLocalizationHealthyCondition>(
    "IsLocalizationHealthy");
}
