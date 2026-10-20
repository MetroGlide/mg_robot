#pragma once

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "localization_supervisor_msgs/msg/localization_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// 自己位置が信頼できるか (監督の状態が NORMAL か) を判定する BT 条件ノード。
// 自己位置が疑わしいときは復旧に任せるので、計画用地図の障害物からの脱出などの走行側の対処を止めるために使う。
//
// SUCCESS は、最後に受け取った状態が NORMAL で、max_age 秒以内のとき。
// 状態を受け取っていない、古い、NORMAL でない (疑い・切り離し・復旧中・復旧できない) ときは FAILURE。
// 状態は `/localization/status` (localization_supervisor_msgs/LocalizationStatus)。
class IsLocalizationHealthyCondition : public BT::ConditionNode
{
public:
  IsLocalizationHealthyCondition(
    const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsLocalizationHealthyCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/localization/status", "Status topic of the supervisor"),
      BT::InputPort<double>("max_age", 3.0, "Max age [s] of the status to trust"),
    };
  }

private:
  void callback(localization_supervisor_msgs::msg::LocalizationStatus::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<localization_supervisor_msgs::msg::LocalizationStatus>::SharedPtr sub_;
  uint8_t state_{0};
  double received_{-1.0};
  bool reported_healthy_{true};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
