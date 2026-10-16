#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "mg_navigation/bt_plugins/path_clearance_monitor.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// 経路の近く (RPP の判定範囲 + margin) が今塞がっているかを判定する BT 条件ノード。
// 塞がっていると確定したときだけ SUCCESS を返し、空いているときと判定できないとき
// (設定値の未取得、コストマップが古い、経路がない、RPP の衝突判定が無効) は FAILURE を返す。
//
// 定期的な再計画の前に Inverter で包んで置き、塞がっている間は再計画しないで
// RPP の停止 (failure_tolerance の待ち) に任せるために使う。判定は IsPathClear と同じ
// PathClearanceMonitor。設定値は最初に一度だけ取得する。
class IsPathBlockedCondition : public BT::ConditionNode
{
public:
  IsPathBlockedCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsPathBlockedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return PathClearanceMonitor::commonPorts();
  }

private:
  void report(bool blocked);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<PathClearanceMonitor> monitor_;
  bool blocked_{false};
  bool reported_blocked_{false};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
