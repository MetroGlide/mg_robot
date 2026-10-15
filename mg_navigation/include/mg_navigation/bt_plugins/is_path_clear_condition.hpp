#pragma once

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "mg_navigation/bt_plugins/path_clearance.hpp"
#include "mg_navigation/bt_plugins/path_clearance_monitor.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// リカバリー中に、RPP が衝突判定なしで走り出せる状態になったかを判定する BT 条件ノード。
// ReactiveFallback に置き、SUCCESS で実行中の回避行動を中断して走行に戻す。
//
// - 判定は PathClearanceMonitor (PathClearanceChecker。RPP と同等以上に厳しい判定)
// - リカバリーの間にブロックを観測し、その後に空いた状態が clear_duration 続いたときだけ
//   SUCCESS を返す。最初から空いているとき (障害物以外が原因の失敗) は FAILURE
// - tick の間隔が空いたら新しいリカバリーとみなし、状態のリセットと設定値の再取得をする
class IsPathClearCondition : public BT::ConditionNode
{
public:
  IsPathClearCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsPathClearCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = PathClearanceMonitor::commonPorts();
    ports.insert(
      BT::InputPort<double>(
        "clear_duration", 1.0, "Time [s] the path must stay clear after being blocked"));
    return ports;
  }

private:
  void startEpisode(double now);
  void evaluate(double now);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<PathClearanceMonitor> monitor_;
  std::string controller_id_;
  double clear_duration_;

  ClearanceTracker tracker_;
  std::optional<double> last_tick_;
  double episode_start_{0.0};
  bool reported_clear_{false};
  bool reported_blocked_{false};
  bool reported_first_check_{false};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
