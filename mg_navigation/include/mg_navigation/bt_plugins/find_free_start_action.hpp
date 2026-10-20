#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mg_navigation/bt_plugins/path_clearance_monitor.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// ロボットが、グローバルコストマップ (計画用地図) の障害物にめり込んでいるとき、その領域の外の
// 最も近い位置を free_start に出す BT アクションノード。
// 計画用地図にだけある進入禁止の領域に入ると、プランナが「始点が致死」で計画できず、リカバリーをくり返す。
// ComputePathToPose の start に free_start を渡せば、領域の外から計画でき、
// 走行はローカルコストマップ (センサだけ) に任せて領域から出られる。
//
// SUCCESS は free_start を出したときだけ。次のときは FAILURE (free_start は変えない):
//   - ロボットが障害物にめり込んでいない (探す必要がない)
//   - max_distance 以内に領域から出られない
//   - 判定できない (フットプリントの未取得、コストマップが古い・未受信、TF が取れない)
// 衝突の判定は RPP と同じ (フットプリントのコストが致死以上)。設定値は持たず、
// グローバルコストマップのフットプリント・footprint_padding を取得する。
class FindFreeStartAction : public BT::SyncActionNode
{
public:
  FindFreeStartAction(const std::string & name, const BT::NodeConfiguration & conf);

  FindFreeStartAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "free_start", "Nearest pose outside the colliding region (robot's heading is kept)"),
      BT::InputPort<double>("max_distance", 3.0, "Max distance [m] to search for the way out"),
      BT::InputPort<double>(
        "margin", 0.1, "The exit must have no collision within this distance [m]"),
      BT::InputPort<std::string>("controller_node", "controller_server", "Controller server node"),
      BT::InputPort<std::string>("controller_id", "FollowPath", "RPP plugin name"),
      BT::InputPort<std::string>(
        "costmap_node", "global_costmap/global_costmap", "Global costmap node"),
      BT::InputPort<std::string>(
        "costmap_topic", "global_costmap/costmap_raw", "Global costmap topic"),
      BT::InputPort<double>("transform_tolerance", 0.2, "TF tolerance [s]"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<PathClearanceMonitor> monitor_;
  bool reported_{false};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
