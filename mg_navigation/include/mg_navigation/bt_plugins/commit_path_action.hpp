#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// 計画の結果 (candidate) を、追従する経路 (path) に反映する BT ノード。
// ComputePathToPose は失敗すると出力を空にするので、candidate に書かせてこのノードで反映し、
// 計画が失敗しても直前の経路で追従 (RPP の判定) を続けられるようにする。
//
// - candidate の終点が goal から goal_tolerance 以内なら、path を candidate にして SUCCESS
// - そうでなく、今の path の終点が goal に届いていれば、path をそのままにして SUCCESS
// - どちらでもなければ (別のゴールへの経路しかない)、path を空にして FAILURE。
//   古いゴールへの経路を追従し続けて、そこで到着と報告するのを防ぐ
class CommitPathAction : public BT::SyncActionNode
{
public:
  CommitPathAction(const std::string & name, const BT::NodeConfiguration & conf);

  CommitPathAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("candidate", "Newly planned path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current navigation goal"),
      BT::BidirectionalPort<nav_msgs::msg::Path>("path", "Path to follow"),
      BT::InputPort<double>(
        "goal_tolerance", 0.6, "Max distance [m] between the path end and the goal"),
    };
  }
};

}  // namespace bt_plugins
}  // namespace mg_navigation
