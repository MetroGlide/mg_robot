#pragma once

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mg_msgs/msg/path_candidate_check.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// 計画の結果 (candidate) を、追従する経路 (path) に反映する BT ノード。
// ComputePathToPose は失敗すると出力を空にするので、candidate に書かせてこのノードで反映し、
// 計画が失敗しても直前の経路で追従 (RPP の判定) を続けられるようにする。
//
// - candidate の終点が goal から goal_tolerance 以内で、かつ遠回りが上限以内なら、
//   path を candidate にして SUCCESS。遠回りの上限は、
//   max_detour_ratio * (始点からゴールまでの直線距離) + max_detour_margin。
//   プランナが逆走や大回りの経路を引いたときに乗らずに待つための安全柵で、上限を超えた
//   candidate は計画の失敗と同じに扱う。今の path には上限を掛けない
// - そうでなく、今の path の終点が goal に届いていれば、path をそのままにして SUCCESS
// - どちらでもなければ (別のゴールへの経路しかない)、path を空にして FAILURE。
//   古いゴールへの経路を追従し続けて、そこで到着と報告するのを防ぐ
class CommitPathAction : public BT::SyncActionNode
{
public:
  CommitPathAction(const std::string & name, const BT::NodeConfiguration & conf);

  CommitPathAction() = delete;

  // 候補の判定結果 (PathCandidateCheck) を配信するトピック
  static constexpr char kCheckTopic[] = "/navigation/path_candidate_check";
  // 採用した経路を配信するトピック (transient local)。candidate を採用したときと、
  // 経路を空にしたときに配信する。planner_server の /plan は棄却した候補も含むので、
  // 表示や通過点の判定にはこちらを使う
  static constexpr char kCommittedPathTopic[] = "/navigation/committed_path";

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("candidate", "Newly planned path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current navigation goal"),
      BT::BidirectionalPort<nav_msgs::msg::Path>("path", "Path to follow"),
      BT::InputPort<double>(
        "goal_tolerance", 0.6, "Max distance [m] between the path end and the goal"),
      BT::InputPort<double>(
        "max_detour_ratio", 2.0,
        "Reject a candidate longer than ratio * straight distance + margin. 0 disables"),
      BT::InputPort<double>(
        "max_detour_margin", 10.0, "Margin [m] added to the detour limit"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<mg_msgs::msg::PathCandidateCheck>::SharedPtr check_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr committed_pub_;
};

}  // namespace bt_plugins
}  // namespace mg_navigation
