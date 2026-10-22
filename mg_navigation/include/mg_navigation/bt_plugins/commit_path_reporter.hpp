#pragma once

#include <memory>

#include "behaviortree_cpp_v3/blackboard.h"
#include "mg_msgs/msg/path_candidate_check.hpp"
#include "mg_navigation/bt_plugins/path_clearance.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// CommitPath の判定と、採用した経路を外へ出す。
// BT には CommitPath が複数あるので、同じ木 (blackboard) の CommitPath で 1 つを共有する。
// transient local の配信者が CommitPath ごとにあると、後から購読した表示に、それぞれが
// 最後に配信した古い経路が届いてしまうため。
class CommitPathReporter
{
public:
  // 候補の判定結果 (PathCandidateCheck) を配信するトピック
  static constexpr char kCheckTopic[] = "/navigation/path_candidate_check";
  // 採用した経路を配信するトピック (transient local)。planner_server の /plan は棄却した候補も
  // 含むので、表示や通過点の判定にはこちらを使う
  static constexpr char kCommittedPathTopic[] = "/navigation/committed_path";

  explicit CommitPathReporter(const rclcpp::Node::SharedPtr & node);

  // blackboard の共有のものを返す。まだなければ作って置く
  static std::shared_ptr<CommitPathReporter> shared(
    const BT::Blackboard::Ptr & blackboard, const rclcpp::Node::SharedPtr & node);

  // 遠回りの判定を配信し、棄却の始まりと解消をイベントログに出す
  void reportDetour(const DetourCheck & detour);

  // 採用した経路を配信する。空の経路は「追従する経路がない」を表す
  void publishCommittedPath(const nav_msgs::msg::Path & path);

  // 直前の判定で候補を棄却したか
  bool rejecting() const {return rejecting_;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<mg_msgs::msg::PathCandidateCheck>::SharedPtr check_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr committed_pub_;
  bool rejecting_{false};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
