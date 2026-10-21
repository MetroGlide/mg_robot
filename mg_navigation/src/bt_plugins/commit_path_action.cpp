#include "mg_navigation/bt_plugins/commit_path_action.hpp"

#include <string>

#include "mg_navigation/bt_plugins/path_clearance.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

namespace
{

// 直前の判定で候補を棄却したかを置く blackboard のキー。BT には CommitPath が複数あるので、
// イベントログを状態が変わったときだけ出すために、同じ木の CommitPath で共有する
constexpr char kRejectingKey[] = "detour_rejecting";

}  // namespace

constexpr char CommitPathAction::kCheckTopic[];
constexpr char CommitPathAction::kCommittedPathTopic[];

CommitPathAction::CommitPathAction(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  check_pub_ = node_->create_publisher<mg_msgs::msg::PathCandidateCheck>(kCheckTopic, 10);
  // 後から購読を始めた表示にも、今の経路が届くようにする
  committed_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    kCommittedPathTopic, rclcpp::QoS(1).transient_local());
}

BT::NodeStatus CommitPathAction::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(rclcpp::get_logger("CommitPath"), "CommitPath: goal is not set");
    return BT::NodeStatus::FAILURE;
  }
  double goal_tolerance = 0.6;
  getInput("goal_tolerance", goal_tolerance);
  double max_detour_ratio = 2.0;
  getInput("max_detour_ratio", max_detour_ratio);
  double max_detour_margin = 10.0;
  getInput("max_detour_margin", max_detour_margin);

  // まだ書かれていないキーは空の経路として扱う
  nav_msgs::msg::Path candidate;
  getInput("candidate", candidate);
  nav_msgs::msg::Path current;
  getInput("path", current);

  // ゴールへ届く候補だけを遠回りの判定にかける。上限を超えたら、計画の失敗と同じ空の候補にする
  if (pathReachesGoal(candidate, goal, goal_tolerance)) {
    const auto detour = checkDetour(candidate, goal, max_detour_ratio, max_detour_margin);
    mg_msgs::msg::PathCandidateCheck check;
    check.stamp = node_->now();
    check.rejected = detour.exceeded;
    check.path_length = static_cast<float>(detour.path_length);
    check.straight_distance = static_cast<float>(detour.straight_distance);
    check.limit = static_cast<float>(detour.limit);
    check_pub_->publish(check);

    bool was_rejecting = false;
    config().blackboard->get(kRejectingKey, was_rejecting);
    if (was_rejecting != detour.exceeded) {
      config().blackboard->set(kRejectingKey, detour.exceeded);
      if (detour.exceeded) {
        RCLCPP_WARN(
          node_->get_logger(),
          "evt=nav.detour_rejected path_length=%.1f straight_distance=%.1f limit=%.1f",
          detour.path_length, detour.straight_distance, detour.limit);
      } else {
        RCLCPP_INFO(
          node_->get_logger(),
          "evt=nav.detour_accepted path_length=%.1f straight_distance=%.1f",
          detour.path_length, detour.straight_distance);
      }
    }
    if (detour.exceeded) {
      candidate = nav_msgs::msg::Path();
    }
  }

  // candidate を採用したときだけ配信する。今の経路を保つときは、配信済みなので送らない
  const bool adopt_candidate = pathReachesGoal(candidate, goal, goal_tolerance);
  const auto selected = selectPathForGoal(candidate, current, goal, goal_tolerance);
  if (selected) {
    setOutput("path", *selected);
    if (adopt_candidate) {
      committed_pub_->publish(*selected);
    }
    return BT::NodeStatus::SUCCESS;
  }
  if (!current.poses.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("CommitPath"),
      "CommitPath: no path reaches the current goal; dropping the path to the previous goal");
  }
  setOutput("path", nav_msgs::msg::Path());
  nav_msgs::msg::Path empty;
  empty.header.frame_id = goal.header.frame_id;
  empty.header.stamp = node_->now();
  committed_pub_->publish(empty);
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_plugins
}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::bt_plugins::CommitPathAction>("CommitPath");
}
