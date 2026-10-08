#include "mg_navigation/bt_plugins/commit_path_action.hpp"

#include <string>

#include "mg_navigation/bt_plugins/path_clearance.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

CommitPathAction::CommitPathAction(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
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

  // まだ書かれていないキーは空の経路として扱う
  nav_msgs::msg::Path candidate;
  getInput("candidate", candidate);
  nav_msgs::msg::Path current;
  getInput("path", current);

  const auto selected = selectPathForGoal(candidate, current, goal, goal_tolerance);
  if (selected) {
    setOutput("path", *selected);
    return BT::NodeStatus::SUCCESS;
  }
  if (!current.poses.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("CommitPath"),
      "CommitPath: no path reaches the current goal; dropping the path to the previous goal");
  }
  setOutput("path", nav_msgs::msg::Path());
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_plugins
}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::bt_plugins::CommitPathAction>("CommitPath");
}
