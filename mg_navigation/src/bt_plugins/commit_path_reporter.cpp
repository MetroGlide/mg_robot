#include "mg_navigation/bt_plugins/commit_path_reporter.hpp"

#include <memory>

namespace mg_navigation
{
namespace bt_plugins
{

namespace
{

// blackboard に置く共有の CommitPathReporter のキー
constexpr char kBlackboardKey[] = "commit_path_reporter";

}  // namespace

constexpr char CommitPathReporter::kCheckTopic[];
constexpr char CommitPathReporter::kCommittedPathTopic[];

CommitPathReporter::CommitPathReporter(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  check_pub_ = node_->create_publisher<mg_msgs::msg::PathCandidateCheck>(kCheckTopic, 10);
  // 後から購読を始めた表示にも、今の経路が届くようにする
  committed_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    kCommittedPathTopic, rclcpp::QoS(1).transient_local());
}

std::shared_ptr<CommitPathReporter> CommitPathReporter::shared(
  const BT::Blackboard::Ptr & blackboard, const rclcpp::Node::SharedPtr & node)
{
  std::shared_ptr<CommitPathReporter> reporter;
  if (!blackboard->get(kBlackboardKey, reporter) || !reporter) {
    reporter = std::make_shared<CommitPathReporter>(node);
    blackboard->set(kBlackboardKey, reporter);
  }
  return reporter;
}

void CommitPathReporter::reportDetour(const DetourCheck & detour)
{
  mg_msgs::msg::PathCandidateCheck check;
  check.stamp = node_->now();
  check.rejected = detour.exceeded;
  check.path_length = static_cast<float>(detour.path_length);
  check.straight_distance = static_cast<float>(detour.straight_distance);
  check.limit = static_cast<float>(detour.limit);
  check_pub_->publish(check);

  if (rejecting_ == detour.exceeded) {
    return;
  }
  rejecting_ = detour.exceeded;
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

void CommitPathReporter::publishCommittedPath(const nav_msgs::msg::Path & path)
{
  committed_pub_->publish(path);
}

}  // namespace bt_plugins
}  // namespace mg_navigation
