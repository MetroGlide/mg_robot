#include "mg_navigation/bt_plugins/is_path_clear_condition.hpp"

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace mg_navigation
{
namespace bt_plugins
{

namespace
{

// これ以上 tick の間隔が空いたら、新しいリカバリーが始まったとみなす [s]。
// リカバリー中は BT のループ (10ms) ごとに tick される
constexpr double kEpisodeGap = 1.0;
// 空いたと判定するのに必要な、空いていたコストマップの枚数
constexpr int kMinClearMaps = 2;

}  // namespace

IsPathClearCondition::IsPathClearCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  getInput("clear_duration", clear_duration_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  const auto options = PathClearanceMonitor::readOptions("IsPathClear", *this);
  controller_id_ = options.controller_id;
  monitor_ = std::make_unique<PathClearanceMonitor>(
    node_, config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"), options);
}

BT::NodeStatus IsPathClearCondition::tick()
{
  monitor_->spinSome();

  const double now = node_->now().seconds();
  if (!last_tick_ || now - *last_tick_ > kEpisodeGap) {
    startEpisode(now);
  }
  last_tick_ = now;

  monitor_->pollParameters();
  if (!monitor_->ready()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!monitor_->useCollisionDetection()) {
    // RPP が障害物で止まらない設定では、空くのを待つ意味がない
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 30000,
      "IsPathClear: %s.use_collision_detection is false; never reports clear",
      controller_id_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  evaluate(now);

  if (tracker_.isClear(now, clear_duration_, kMinClearMaps)) {
    if (!reported_clear_) {
      RCLCPP_INFO(node_->get_logger(), "IsPathClear: path became clear, resuming");
      reported_clear_ = true;
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsPathClearCondition::startEpisode(double now)
{
  tracker_.reset();
  episode_start_ = now;
  monitor_->resetEvaluation();
  reported_clear_ = false;
  reported_blocked_ = false;
  reported_first_check_ = false;
  // 設定値の変更は、次のリカバリーから反映する
  monitor_->refreshParameters();
}

void IsPathClearCondition::evaluate(double now)
{
  nav_msgs::msg::Path path;
  getInput("path", path);
  const auto status = monitor_->evaluate(path, now);
  if (status == PathClearanceMonitor::Status::kUnavailable) {
    tracker_.invalidate();
    return;
  }
  if (status != PathClearanceMonitor::Status::kEvaluated) {
    return;
  }

  const bool blocked = monitor_->lastBlocked();
  // ブロックは、リカバリーに入る直前のコストマップでも記録する (直後にコストマップをクリアしたり、
  // 障害物がすぐ去ったりしても、塞がっていたことを取りこぼさない)。
  // 空いていることは、このリカバリーが始まってから届いたコストマップでだけ記録する
  const bool was_clear_pending = tracker_.clearPending();
  if (blocked || monitor_->costmapReceived() >= episode_start_) {
    tracker_.observe(blocked, monitor_->costmapSeq(), now);
  }
  if (!was_clear_pending && tracker_.clearPending()) {
    RCLCPP_INFO(
      node_->get_logger(), "IsPathClear: path looks clear, confirming for %.1f s",
      clear_duration_);
  }
  if (!reported_first_check_) {
    const auto & end = path.poses.back().pose.position;
    RCLCPP_INFO(
      node_->get_logger(),
      "IsPathClear: first check of this recovery: blocked=%d (costmap age %.2f s, "
      "path %zu poses ending at (%.2f, %.2f))",
      blocked, now - monitor_->costmapReceived(), path.poses.size(), end.x, end.y);
    reported_first_check_ = true;
  }
  if (blocked && !reported_blocked_) {
    RCLCPP_INFO(node_->get_logger(), "IsPathClear: path is blocked, waiting for it to clear");
    reported_blocked_ = true;
  }
}

}  // namespace bt_plugins
}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::bt_plugins::IsPathClearCondition>("IsPathClear");
}
