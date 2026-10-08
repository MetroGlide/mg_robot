#include "mg_navigation/bt_plugins/is_path_clear_condition.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
// これより古いコストマップでは判定しない [s]
constexpr double kCostmapTimeout = 2.0;
// コストマップや経路が変わらなくても、ロボットの姿勢の変化を反映するために判定し直す間隔 [s]
constexpr double kReevaluatePeriod = 0.1;
// パラメータを取得できないときに、取得をやり直す間隔 [s]
constexpr double kRequestRetryPeriod = 1.0;

const char * const kControllerParamNames[] = {
  "use_velocity_scaled_lookahead_dist",
  "lookahead_dist",
  "min_lookahead_dist",
  "max_lookahead_dist",
  "use_rotate_to_heading",
  "rotate_to_heading_min_angle",
  "max_robot_pose_search_dist",
  "use_collision_detection",
};

const char * const kCostmapParamNames[] = {
  "footprint",
  "footprint_padding",
  "robot_radius",
  "track_unknown_space",
  "robot_base_frame",
};

}  // namespace

IsPathClearCondition::IsPathClearCondition(
  const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  std::string controller_node;
  std::string costmap_node;
  std::string costmap_topic;
  getInput("controller_node", controller_node);
  getInput("controller_id", controller_id_);
  getInput("costmap_node", costmap_node);
  getInput("costmap_topic", costmap_topic);
  getInput("margin", margin_);
  getInput("clear_duration", clear_duration_);
  getInput("transform_tolerance", transform_tolerance_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsPathClearCondition::costmapCallback, this, std::placeholders::_1),
    sub_option);

  controller_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, controller_node, rmw_qos_profile_parameters, callback_group_);
  costmap_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, costmap_node, rmw_qos_profile_parameters, callback_group_);
}

void IsPathClearCondition::costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  costmap_msg_ = msg;
  ++costmap_seq_;
  costmap_received_ = node_->now().seconds();
}

BT::NodeStatus IsPathClearCondition::tick()
{
  callback_group_executor_.spin_some();

  const double now = node_->now().seconds();
  if (!last_tick_ || now - *last_tick_ > kEpisodeGap) {
    startEpisode(now);
  }
  last_tick_ = now;

  pollParameters();
  if (!controller_params_ || !footprint_) {
    return BT::NodeStatus::FAILURE;
  }
  if (!*use_collision_detection_) {
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
  last_eval_.reset();
  last_eval_path_ = nav_msgs::msg::Path();
  reported_clear_ = false;
  reported_blocked_ = false;
  reported_first_check_ = false;
  // 設定値の変更は、次のリカバリーから反映する
  parameters_requested_ = false;
  requestParameters();
}

void IsPathClearCondition::requestParameters()
{
  if (parameters_requested_ || controller_params_future_ || costmap_params_future_) {
    return;
  }
  const double now = node_->now().seconds();
  if (last_request_attempt_ && now - *last_request_attempt_ < kRequestRetryPeriod) {
    return;
  }
  last_request_attempt_ = now;

  if (!controller_params_client_->service_is_ready() ||
    !costmap_params_client_->service_is_ready())
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000,
      "IsPathClear: parameter services of the controller or the local costmap are not ready");
    return;
  }

  std::vector<std::string> controller_names;
  for (const auto * name : kControllerParamNames) {
    controller_names.push_back(controller_id_ + "." + name);
  }
  controller_params_future_ = controller_params_client_->get_parameters(controller_names);
  costmap_params_future_ = costmap_params_client_->get_parameters(
    std::vector<std::string>(std::begin(kCostmapParamNames), std::end(kCostmapParamNames)));
  parameters_requested_ = true;
}

void IsPathClearCondition::pollParameters()
{
  if (!parameters_requested_) {
    requestParameters();
  }
  const auto ready = [](const auto & future) {
      return future && future->wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    };
  if (ready(controller_params_future_)) {
    if (!applyControllerParameters(controller_params_future_->get())) {
      RCLCPP_ERROR(
        node_->get_logger(), "IsPathClear: failed to read RPP parameters of %s",
        controller_id_.c_str());
    }
    controller_params_future_.reset();
  }
  if (ready(costmap_params_future_)) {
    if (!applyCostmapParameters(costmap_params_future_->get())) {
      RCLCPP_ERROR(node_->get_logger(), "IsPathClear: failed to read local costmap parameters");
    }
    costmap_params_future_.reset();
  }
}

bool IsPathClearCondition::applyControllerParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  if (params.size() != std::size(kControllerParamNames)) {
    return false;
  }
  for (const auto & p : params) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return false;
    }
  }
  PathClearanceParams result;
  result.use_velocity_scaled_lookahead_dist = params[0].as_bool();
  result.lookahead_dist = params[1].as_double();
  result.min_lookahead_dist = params[2].as_double();
  result.max_lookahead_dist = params[3].as_double();
  result.use_rotate_to_heading = params[4].as_bool();
  result.rotate_to_heading_min_angle = params[5].as_double();
  result.max_robot_pose_search_dist = params[6].as_double();
  result.margin = margin_;
  result.track_unknown_space = track_unknown_space_.value_or(false);
  controller_params_ = result;
  use_collision_detection_ = params[7].as_bool();
  return true;
}

bool IsPathClearCondition::applyCostmapParameters(const std::vector<rclcpp::Parameter> & params)
{
  if (params.size() != std::size(kCostmapParamNames)) {
    return false;
  }
  const auto & footprint_param = params[0];
  const auto & padding_param = params[1];
  const auto & radius_param = params[2];
  const auto & unknown_param = params[3];
  const auto & base_frame_param = params[4];
  if (padding_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
    unknown_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET ||
    base_frame_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    return false;
  }

  // Costmap2DROS と同じく、footprint が "[]" なら robot_radius の円を使い、どちらも padding を足す
  std::vector<geometry_msgs::msg::Point> footprint;
  const std::string footprint_str =
    footprint_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING ?
    footprint_param.as_string() : std::string("[]");
  if (footprint_str != "" && footprint_str != "[]") {
    if (!nav2_costmap_2d::makeFootprintFromString(footprint_str, footprint)) {
      return false;
    }
  } else {
    if (radius_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      return false;
    }
    footprint = nav2_costmap_2d::makeFootprintFromRadius(radius_param.as_double());
  }
  nav2_costmap_2d::padFootprint(footprint, padding_param.as_double());

  footprint_ = footprint;
  track_unknown_space_ = unknown_param.as_bool();
  robot_base_frame_ = base_frame_param.as_string();
  if (controller_params_) {
    controller_params_->track_unknown_space = *track_unknown_space_;
  }
  return true;
}

void IsPathClearCondition::updateCostmap()
{
  if (costmap_built_seq_ == costmap_seq_) {
    return;
  }
  const auto & meta = costmap_msg_->metadata;
  if (!costmap_) {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
      meta.size_x, meta.size_y, meta.resolution, meta.origin.position.x, meta.origin.position.y);
  } else if (costmap_->getSizeInCellsX() != meta.size_x ||  // NOLINT
    costmap_->getSizeInCellsY() != meta.size_y ||
    costmap_->getResolution() != meta.resolution ||
    costmap_->getOriginX() != meta.origin.position.x ||
    costmap_->getOriginY() != meta.origin.position.y)
  {
    costmap_->resizeMap(
      meta.size_x, meta.size_y, meta.resolution, meta.origin.position.x, meta.origin.position.y);
  }
  unsigned char * data = costmap_->getCharMap();
  std::copy(costmap_msg_->data.begin(), costmap_msg_->data.end(), data);
  costmap_built_seq_ = costmap_seq_;
}

bool IsPathClearCondition::transformPath(
  const nav_msgs::msg::Path & path, const std::string & frame,
  std::vector<geometry_msgs::msg::Pose2D> & out) const
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(frame, path.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "IsPathClear: failed to transform the path: %s", e.what());
    return false;
  }
  out.clear();
  out.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    geometry_msgs::msg::PoseStamped transformed;
    tf2::doTransform(pose, transformed, transform);
    geometry_msgs::msg::Pose2D p;
    p.x = transformed.pose.position.x;
    p.y = transformed.pose.position.y;
    p.theta = tf2::getYaw(transformed.pose.orientation);
    out.push_back(p);
  }
  return true;
}

void IsPathClearCondition::evaluate(double now)
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "IsPathClear: no path to check");
    tracker_.invalidate();
    return;
  }
  if (!costmap_msg_) {
    return;
  }
  if (now - costmap_received_ > kCostmapTimeout) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "IsPathClear: local costmap is stale");
    tracker_.invalidate();
    return;
  }

  const bool changed = costmap_seq_ != last_eval_seq_ || path != last_eval_path_;
  if (!changed && last_eval_ && now - *last_eval_ < kReevaluatePeriod) {
    return;
  }
  last_eval_ = now;
  last_eval_seq_ = costmap_seq_;
  last_eval_path_ = path;

  const std::string & frame = costmap_msg_->header.frame_id;
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, frame, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "IsPathClear: failed to get the robot pose in %s", frame.c_str());
    tracker_.invalidate();
    return;
  }
  std::vector<geometry_msgs::msg::Pose2D> path_2d;
  if (!transformPath(path, frame, path_2d)) {
    tracker_.invalidate();
    return;
  }

  updateCostmap();
  geometry_msgs::msg::Pose2D robot;
  robot.x = robot_pose.pose.position.x;
  robot.y = robot_pose.pose.position.y;
  robot.theta = tf2::getYaw(robot_pose.pose.orientation);

  const PathClearanceChecker checker(costmap_.get(), *footprint_, *controller_params_);
  const bool blocked = checker.isBlocked(robot, path_2d);
  // ブロックは、リカバリーに入る直前のコストマップでも記録する (直後にコストマップをクリアしたり、
  // 障害物がすぐ去ったりしても、塞がっていたことを取りこぼさない)。
  // 空いていることは、このリカバリーが始まってから届いたコストマップでだけ記録する
  const bool was_clear_pending = tracker_.clearPending();
  if (blocked || costmap_received_ >= episode_start_) {
    tracker_.observe(blocked, costmap_seq_, now);
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
      blocked, now - costmap_received_, path.poses.size(), end.x, end.y);
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
