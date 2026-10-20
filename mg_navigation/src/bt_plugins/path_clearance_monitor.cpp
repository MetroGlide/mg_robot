#include "mg_navigation/bt_plugins/path_clearance_monitor.hpp"

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

PathClearanceMonitor::PathClearanceMonitor(
  const rclcpp::Node::SharedPtr & node, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const PathClearanceMonitorOptions & options)
: node_(node), tf_(tf), options_(options)
{
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    options_.costmap_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&PathClearanceMonitor::costmapCallback, this, std::placeholders::_1),
    sub_option);

  controller_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, options_.controller_node, rmw_qos_profile_parameters, callback_group_);
  costmap_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, options_.costmap_node, rmw_qos_profile_parameters, callback_group_);
}

BT::PortsList PathClearanceMonitor::commonPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Path to check"),
    BT::InputPort<double>(
      "margin", 0.3, "Extra distance [m] beyond RPP's max lookahead to check along the path"),
    BT::InputPort<std::string>("controller_node", "controller_server", "Controller server node"),
    BT::InputPort<std::string>("controller_id", "FollowPath", "RPP plugin name"),
    BT::InputPort<std::string>(
      "costmap_node", "local_costmap/local_costmap", "Local costmap node"),
    BT::InputPort<std::string>(
      "costmap_topic", "local_costmap/costmap_raw", "Local costmap topic"),
    BT::InputPort<double>("transform_tolerance", 0.2, "TF tolerance [s]"),
  };
}

PathClearanceMonitorOptions PathClearanceMonitor::readOptions(
  const std::string & log_name, const BT::TreeNode & node)
{
  PathClearanceMonitorOptions options;
  options.log_name = log_name;
  node.getInput("controller_node", options.controller_node);
  node.getInput("controller_id", options.controller_id);
  node.getInput("costmap_node", options.costmap_node);
  node.getInput("costmap_topic", options.costmap_topic);
  node.getInput("margin", options.margin);
  node.getInput("transform_tolerance", options.transform_tolerance);
  return options;
}

void PathClearanceMonitor::costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  costmap_msg_ = msg;
  ++costmap_seq_;
  costmap_received_ = node_->now().seconds();
}

void PathClearanceMonitor::spinSome()
{
  callback_group_executor_.spin_some();
}

void PathClearanceMonitor::refreshParameters()
{
  parameters_requested_ = false;
  requestParameters();
}

void PathClearanceMonitor::requestParameters()
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
      "%s: parameter services of the controller or the local costmap are not ready",
      options_.log_name.c_str());
    return;
  }

  std::vector<std::string> controller_names;
  for (const auto * name : kControllerParamNames) {
    controller_names.push_back(options_.controller_id + "." + name);
  }
  controller_params_future_ = controller_params_client_->get_parameters(controller_names);
  costmap_params_future_ = costmap_params_client_->get_parameters(
    std::vector<std::string>(std::begin(kCostmapParamNames), std::end(kCostmapParamNames)));
  parameters_requested_ = true;
}

void PathClearanceMonitor::pollParameters()
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
        node_->get_logger(), "%s: failed to read RPP parameters of %s",
        options_.log_name.c_str(), options_.controller_id.c_str());
      parameters_requested_ = false;
    }
    controller_params_future_.reset();
  }
  if (ready(costmap_params_future_)) {
    if (!applyCostmapParameters(costmap_params_future_->get())) {
      RCLCPP_ERROR(
        node_->get_logger(), "%s: failed to read local costmap parameters",
        options_.log_name.c_str());
      parameters_requested_ = false;
    }
    costmap_params_future_.reset();
  }
}

void PathClearanceMonitor::pollParametersWithin(std::chrono::milliseconds timeout)
{
  pollParameters();
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!ready() && (controller_params_future_ || costmap_params_future_)) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return;
    }
    callback_group_executor_.spin_once(deadline - now);
    pollParameters();
  }
}

bool PathClearanceMonitor::applyControllerParameters(
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
  result.margin = options_.margin;
  result.track_unknown_space = track_unknown_space_.value_or(false);
  controller_params_ = result;
  use_collision_detection_ = params[7].as_bool();
  return true;
}

bool PathClearanceMonitor::applyCostmapParameters(const std::vector<rclcpp::Parameter> & params)
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

void PathClearanceMonitor::updateCostmap()
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

bool PathClearanceMonitor::transformPath(
  const nav_msgs::msg::Path & path, const std::string & frame,
  std::vector<geometry_msgs::msg::Pose2D> & out) const
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(frame, path.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "%s: failed to transform the path: %s", options_.log_name.c_str(), e.what());
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

void PathClearanceMonitor::resetEvaluation()
{
  last_eval_.reset();
  last_eval_path_ = nav_msgs::msg::Path();
  last_blocked_ = false;
}

PathClearanceMonitor::FreeStartStatus PathClearanceMonitor::findFreeStart(
  double now, double max_distance, double margin, geometry_msgs::msg::PoseStamped & start)
{
  if (!costmap_msg_) {
    return FreeStartStatus::kNoCostmap;
  }
  if (!footprint_ || now - costmap_received_ > kCostmapTimeout) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "%s: costmap is stale or footprint is unknown",
      options_.log_name.c_str());
    return FreeStartStatus::kUnavailable;
  }
  const std::string & frame = costmap_msg_->header.frame_id;
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, frame, robot_base_frame_, options_.transform_tolerance))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "%s: failed to get the robot pose in %s", options_.log_name.c_str(), frame.c_str());
    return FreeStartStatus::kUnavailable;
  }
  updateCostmap();
  geometry_msgs::msg::Pose2D robot;
  robot.x = robot_pose.pose.position.x;
  robot.y = robot_pose.pose.position.y;
  robot.theta = tf2::getYaw(robot_pose.pose.orientation);

  PathClearanceParams params = controller_params_.value_or(PathClearanceParams());
  params.track_unknown_space = track_unknown_space_.value_or(false);
  const PathClearanceChecker checker(costmap_.get(), *footprint_, params);
  if (!checker.inCollision(robot.x, robot.y, robot.theta)) {
    return FreeStartStatus::kNotNeeded;
  }
  const auto free_pose = checker.findNearestFreePose(robot, max_distance, margin);
  if (!free_pose) {
    return FreeStartStatus::kNotFound;
  }
  start.header.frame_id = frame;
  start.header.stamp = robot_pose.header.stamp;
  start.pose.position.x = free_pose->x;
  start.pose.position.y = free_pose->y;
  start.pose.position.z = 0.0;
  start.pose.orientation = robot_pose.pose.orientation;
  return FreeStartStatus::kFound;
}

PathClearanceMonitor::Status PathClearanceMonitor::evaluate(
  const nav_msgs::msg::Path & path, double now)
{
  if (path.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "%s: no path to check",
      options_.log_name.c_str());
    return Status::kUnavailable;
  }
  if (!costmap_msg_) {
    return Status::kNoCostmap;
  }
  if (now - costmap_received_ > kCostmapTimeout) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "%s: local costmap is stale",
      options_.log_name.c_str());
    return Status::kUnavailable;
  }

  const bool changed = costmap_seq_ != last_eval_seq_ || path != last_eval_path_;
  if (!changed && last_eval_ && now - *last_eval_ < kReevaluatePeriod) {
    return Status::kUnchanged;
  }
  last_eval_ = now;
  last_eval_seq_ = costmap_seq_;
  last_eval_path_ = path;

  const std::string & frame = costmap_msg_->header.frame_id;
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, frame, robot_base_frame_, options_.transform_tolerance))
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "%s: failed to get the robot pose in %s", options_.log_name.c_str(), frame.c_str());
    return Status::kUnavailable;
  }
  std::vector<geometry_msgs::msg::Pose2D> path_2d;
  if (!transformPath(path, frame, path_2d)) {
    return Status::kUnavailable;
  }

  updateCostmap();
  geometry_msgs::msg::Pose2D robot;
  robot.x = robot_pose.pose.position.x;
  robot.y = robot_pose.pose.position.y;
  robot.theta = tf2::getYaw(robot_pose.pose.orientation);

  const PathClearanceChecker checker(costmap_.get(), *footprint_, *controller_params_);
  last_blocked_ = checker.isBlocked(robot, path_2d);
  return Status::kEvaluated;
}

}  // namespace bt_plugins
}  // namespace mg_navigation
