#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "mg_navigation/bt_plugins/path_clearance.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace mg_navigation
{
namespace bt_plugins
{

// リカバリー中に、RPP が衝突判定なしで走り出せる状態になったかを判定する BT 条件ノード。
// ReactiveFallback に置き、SUCCESS で実行中の回避行動を中断して走行に戻す。
//
// - 判定は PathClearanceChecker (RPP と同等以上に厳しい判定)。設定値は controller_server の
//   RPP と local_costmap のパラメータを読む (このノードには重複して持たない)
// - リカバリーの間にブロックを観測し、その後に空いた状態が clear_duration 続いたときだけ
//   SUCCESS を返す。最初から空いているとき (障害物以外が原因の失敗) は FAILURE
// - tick の間隔が空いたら新しいリカバリーとみなし、状態のリセットと設定値の再取得をする
class IsPathClearCondition : public BT::ConditionNode
{
public:
  IsPathClearCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsPathClearCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to check"),
      BT::InputPort<double>(
        "margin", 0.3, "Extra distance [m] beyond RPP's max lookahead to check along the path"),
      BT::InputPort<double>(
        "clear_duration", 1.0, "Time [s] the path must stay clear after being blocked"),
      BT::InputPort<std::string>("controller_node", "controller_server", "Controller server node"),
      BT::InputPort<std::string>("controller_id", "FollowPath", "RPP plugin name"),
      BT::InputPort<std::string>(
        "costmap_node", "local_costmap/local_costmap", "Local costmap node"),
      BT::InputPort<std::string>(
        "costmap_topic", "local_costmap/costmap_raw", "Local costmap topic"),
      BT::InputPort<double>("transform_tolerance", 0.2, "TF tolerance [s]"),
    };
  }

private:
  void costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg);
  void startEpisode(double now);
  void requestParameters();
  void pollParameters();
  bool applyControllerParameters(const std::vector<rclcpp::Parameter> & params);
  bool applyCostmapParameters(const std::vector<rclcpp::Parameter> & params);
  void evaluate(double now);
  bool transformPath(
    const nav_msgs::msg::Path & path, const std::string & frame,
    std::vector<geometry_msgs::msg::Pose2D> & out) const;
  void updateCostmap();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::string controller_id_;
  double margin_;
  double clear_duration_;
  double transform_tolerance_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr costmap_msg_;
  std::uint64_t costmap_seq_{0};
  double costmap_received_{0.0};
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::uint64_t costmap_built_seq_{0};

  std::shared_ptr<rclcpp::AsyncParametersClient> controller_params_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> costmap_params_client_;
  std::optional<std::shared_future<std::vector<rclcpp::Parameter>>> controller_params_future_;
  std::optional<std::shared_future<std::vector<rclcpp::Parameter>>> costmap_params_future_;
  bool parameters_requested_{false};
  std::optional<double> last_request_attempt_;

  // controller_server と local_costmap の両方から取得できたときだけ値を持つ
  std::optional<PathClearanceParams> controller_params_;
  std::optional<bool> use_collision_detection_;
  std::optional<std::vector<geometry_msgs::msg::Point>> footprint_;
  std::optional<bool> track_unknown_space_;
  std::string robot_base_frame_;

  ClearanceTracker tracker_;
  std::optional<double> last_tick_;
  double episode_start_{0.0};
  std::optional<double> last_eval_;
  std::uint64_t last_eval_seq_{0};
  nav_msgs::msg::Path last_eval_path_;
  bool reported_clear_{false};
  bool reported_blocked_{false};
  bool reported_first_check_{false};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
