#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
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

struct PathClearanceMonitorOptions
{
  // ログの先頭に付ける名前 (BT ノードの型名)
  std::string log_name;
  std::string controller_node;
  std::string controller_id;
  std::string costmap_node;
  std::string costmap_topic;
  double margin{0.3};
  double transform_tolerance{0.2};
};

// BT 条件ノードから、経路が RPP の衝突判定で塞がっているかを PathClearanceChecker で調べる。
// 設定値は controller_server の RPP と local_costmap のパラメータを読み (重複して持たない)、
// local costmap は costmap_raw を購読する。コールバックは tick の中で処理する。
class PathClearanceMonitor
{
public:
  enum class Status
  {
    // 判定した (結果は lastBlocked())
    kEvaluated,
    // コストマップも経路も変わらず、前回の判定から間もないので判定しなかった
    kUnchanged,
    // コストマップをまだ受け取っていない
    kNoCostmap,
    // 判定できない (経路がない、コストマップが古い、TF が取れない)
    kUnavailable,
  };

  PathClearanceMonitor(
    const rclcpp::Node::SharedPtr & node, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const PathClearanceMonitorOptions & options);

  // 両ノードに共通の入力ポート
  static BT::PortsList commonPorts();
  // commonPorts の値を config から読む
  static PathClearanceMonitorOptions readOptions(
    const std::string & log_name, const BT::TreeNode & node);

  // 受信したコストマップとパラメータの応答を処理する。tick のはじめに呼ぶ
  void spinSome();
  // 設定値を取得し直す (取得済みの値は、新しい値が届くまで使う)
  void refreshParameters();
  // パラメータの応答を取り込み、未取得なら要求する
  void pollParameters();
  // pollParameters に加え、要求中の応答を timeout まで待つ (要求できないときは待たない)
  void pollParametersWithin(std::chrono::milliseconds timeout);
  // 設定値をすべて取得済みなら true
  bool ready() const {return controller_params_.has_value() && footprint_.has_value();}
  bool useCollisionDetection() const {return use_collision_detection_.value_or(false);}

  Status evaluate(const nav_msgs::msg::Path & path, double now);
  // 判定の記録を消し、次の evaluate で必ず判定する
  void resetEvaluation();

  enum class FreeStartStatus
  {
    // 衝突している領域の外の位置を見つけた
    kFound,
    // 今の姿勢が衝突していない (探す必要がない)
    kNotNeeded,
    // 衝突している領域から max_distance 以内に出られない
    kNotFound,
    // コストマップをまだ受け取っていない
    kNoCostmap,
    // 判定できない (フットプリントを取得できていない、コストマップが古い、TF が取れない)
    kUnavailable,
  };

  // フットプリントを取得済みなら true (findFreeStart に必要なのはこれだけ)
  bool footprintReady() const {return footprint_.has_value();}

  // ロボットの今の姿勢が、コストマップで衝突しているとき、その領域の外の最も近い位置を start に入れる。
  // 向きはロボットの今の向きのまま。出口は margin [m] 以内に衝突がない位置にする
  FreeStartStatus findFreeStart(
    double now, double max_distance, double margin, geometry_msgs::msg::PoseStamped & start);

  bool lastBlocked() const {return last_blocked_;}
  std::uint64_t costmapSeq() const {return costmap_seq_;}
  double costmapReceived() const {return costmap_received_;}

private:
  void costmapCallback(nav2_msgs::msg::Costmap::SharedPtr msg);
  void requestParameters();
  bool applyControllerParameters(const std::vector<rclcpp::Parameter> & params);
  bool applyCostmapParameters(const std::vector<rclcpp::Parameter> & params);
  bool transformPath(
    const nav_msgs::msg::Path & path, const std::string & frame,
    std::vector<geometry_msgs::msg::Pose2D> & out) const;
  void updateCostmap();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  PathClearanceMonitorOptions options_;

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

  std::optional<double> last_eval_;
  std::uint64_t last_eval_seq_{0};
  nav_msgs::msg::Path last_eval_path_;
  bool last_blocked_{false};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
