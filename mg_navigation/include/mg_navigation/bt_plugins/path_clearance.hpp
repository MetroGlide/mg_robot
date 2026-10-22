#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// RPP (nav2_regulated_pure_pursuit_controller, Humble 1.1.20) の衝突判定に使われる設定値。
// RPP と同じ値を使うため、controller_server と local_costmap のパラメータから作る。
struct PathClearanceParams
{
  bool use_velocity_scaled_lookahead_dist{false};
  double lookahead_dist{0.6};
  double min_lookahead_dist{0.3};
  double max_lookahead_dist{0.9};
  bool use_rotate_to_heading{true};
  double rotate_to_heading_min_angle{0.785};
  double max_robot_pose_search_dist{4.0};
  bool track_unknown_space{false};
  // RPP の判定範囲の外側に足す余裕 [m]
  double margin{0.3};
};

// RPP が走り出したときに衝突判定で止まるかを、RPP と同等以上に厳しく判定する。
// 次の 3 つのどこかで、フットプリントが致死コストに掛かればブロックとみなす。
//   1. 現在の姿勢
//   2. 現在の姿勢から、先読み距離 (最小・最大) のキャロットへ向かう動き。
//      向きのずれが大きければその場の回転、そうでなければピュアパーシュートの円弧
//   3. 経路の、ロボットの最近傍点から (最大の先読み距離 + 余裕) までの範囲
// 姿勢と経路はコストマップと同じ座標系で与える。
class PathClearanceChecker
{
public:
  PathClearanceChecker(
    nav2_costmap_2d::Costmap2D * costmap,
    const std::vector<geometry_msgs::msg::Point> & footprint,
    const PathClearanceParams & params);

  bool isBlocked(
    const geometry_msgs::msg::Pose2D & robot_pose,
    const std::vector<geometry_msgs::msg::Pose2D> & path) const;

  // RPP の inCollision と同じ判定
  bool inCollision(double x, double y, double theta) const;

  // robot_pose (向きは robot_pose のまま) が衝突しているとき、その衝突している領域の中だけを通って
  // 出られる、衝突しない最も近い位置 (経路の長さで最も近い) を探す。max_distance [m] の外は探さない。
  // margin [m] 以内に衝突する位置がある場所は、出口にしない (出た直後に衝突に戻らないため)。
  // 衝突していないとき、領域の外に出られないとき、max_distance 以内に見つからないときは std::nullopt。
  // 衝突している領域の中だけを通るので、細い進入禁止の線の向こう側へ出るのは、
  // 領域をまたぐ道のりが、手前に出る道のりより短いときだけ
  std::optional<geometry_msgs::msg::Pose2D> findNearestFreePose(
    const geometry_msgs::msg::Pose2D & robot_pose, double max_distance, double margin) const;

private:
  std::vector<geometry_msgs::msg::Pose2D> resamplePathAhead(
    const geometry_msgs::msg::Pose2D & robot_pose,
    const std::vector<geometry_msgs::msg::Pose2D> & path,
    double length) const;
  bool isMotionToCarrotBlocked(
    const geometry_msgs::msg::Pose2D & robot_pose,
    const geometry_msgs::msg::Pose2D & carrot) const;

  nav2_costmap_2d::Costmap2D * costmap_;
  // footprintCostAtPose が const でないため mutable にする
  mutable nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
  collision_checker_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  PathClearanceParams params_;
};

// ブロックを観測した後に、空いた状態が続いたことを判定する。
// 最初から空いている場合は空いたとみなさない (障害物以外が原因の失敗で、
// リカバリーを飛ばして再開と失敗をくり返さないため)。
class ClearanceTracker
{
public:
  void reset();
  // 判定を 1 回記録する。map_seq はその判定に使ったコストマップの通し番号
  void observe(bool blocked, std::uint64_t map_seq, double now);
  // 空いていることを確かめられない状態 (コストマップが古いなど) にする
  void invalidate();
  // ブロックの後、clear_duration [s] 以上、かつ min_clear_maps 枚以上のコストマップで
  // 空いていれば true
  bool isClear(double now, double clear_duration, int min_clear_maps) const;
  bool seenBlocked() const {return seen_blocked_;}
  // ブロックの後に空いた状態を観測し、続いているか確かめている最中なら true
  bool clearPending() const {return clear_since_.has_value();}

private:
  bool seen_blocked_{false};
  std::optional<double> clear_since_;
  int clear_maps_{0};
  std::optional<std::uint64_t> last_counted_map_;
};

// 経路の終点がゴールから tolerance [m] 以内なら true。座標系が違えば false
bool pathReachesGoal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal,
  double tolerance);

// 経路に沿った長さ [m]。姿勢が 2 つ未満なら 0
double pathLength(const nav_msgs::msg::Path & path);

// 経路の遠回りの判定結果
struct DetourCheck
{
  // 経路の長さ [m]
  double path_length{0.0};
  // 経路の始点からゴールまでの直線距離 [m]
  double straight_distance{0.0};
  // 採用できる経路の長さの上限 [m]。判定を無効にしているときは負
  double limit{-1.0};
  // 上限を超えているとき true
  bool exceeded{false};
};

// path の長さが、始点からゴールまでの直線距離 × max_ratio + margin [m] を超えるかを調べる。
// max_ratio が 0 以下なら判定しない (exceeded は常に false)。path が空なら exceeded は false
DetourCheck checkDetour(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal,
  double max_ratio,
  double margin);

// CommitPath の判定。candidate がゴール向けなら candidate を、そうでなく current が
// ゴール向けなら current を返す。どちらも違えば std::nullopt (経路を空にする)
std::optional<nav_msgs::msg::Path> selectPathForGoal(
  const nav_msgs::msg::Path & candidate,
  const nav_msgs::msg::Path & current,
  const geometry_msgs::msg::PoseStamped & goal,
  double tolerance);

}  // namespace bt_plugins
}  // namespace mg_navigation
