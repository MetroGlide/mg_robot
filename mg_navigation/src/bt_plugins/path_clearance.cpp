#include "mg_navigation/bt_plugins/path_clearance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

namespace
{

// その場の回転を掃引するときの角度の刻み [rad]
constexpr double kRotationStep = 0.05;

double distance(
  const geometry_msgs::msg::Pose2D & a, const geometry_msgs::msg::Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

PathClearanceChecker::PathClearanceChecker(
  nav2_costmap_2d::Costmap2D * costmap,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const PathClearanceParams & params)
: costmap_(costmap),
  collision_checker_(costmap),
  footprint_(footprint),
  params_(params)
{
}

bool PathClearanceChecker::inCollision(double x, double y, double theta) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    // RPP はコストマップの外を衝突としない
    return false;
  }

  const double footprint_cost =
    collision_checker_.footprintCostAtPose(x, y, theta, footprint_);
  if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
    params_.track_unknown_space)
  {
    return false;
  }
  return footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

std::vector<geometry_msgs::msg::Pose2D> PathClearanceChecker::resamplePathAhead(
  const geometry_msgs::msg::Pose2D & robot_pose,
  const std::vector<geometry_msgs::msg::Pose2D> & path,
  double length) const
{
  std::vector<geometry_msgs::msg::Pose2D> resampled;
  if (path.empty()) {
    return resampled;
  }

  // RPP と同じく、経路の先頭から max_robot_pose_search_dist までの中で最近傍点を探す
  std::size_t closest = 0;
  double closest_dist = std::numeric_limits<double>::max();
  double integrated = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      integrated += distance(path[i - 1], path[i]);
      if (integrated > params_.max_robot_pose_search_dist) {
        break;
      }
    }
    const double d = distance(path[i], robot_pose);
    if (d < closest_dist) {
      closest_dist = d;
      closest = i;
    }
  }

  // 最近傍点から length まで、コストマップの解像度の間隔で並べ直す。
  // 向きは経路の点の向きではなく、区間の進行方向を使う
  const double step = costmap_->getResolution();
  double travelled = 0.0;
  for (std::size_t i = closest; i + 1 < path.size() && travelled <= length; ++i) {
    const auto & a = path[i];
    const auto & b = path[i + 1];
    const double seg = distance(a, b);
    if (seg < 1e-6) {
      continue;
    }
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    for (double s = 0.0; s < seg && travelled + s <= length; s += step) {
      geometry_msgs::msg::Pose2D p;
      p.x = a.x + (b.x - a.x) * s / seg;
      p.y = a.y + (b.y - a.y) * s / seg;
      p.theta = heading;
      resampled.push_back(p);
    }
    travelled += seg;
  }

  // 経路の終点 (length 以内なら)
  if (travelled <= length || resampled.empty()) {
    geometry_msgs::msg::Pose2D last = path.back();
    if (path.size() >= 2) {
      const auto & prev = path[path.size() - 2];
      if (distance(prev, last) > 1e-6) {
        last.theta = std::atan2(last.y - prev.y, last.x - prev.x);
      }
    }
    resampled.push_back(last);
  }
  return resampled;
}

bool PathClearanceChecker::isMotionToCarrotBlocked(
  const geometry_msgs::msg::Pose2D & robot_pose,
  const geometry_msgs::msg::Pose2D & carrot) const
{
  // キャロットをロボット座標系に移す
  const double dx = carrot.x - robot_pose.x;
  const double dy = carrot.y - robot_pose.y;
  const double c = std::cos(robot_pose.theta);
  const double s = std::sin(robot_pose.theta);
  const double carrot_x = c * dx + s * dy;
  const double carrot_y = -s * dx + c * dy;
  const double carrot_dist2 = carrot_x * carrot_x + carrot_y * carrot_y;
  const double carrot_dist = std::sqrt(carrot_dist2);

  const double angle_to_carrot = std::atan2(carrot_y, carrot_x);
  if (params_.use_rotate_to_heading &&
    std::fabs(angle_to_carrot) > params_.rotate_to_heading_min_angle)
  {
    // RPP はその場で回転する。回転後の向きまで掃引する
    const int steps = static_cast<int>(std::ceil(std::fabs(angle_to_carrot) / kRotationStep));
    for (int i = 1; i <= steps; ++i) {
      const double theta = robot_pose.theta + angle_to_carrot * i / steps;
      if (inCollision(robot_pose.x, robot_pose.y, theta)) {
        return true;
      }
    }
    return false;
  }

  // RPP のピュアパーシュートの円弧を、キャロットまでの距離だけ進める
  const double curvature = carrot_dist2 > 0.001 ? 2.0 * carrot_y / carrot_dist2 : 0.0;
  const double step = costmap_->getResolution();
  geometry_msgs::msg::Pose2D pose = robot_pose;
  while (true) {
    pose.x += step * std::cos(pose.theta);
    pose.y += step * std::sin(pose.theta);
    pose.theta += step * curvature;
    if (distance(pose, robot_pose) > carrot_dist) {
      break;
    }
    if (inCollision(pose.x, pose.y, pose.theta)) {
      return true;
    }
  }
  return false;
}

bool PathClearanceChecker::isBlocked(
  const geometry_msgs::msg::Pose2D & robot_pose,
  const std::vector<geometry_msgs::msg::Pose2D> & path) const
{
  if (inCollision(robot_pose.x, robot_pose.y, robot_pose.theta)) {
    return true;
  }

  std::vector<double> lookaheads;
  if (params_.use_velocity_scaled_lookahead_dist) {
    lookaheads = {params_.min_lookahead_dist, params_.max_lookahead_dist};
  } else {
    lookaheads = {params_.lookahead_dist};
  }
  const double band_length =
    *std::max_element(lookaheads.begin(), lookaheads.end()) + params_.margin;

  const auto ahead = resamplePathAhead(robot_pose, path, band_length);
  if (ahead.empty()) {
    return true;
  }

  for (const auto & p : ahead) {
    if (inCollision(p.x, p.y, p.theta)) {
      return true;
    }
  }

  for (const double lookahead : lookaheads) {
    // RPP の getLookAheadPoint と同じく、ロボットからの直線距離が先読み距離を超える最初の点
    auto carrot = std::find_if(
      ahead.begin(), ahead.end(), [&](const auto & p) {
        return distance(p, robot_pose) >= lookahead;
      });
    if (carrot == ahead.end()) {
      carrot = std::prev(ahead.end());
    }
    if (isMotionToCarrotBlocked(robot_pose, *carrot)) {
      return true;
    }
  }
  return false;
}

void ClearanceTracker::reset()
{
  seen_blocked_ = false;
  invalidate();
}

void ClearanceTracker::invalidate()
{
  clear_since_.reset();
  clear_maps_ = 0;
  last_counted_map_.reset();
}

void ClearanceTracker::observe(bool blocked, std::uint64_t map_seq, double now)
{
  if (blocked) {
    seen_blocked_ = true;
    invalidate();
    return;
  }
  if (!seen_blocked_) {
    return;
  }
  if (!clear_since_) {
    clear_since_ = now;
  }
  if (!last_counted_map_ || *last_counted_map_ != map_seq) {
    ++clear_maps_;
    last_counted_map_ = map_seq;
  }
}

bool ClearanceTracker::isClear(double now, double clear_duration, int min_clear_maps) const
{
  return seen_blocked_ && clear_since_ &&
         now - *clear_since_ >= clear_duration &&
         clear_maps_ >= min_clear_maps;
}

bool pathReachesGoal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal,
  double tolerance)
{
  if (path.poses.empty() || path.header.frame_id != goal.header.frame_id) {
    return false;
  }
  const auto & end = path.poses.back().pose.position;
  return std::hypot(
    end.x - goal.pose.position.x, end.y - goal.pose.position.y) <= tolerance;
}

std::optional<nav_msgs::msg::Path> selectPathForGoal(
  const nav_msgs::msg::Path & candidate,
  const nav_msgs::msg::Path & current,
  const geometry_msgs::msg::PoseStamped & goal,
  double tolerance)
{
  if (pathReachesGoal(candidate, goal, tolerance)) {
    return candidate;
  }
  if (pathReachesGoal(current, goal, tolerance)) {
    return current;
  }
  return std::nullopt;
}

}  // namespace bt_plugins
}  // namespace mg_navigation
