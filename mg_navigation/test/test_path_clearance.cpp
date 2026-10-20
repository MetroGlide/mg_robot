#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "mg_navigation/bt_plugins/path_clearance.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

using mg_navigation::bt_plugins::ClearanceTracker;
using mg_navigation::bt_plugins::PathClearanceChecker;
using mg_navigation::bt_plugins::PathClearanceParams;
using mg_navigation::bt_plugins::pathReachesGoal;
using mg_navigation::bt_plugins::selectPathForGoal;

namespace
{

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

geometry_msgs::msg::Pose2D pose(double x, double y, double theta)
{
  geometry_msgs::msg::Pose2D p;
  p.x = x;
  p.y = y;
  p.theta = theta;
  return p;
}

class PathClearanceTest : public ::testing::Test
{
protected:
  PathClearanceTest()
  // 10m 四方、原点 (-5, -5)、解像度 0.05m
  : costmap_(200, 200, 0.05, -5.0, -5.0)
  {
    // MG-01 のフットプリントに padding 0.1m を足したもの
    footprint_ = {point(0.5, 0.4), point(0.5, -0.4), point(-0.3, -0.4), point(-0.3, 0.4)};
    params_.use_velocity_scaled_lookahead_dist = true;
    params_.min_lookahead_dist = 1.3;
    params_.max_lookahead_dist = 1.9;
    params_.use_rotate_to_heading = true;
    params_.rotate_to_heading_min_angle = 0.785;
    params_.max_robot_pose_search_dist = 4.0;
    params_.margin = 0.3;
    // x 軸に沿った 5m の直線経路
    for (double x = 0.0; x <= 5.0; x += 0.1) {
      path_.push_back(pose(x, 0.0, 0.0));
    }
  }

  // (x, y) を中心に 3x3 セルの障害物を置く
  void addObstacle(double x, double y, unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    unsigned int mx, my;
    ASSERT_TRUE(costmap_.worldToMap(x, y, mx, my));
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        costmap_.setCost(mx + dx, my + dy, cost);
      }
    }
  }

  bool isBlocked(const geometry_msgs::msg::Pose2D & robot)
  {
    PathClearanceChecker checker(&costmap_, footprint_, params_);
    return checker.isBlocked(robot, path_);
  }

  nav2_costmap_2d::Costmap2D costmap_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  PathClearanceParams params_;
  std::vector<geometry_msgs::msg::Pose2D> path_;
};

}  // namespace

TEST_F(PathClearanceTest, EmptyMapIsClear)
{
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, ObstacleOnPathAheadIsBlocked)
{
  addObstacle(1.0, 0.0);
  EXPECT_TRUE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, ObstacleAtCurrentPoseIsBlocked)
{
  addObstacle(0.5, 0.0);
  EXPECT_TRUE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, ObstacleBeyondCheckedRangeIsClear)
{
  // max_lookahead_dist 1.9 + margin 0.3 + フットプリントの前端 0.5 より先
  addObstacle(3.2, 0.0);
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, ObstacleWithinMarginIsBlocked)
{
  // RPP のキャロット (最大 1.9m) より先だが、margin の範囲内
  addObstacle(2.5, 0.0);
  EXPECT_TRUE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, ObstacleBesideThePathIsClear)
{
  addObstacle(1.0, 1.0);
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, ObstacleOnlyOnPurePursuitArcIsBlocked)
{
  // 経路の帯 (|y| <= 0.4) の外で、向きのずれた (0.6rad) ロボットがキャロットへ向かう円弧の上
  addObstacle(0.65, 0.55);
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
  EXPECT_TRUE(isBlocked(pose(0.0, 0.0, 0.6)));
}

TEST_F(PathClearanceTest, ObstacleOnlyInRotationSweepIsBlocked)
{
  // 向きのずれが rotate_to_heading_min_angle を超えると RPP はその場で回転する。
  // 回転の途中 (約 0.6rad) でだけフットプリントの前端に掛かる位置
  addObstacle(0.6 * std::cos(1.2), 0.6 * std::sin(1.2));
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
  EXPECT_TRUE(isBlocked(pose(0.0, 0.0, 1.2)));
}

TEST_F(PathClearanceTest, UnknownCellsFollowTrackUnknownSpace)
{
  addObstacle(1.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  params_.track_unknown_space = true;
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
  params_.track_unknown_space = false;
  EXPECT_TRUE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, InscribedCostIsNotBlocked)
{
  // RPP と同じく、致死コスト未満 (膨張域) では止まらない
  addObstacle(1.0, 0.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, EmptyPathIsBlocked)
{
  path_.clear();
  EXPECT_TRUE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST_F(PathClearanceTest, StaticLookaheadUsesLookaheadDist)
{
  params_.use_velocity_scaled_lookahead_dist = false;
  params_.lookahead_dist = 0.6;
  // lookahead 0.6 + margin 0.3 + 前端 0.5 = 1.4m より先は見ない
  addObstacle(2.0, 0.0);
  EXPECT_FALSE(isBlocked(pose(0.0, 0.0, 0.0)));
}

TEST(ClearanceTrackerTest, ClearFromTheStartIsNotClear)
{
  ClearanceTracker tracker;
  tracker.observe(false, 1, 0.0);
  tracker.observe(false, 2, 0.5);
  tracker.observe(false, 3, 1.0);
  EXPECT_FALSE(tracker.isClear(2.0, 1.0, 2));
}

TEST(ClearanceTrackerTest, ClearAfterBlockedNeedsDurationAndMaps)
{
  ClearanceTracker tracker;
  tracker.observe(true, 1, 0.0);
  tracker.observe(false, 2, 0.5);
  EXPECT_FALSE(tracker.isClear(1.0, 1.0, 2));  // 時間が足りない
  // 同じコストマップを姿勢の変化で判定し直しても、枚数は増えない
  tracker.observe(false, 2, 1.4);
  EXPECT_FALSE(tracker.isClear(1.5, 1.0, 2));
  tracker.observe(false, 3, 1.5);
  EXPECT_TRUE(tracker.isClear(1.5, 1.0, 2));
}

TEST(ClearanceTrackerTest, BlockedAgainRestartsTheDuration)
{
  ClearanceTracker tracker;
  tracker.observe(true, 1, 0.0);
  tracker.observe(false, 2, 0.5);
  tracker.observe(true, 3, 1.0);
  tracker.observe(false, 4, 1.5);
  tracker.observe(false, 5, 2.0);
  EXPECT_FALSE(tracker.isClear(2.0, 1.0, 2));
  EXPECT_TRUE(tracker.isClear(2.5, 1.0, 2));
}

TEST(ClearanceTrackerTest, InvalidateAndReset)
{
  ClearanceTracker tracker;
  tracker.observe(true, 1, 0.0);
  tracker.observe(false, 2, 0.5);
  tracker.observe(false, 3, 1.0);
  tracker.invalidate();
  EXPECT_FALSE(tracker.isClear(2.0, 1.0, 2));
  EXPECT_TRUE(tracker.seenBlocked());
  tracker.reset();
  EXPECT_FALSE(tracker.seenBlocked());
}

namespace
{

nav_msgs::msg::Path makePath(const std::string & frame, double end_x, double end_y)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped end;
  end.pose.position.x = end_x;
  end.pose.position.y = end_y;
  path.poses = {start, end};
  return path;
}

geometry_msgs::msg::PoseStamped makeGoal(const std::string & frame, double x, double y)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame;
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  return goal;
}

}  // namespace

// 衝突している領域から出られる、最も近い位置を探す
class FreePoseTest : public PathClearanceTest
{
protected:
  // x の範囲 [x0, x1]、y の範囲 [y0, y1] を致死にする
  void fillBlock(double x0, double x1, double y0, double y1)
  {
    unsigned int ax, ay, bx, by;
    ASSERT_TRUE(costmap_.worldToMap(x0, y0, ax, ay));
    ASSERT_TRUE(costmap_.worldToMap(x1, y1, bx, by));
    for (unsigned int mx = ax; mx <= bx; ++mx) {
      for (unsigned int my = ay; my <= by; ++my) {
        costmap_.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  std::optional<geometry_msgs::msg::Pose2D> find(
    const geometry_msgs::msg::Pose2D & robot, double max_distance, double margin = 0.1)
  {
    PathClearanceChecker checker(&costmap_, footprint_, params_);
    return checker.findNearestFreePose(robot, max_distance, margin);
  }

  bool inCollision(const geometry_msgs::msg::Pose2D & p)
  {
    PathClearanceChecker checker(&costmap_, footprint_, params_);
    return checker.inCollision(p.x, p.y, p.theta);
  }
};

TEST_F(FreePoseTest, NotNeededWhenNotColliding)
{
  EXPECT_FALSE(find(pose(0.0, 0.0, 0.0), 3.0).has_value());
}

TEST_F(FreePoseTest, FindsTheNearestExitOfTheBlock)
{
  // 3m 四方の致死の領域。ロボットは領域の左端の近く (x=-1.2) にいる
  fillBlock(-1.5, 1.5, -1.5, 1.5);
  const auto robot = pose(-1.2, 0.0, 0.0);
  ASSERT_TRUE(inCollision(robot));
  const auto exit = find(robot, 3.0);
  ASSERT_TRUE(exit.has_value());
  EXPECT_FALSE(inCollision(*exit));
  // 近い左側に出る (右側は 3m 以上先)。向きは変えない
  EXPECT_LT(exit->x, robot.x);
  EXPECT_DOUBLE_EQ(exit->theta, 0.0);
  EXPECT_LT(std::hypot(exit->x - robot.x, exit->y - robot.y), 1.2);
}

TEST_F(FreePoseTest, ExitKeepsTheMarginFromTheBlock)
{
  fillBlock(-1.5, 1.5, -1.5, 1.5);
  const auto robot = pose(-1.2, 0.0, 0.0);
  const auto without_margin = find(robot, 3.0, 0.0);
  const auto with_margin = find(robot, 3.0, 0.3);
  ASSERT_TRUE(without_margin.has_value());
  ASSERT_TRUE(with_margin.has_value());
  // 余裕を求めると、出口が領域から遠くなる
  EXPECT_LT(with_margin->x, without_margin->x);
  for (int i = 0; i < 8; ++i) {
    const double angle = i * M_PI / 4.0;
    EXPECT_FALSE(
      inCollision(
        pose(
          with_margin->x + 0.3 * std::cos(angle), with_margin->y + 0.3 * std::sin(angle), 0.0)));
  }
}

TEST_F(FreePoseTest, NoExitWithinMaxDistance)
{
  fillBlock(-1.5, 1.5, -1.5, 1.5);
  EXPECT_FALSE(find(pose(0.0, 0.0, 0.0), 0.5).has_value());
}

TEST_F(FreePoseTest, NoExitWhenEverythingIsBlocked)
{
  fillBlock(-4.9, 4.9, -4.9, 4.9);
  EXPECT_FALSE(find(pose(0.0, 0.0, 0.0), 2.0).has_value());
}

TEST(PathForGoalTest, PathReachesGoal)
{
  const auto goal = makeGoal("map", 10.0, 0.0);
  EXPECT_TRUE(pathReachesGoal(makePath("map", 10.5, 0.0), goal, 0.6));
  EXPECT_FALSE(pathReachesGoal(makePath("map", 11.0, 0.0), goal, 0.6));
  EXPECT_FALSE(pathReachesGoal(makePath("odom", 10.0, 0.0), goal, 0.6));
  EXPECT_FALSE(pathReachesGoal(nav_msgs::msg::Path(), goal, 0.6));
}

TEST(PathForGoalTest, SelectPathForGoal)
{
  const auto goal = makeGoal("map", 10.0, 0.0);
  const auto to_goal = makePath("map", 10.0, 0.0);
  const auto to_other = makePath("map", 5.0, 0.0);
  const nav_msgs::msg::Path empty;

  // 新しい経路がゴール向けなら採用する
  EXPECT_EQ(*selectPathForGoal(to_goal, to_other, goal, 0.6), to_goal);
  // 計画が失敗しても、今の経路がゴール向けなら保つ
  EXPECT_EQ(*selectPathForGoal(empty, to_goal, goal, 0.6), to_goal);
  // 古いゴール宛ての計画結果では上書きしない
  EXPECT_EQ(*selectPathForGoal(to_other, to_goal, goal, 0.6), to_goal);
  // ゴール向けの経路がなければ空にする
  EXPECT_FALSE(selectPathForGoal(empty, to_other, goal, 0.6).has_value());
}
