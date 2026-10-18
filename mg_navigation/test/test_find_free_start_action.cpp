#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mg_navigation/bt_plugins/find_free_start_action.hpp"
#include "path_clearance_condition_fixture.hpp"

namespace
{

// フィクスチャのコストマップ (local_costmap) と、パラメータサービスを使う
const char * const kTreeXml =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <FindFreeStart free_start="{free_start}" max_distance="2.0" margin="0.1"
                     costmap_node="local_costmap/local_costmap"
                     costmap_topic="local_costmap/costmap_raw"/>
    </BehaviorTree>
  </root>)";

class FindFreeStartActionTest : public mg_navigation::test::PathClearanceConditionTest
{
protected:
  FindFreeStartActionTest()
  {
    factory_.registerNodeType<mg_navigation::bt_plugins::FindFreeStartAction>("FindFreeStart");
    createTree(kTreeXml);
  }

  // ロボット (原点) を中心にした正方形 (一辺 2*half) を致死にしたコストマップ。half が負なら障害物なし
  void publishBlock(double half)
  {
    nav2_msgs::msg::Costmap msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = node_->now();
    msg.metadata.resolution = kResolution;
    msg.metadata.size_x = kSize;
    msg.metadata.size_y = kSize;
    msg.metadata.origin.position.x = kOrigin;
    msg.metadata.origin.position.y = kOrigin;
    msg.data.assign(kSize * kSize, nav2_costmap_2d::FREE_SPACE);
    if (half >= 0.0) {
      for (unsigned int my = 0; my < kSize; ++my) {
        for (unsigned int mx = 0; mx < kSize; ++mx) {
          const double x = kOrigin + (mx + 0.5) * kResolution;
          const double y = kOrigin + (my + 0.5) * kResolution;
          if (std::fabs(x) <= half && std::fabs(y) <= half) {
            msg.data[my * kSize + mx] = nav2_costmap_2d::LETHAL_OBSTACLE;
          }
        }
      }
    }
    costmap_pub_->publish(msg);
  }

  geometry_msgs::msg::PoseStamped freeStart()
  {
    return tree_.rootBlackboard()->get<geometry_msgs::msg::PoseStamped>("free_start");
  }
};

}  // namespace

TEST_F(FindFreeStartActionTest, FindsAPoseOutsideTheBlock)
{
  startParameterServers(true);
  ASSERT_TRUE(
    tickUntil(BT::NodeStatus::SUCCESS, [this]() {publishBlock(0.6);}));
  const auto start = freeStart();
  EXPECT_EQ(start.header.frame_id, "odom");
  // ロボットが領域の中心にいるので、どちらへ出てもフットプリントが領域の外に出る距離
  EXPECT_GT(std::hypot(start.pose.position.x, start.pose.position.y), 0.6);
  EXPECT_NEAR(start.pose.orientation.w, 1.0, 1e-6);
}

TEST_F(FindFreeStartActionTest, NotNeededOutsideTheBlock)
{
  startParameterServers(true);
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishBlock(-1.0);}, std::chrono::milliseconds(1000)));
}

TEST_F(FindFreeStartActionTest, FailsWhenTheWayOutIsTooFar)
{
  startParameterServers(true);
  // max_distance 2.0 m では出られない大きさ
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishBlock(4.0);}, std::chrono::milliseconds(1500)));
}

TEST_F(FindFreeStartActionTest, FailsWithoutFootprint)
{
  // フットプリントを取得できない間は、判定しない
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishBlock(0.6);}, std::chrono::milliseconds(1000)));
}
