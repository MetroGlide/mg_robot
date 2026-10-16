#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "mg_navigation/bt_plugins/is_path_blocked_condition.hpp"
#include "path_clearance_condition_fixture.hpp"

namespace
{

const char * const kTreeXml =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <IsPathBlocked path="{path}"/>
    </BehaviorTree>
  </root>)";

class IsPathBlockedConditionTest : public mg_navigation::test::PathClearanceConditionTest
{
protected:
  IsPathBlockedConditionTest()
  {
    factory_.registerNodeType<mg_navigation::bt_plugins::IsPathBlockedCondition>(
      "IsPathBlocked");
    createTree(kTreeXml);
  }
};

TEST_F(IsPathBlockedConditionTest, WithoutParametersIsNotBlocked)
{
  // 設定値を取得できない間は、障害物があっても再計画を止めない
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(1.0);},
      std::chrono::milliseconds(1500)));
}

TEST_F(IsPathBlockedConditionTest, BlockedOnlyWhileObstacleIsOnThePath)
{
  startParameterServers(true);
  ASSERT_TRUE(tickUntil(BT::NodeStatus::SUCCESS, [this]() {publishCostmap(1.0);}));
  EXPECT_TRUE(tickUntil(BT::NodeStatus::FAILURE, [this]() {publishCostmap(-1.0);}));
  // 判定範囲 (最大の先読み距離 + margin) より遠い障害物では止めない
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(3.5);},
      std::chrono::milliseconds(500)));
}

TEST_F(IsPathBlockedConditionTest, FirstTickJudgesWhenServersAreUp)
{
  // リカバリーの段階の中で初めて tick されたときも、その tick で判定する
  startParameterServers(true);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  while (std::chrono::steady_clock::now() < deadline) {
    publishCostmap(1.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(tree_.tickRoot(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsPathBlockedConditionTest, StaleCostmapIsNotBlocked)
{
  startParameterServers(true);
  ASSERT_TRUE(tickUntil(BT::NodeStatus::SUCCESS, [this]() {publishCostmap(1.0);}));
  EXPECT_TRUE(tickUntil(BT::NodeStatus::FAILURE, []() {}, std::chrono::milliseconds(4000)));
}

TEST_F(IsPathBlockedConditionTest, CollisionDetectionDisabledIsNotBlocked)
{
  startParameterServers(false);
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(1.0);},
      std::chrono::milliseconds(2000)));
}

}  // namespace
