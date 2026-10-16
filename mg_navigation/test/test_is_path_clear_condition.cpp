#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "mg_navigation/bt_plugins/is_path_clear_condition.hpp"
#include "path_clearance_condition_fixture.hpp"

namespace
{

const char * const kTreeXml =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <IsPathClear path="{path}" clear_duration="0.5"/>
    </BehaviorTree>
  </root>)";

class IsPathClearConditionTest : public mg_navigation::test::PathClearanceConditionTest
{
protected:
  IsPathClearConditionTest()
  {
    factory_.registerNodeType<mg_navigation::bt_plugins::IsPathClearCondition>("IsPathClear");
    createTree(kTreeXml);
  }
};

TEST_F(IsPathClearConditionTest, ClearAfterBlocked)
{
  startParameterServers(true);
  // 塞がっている間は空いたと言わない
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(1.0);},
      std::chrono::milliseconds(1500)));
  // 空いた状態が続けば SUCCESS
  EXPECT_TRUE(tickUntil(BT::NodeStatus::SUCCESS, [this]() {publishCostmap(-1.0);}));
}

TEST_F(IsPathClearConditionTest, ClearFromTheStartIsNotSuccess)
{
  // 障害物以外が原因でリカバリーに入ったときは、空いていても走行へ戻さない
  startParameterServers(true);
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(-1.0);},
      std::chrono::milliseconds(2000)));
}

TEST_F(IsPathClearConditionTest, NewRecoveryForgetsThePreviousBlock)
{
  startParameterServers(true);
  ASSERT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(1.0);},
      std::chrono::milliseconds(1500)));
  ASSERT_TRUE(tickUntil(BT::NodeStatus::SUCCESS, [this]() {publishCostmap(-1.0);}));
  // tick の間隔が空くと新しいリカバリーとみなし、前のリカバリーのブロックを使わない
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(-1.0);},
      std::chrono::milliseconds(1500)));
}

TEST_F(IsPathClearConditionTest, CollisionDetectionDisabledNeverClear)
{
  startParameterServers(false);
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(1.0);},
      std::chrono::milliseconds(1000)));
  EXPECT_TRUE(
    tickFor(
      BT::NodeStatus::FAILURE, [this]() {publishCostmap(-1.0);},
      std::chrono::milliseconds(1500)));
}

}  // namespace
