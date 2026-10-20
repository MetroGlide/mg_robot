#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "mg_msgs/msg/path_candidate_check.hpp"
#include "mg_navigation/bt_plugins/commit_path_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

const char * const kTreeXml =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <CommitPath candidate="{candidate_path}" goal="{goal}" path="{path}"/>
    </BehaviorTree>
  </root>)";

nav_msgs::msg::Path makePath(double end_x)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped end = start;
  end.pose.position.x = end_x;
  path.poses = {start, end};
  return path;
}

geometry_msgs::msg::PoseStamped makeGoal(double x)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  return goal;
}

// (0, 0) から x 軸上の中継点 (via_x, 0) を経て end_x へ向かう経路。中継点が遠いほど遠回りになる
nav_msgs::msg::Path makeDetourPath(double via_x, double end_x)
{
  nav_msgs::msg::Path path = makePath(end_x);
  geometry_msgs::msg::PoseStamped via = path.poses.front();
  via.pose.position.x = via_x;
  path.poses.insert(path.poses.begin() + 1, via);
  return path;
}

class CommitPathActionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  CommitPathActionTest()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_navigator_test");
    check_sub_ = node_->create_subscription<mg_msgs::msg::PathCandidateCheck>(
      mg_navigation::bt_plugins::CommitPathAction::kCheckTopic, 10,
      [this](mg_msgs::msg::PathCandidateCheck::SharedPtr msg) {last_check_ = *msg;});
    factory_.registerNodeType<mg_navigation::bt_plugins::CommitPathAction>("CommitPath");
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", node_);
    blackboard_->set("goal", makeGoal(10.0));
  }

  BT::NodeStatus tick(const char * xml = kTreeXml)
  {
    auto tree = factory_.createTreeFromText(xml, blackboard_);
    return tree.tickRoot();
  }

  // 判定結果が届くまで tick をくり返し、最後の tick の結果を返す。
  // 購読の接続前に配信した分は届かないので、1 回の tick では待てない
  BT::NodeStatus tickUntilCheck(const char * xml = kTreeXml)
  {
    auto tree = factory_.createTreeFromText(xml, blackboard_);
    BT::NodeStatus status = tree.tickRoot();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!last_check_ && std::chrono::steady_clock::now() < deadline) {
      status = tree.tickRoot();
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return status;
  }

  nav_msgs::msg::Path path() const
  {
    return blackboard_->get<nav_msgs::msg::Path>("path");
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<mg_msgs::msg::PathCandidateCheck>::SharedPtr check_sub_;
  std::optional<mg_msgs::msg::PathCandidateCheck> last_check_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

}  // namespace

TEST_F(CommitPathActionTest, CommitsCandidateToGoal)
{
  blackboard_->set("candidate_path", makePath(10.0));
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(path(), makePath(10.0));
}

TEST_F(CommitPathActionTest, KeepsCurrentPathWhenPlanningFailed)
{
  blackboard_->set("path", makePath(10.0));
  blackboard_->set("candidate_path", nav_msgs::msg::Path());
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(path(), makePath(10.0));
}

TEST_F(CommitPathActionTest, DropsPathToPreviousGoal)
{
  blackboard_->set("path", makePath(5.0));
  blackboard_->set("candidate_path", nav_msgs::msg::Path());
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(CommitPathActionTest, FailsWithoutAnyPath)
{
  // candidate も path もまだ書かれていない (走行開始直後に計画が失敗した)
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(path().poses.empty());
}

// 直線距離 10m に対し、既定の上限は 2 * 10 + 10 = 30m

TEST_F(CommitPathActionTest, RejectsTooLongCandidateAndKeepsCurrentPath)
{
  blackboard_->set("path", makePath(10.0));
  blackboard_->set("candidate_path", makeDetourPath(50.0, 10.0));
  EXPECT_EQ(tickUntilCheck(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(path(), makePath(10.0));
  ASSERT_TRUE(last_check_.has_value());
  EXPECT_TRUE(last_check_->rejected);
  EXPECT_FLOAT_EQ(last_check_->path_length, 90.0f);
  EXPECT_FLOAT_EQ(last_check_->straight_distance, 10.0f);
  EXPECT_FLOAT_EQ(last_check_->limit, 30.0f);
}

TEST_F(CommitPathActionTest, FailsWhenTooLongCandidateIsTheOnlyPath)
{
  blackboard_->set("candidate_path", makeDetourPath(50.0, 10.0));
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(path().poses.empty());
}

TEST_F(CommitPathActionTest, AcceptsDetourWithinLimit)
{
  // 経路長 10 + 10 + 10 = 30m は上限ちょうど (超えない)
  blackboard_->set("candidate_path", makeDetourPath(20.0, 10.0));
  EXPECT_EQ(tickUntilCheck(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(path(), makeDetourPath(20.0, 10.0));
  ASSERT_TRUE(last_check_.has_value());
  EXPECT_FALSE(last_check_->rejected);
}

TEST_F(CommitPathActionTest, DetourLimitIsDisabledByZeroRatio)
{
  const char * const xml =
    R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <CommitPath candidate="{candidate_path}" goal="{goal}" path="{path}"
                    max_detour_ratio="0"/>
      </BehaviorTree>
    </root>)";
  blackboard_->set("candidate_path", makeDetourPath(50.0, 10.0));
  EXPECT_EQ(tickUntilCheck(xml), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(path(), makeDetourPath(50.0, 10.0));
  ASSERT_TRUE(last_check_.has_value());
  EXPECT_FALSE(last_check_->rejected);
  EXPECT_LT(last_check_->limit, 0.0f);
}

TEST_F(CommitPathActionTest, MarginAllowsDetourToNearGoal)
{
  // 直線距離 1m のゴールでも、余裕の分 (2 * 1 + 10 = 12m) までの迂回は採用する
  blackboard_->set("goal", makeGoal(1.0));
  blackboard_->set("candidate_path", makeDetourPath(5.0, 1.0));
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(path(), makeDetourPath(5.0, 1.0));
}
