#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "mg_msgs/msg/path_candidate_check.hpp"
#include "mg_navigation/bt_plugins/commit_path_action.hpp"
#include "mg_navigation/bt_plugins/commit_path_reporter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using mg_navigation::bt_plugins::CommitPathReporter;

// blackboard に置かれた、その木の CommitPathReporter
std::shared_ptr<CommitPathReporter> reporter(const BT::Blackboard::Ptr & blackboard)
{
  return blackboard->get<std::shared_ptr<CommitPathReporter>>("commit_path_reporter");
}

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
      CommitPathReporter::kCheckTopic, 10,
      [this](mg_msgs::msg::PathCandidateCheck::SharedPtr msg) {last_check_ = *msg;});
    committed_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      CommitPathReporter::kCommittedPathTopic,
      rclcpp::QoS(10).transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {committed_.push_back(*msg);});
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
  // 採用した経路の配信を、1 回の tick の後に集める。
  // transient local なので、接続が tick の後でも最後の 1 件は届く
  std::vector<nav_msgs::msg::Path> committedAfter(BT::Tree & tree)
  {
    tree.tickRoot();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return committed_;
  }

  rclcpp::Subscription<mg_msgs::msg::PathCandidateCheck>::SharedPtr check_sub_;
  std::optional<mg_msgs::msg::PathCandidateCheck> last_check_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr committed_sub_;
  std::vector<nav_msgs::msg::Path> committed_;
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

TEST_F(CommitPathActionTest, PublishesAdoptedCandidate)
{
  blackboard_->set("candidate_path", makePath(10.0));
  auto tree = factory_.createTreeFromText(kTreeXml, blackboard_);
  const auto committed = committedAfter(tree);
  ASSERT_EQ(committed.size(), 1u);
  EXPECT_EQ(committed.front(), makePath(10.0));
}

TEST_F(CommitPathActionTest, DoesNotPublishWhenKeepingCurrentPath)
{
  blackboard_->set("path", makePath(10.0));
  blackboard_->set("candidate_path", nav_msgs::msg::Path());
  auto tree = factory_.createTreeFromText(kTreeXml, blackboard_);
  EXPECT_TRUE(committedAfter(tree).empty());
}

TEST_F(CommitPathActionTest, DoesNotPublishRejectedCandidate)
{
  blackboard_->set("path", makePath(10.0));
  blackboard_->set("candidate_path", makeDetourPath(50.0, 10.0));
  auto tree = factory_.createTreeFromText(kTreeXml, blackboard_);
  EXPECT_TRUE(committedAfter(tree).empty());
}

TEST_F(CommitPathActionTest, PublishesEmptyPathWhenDroppingThePath)
{
  blackboard_->set("path", makePath(5.0));
  blackboard_->set("candidate_path", nav_msgs::msg::Path());
  auto tree = factory_.createTreeFromText(kTreeXml, blackboard_);
  const auto committed = committedAfter(tree);
  ASSERT_EQ(committed.size(), 1u);
  EXPECT_TRUE(committed.front().poses.empty());
  EXPECT_EQ(committed.front().header.frame_id, "map");
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

TEST_F(CommitPathActionTest, RejectionStateIsKeptPerTree)
{
  // イベントログを変化時だけ出すための状態は、木 (blackboard) ごとの CommitPathReporter が持つ
  blackboard_->set("candidate_path", makeDetourPath(50.0, 10.0));
  tick();
  EXPECT_TRUE(reporter(blackboard_)->rejecting());
  blackboard_->set("candidate_path", makePath(10.0));
  tick();
  EXPECT_FALSE(reporter(blackboard_)->rejecting());

  // 別の木は、別の CommitPathReporter で、最初の棄却を自分の状態として記録する
  blackboard_->set("candidate_path", makeDetourPath(50.0, 10.0));
  tick();
  auto other = BT::Blackboard::create();
  other->set("node", node_);
  other->set("goal", makeGoal(10.0));
  other->set("candidate_path", makeDetourPath(50.0, 10.0));
  auto tree = factory_.createTreeFromText(kTreeXml, other);
  tree.tickRoot();
  EXPECT_NE(reporter(other), reporter(blackboard_));
  EXPECT_TRUE(reporter(other)->rejecting());
}

TEST_F(CommitPathActionTest, CommitPathsInATreeShareOnePublisher)
{
  // 配信者が CommitPath ごとにあると、後から購読した表示に、それぞれの古い経路が届く
  const char * const xml =
    R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence>
          <CommitPath candidate="{candidate_path}" goal="{goal}" path="{path}"/>
          <CommitPath candidate="{candidate_path}" goal="{goal}" path="{path}"/>
          <CommitPath candidate="{candidate_path}" goal="{goal}" path="{path}"/>
        </Sequence>
      </BehaviorTree>
    </root>)";
  auto tree = factory_.createTreeFromText(xml, blackboard_);
  EXPECT_EQ(node_->count_publishers(CommitPathReporter::kCommittedPathTopic), 1u);
  EXPECT_EQ(node_->count_publishers(CommitPathReporter::kCheckTopic), 1u);
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
