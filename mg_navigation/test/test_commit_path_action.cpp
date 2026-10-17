#include <gtest/gtest.h>

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "mg_navigation/bt_plugins/commit_path_action.hpp"

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

class CommitPathActionTest : public ::testing::Test
{
protected:
  CommitPathActionTest()
  {
    factory_.registerNodeType<mg_navigation::bt_plugins::CommitPathAction>("CommitPath");
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("goal", makeGoal(10.0));
  }

  BT::NodeStatus tick()
  {
    auto tree = factory_.createTreeFromText(kTreeXml, blackboard_);
    return tree.tickRoot();
  }

  nav_msgs::msg::Path path() const
  {
    return blackboard_->get<nav_msgs::msg::Path>("path");
  }

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
