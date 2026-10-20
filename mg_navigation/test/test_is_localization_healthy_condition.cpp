#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "localization_supervisor_msgs/msg/localization_status.hpp"
#include "mg_navigation/bt_plugins/is_localization_healthy_condition.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using localization_supervisor_msgs::msg::LocalizationStatus;

const char * const kTreeXml =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <IsLocalizationHealthy max_age="0.5"/>
    </BehaviorTree>
  </root>)";

class IsLocalizationHealthyTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  IsLocalizationHealthyTest()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_navigator_test");
    pub_ = node_->create_publisher<LocalizationStatus>("/localization/status", 10);
    factory_.registerNodeType<mg_navigation::bt_plugins::IsLocalizationHealthyCondition>(
      "IsLocalizationHealthy");
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node_);
    tree_ = factory_.createTreeFromText(kTreeXml, blackboard);
  }

  void publish(uint8_t state)
  {
    LocalizationStatus msg;
    msg.state = state;
    pub_->publish(msg);
  }

  // publish をくり返しながら tick し、expected が返るまで待つ
  bool tickUntil(BT::NodeStatus expected, const std::function<void()> & publish_fn)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      publish_fn();
      if (tree_.tickRoot() == expected) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<LocalizationStatus>::SharedPtr pub_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

}  // namespace

TEST_F(IsLocalizationHealthyTest, NoStatusIsNotHealthy)
{
  EXPECT_EQ(tree_.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(IsLocalizationHealthyTest, NormalIsHealthy)
{
  EXPECT_TRUE(
    tickUntil(BT::NodeStatus::SUCCESS, [this]() {publish(LocalizationStatus::NORMAL);}));
}

TEST_F(IsLocalizationHealthyTest, OtherStatesAreNotHealthy)
{
  ASSERT_TRUE(
    tickUntil(BT::NodeStatus::SUCCESS, [this]() {publish(LocalizationStatus::NORMAL);}));
  for (const auto state : {LocalizationStatus::SUSPECT, LocalizationStatus::ISOLATED,
      LocalizationStatus::RECOVERING, LocalizationStatus::DEGRADED})
  {
    EXPECT_TRUE(tickUntil(BT::NodeStatus::FAILURE, [this, state]() {publish(state);}));
  }
}

TEST_F(IsLocalizationHealthyTest, StaleStatusIsNotHealthy)
{
  ASSERT_TRUE(
    tickUntil(BT::NodeStatus::SUCCESS, [this]() {publish(LocalizationStatus::NORMAL);}));
  // 配信が止まって max_age を過ぎる
  EXPECT_TRUE(tickUntil(BT::NodeStatus::FAILURE, []() {}));
}
