#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "localization_supervisor_msgs/msg/localization_status.hpp"
#include "localization_supervisor_msgs/srv/request_verify.hpp"
#include "mg_navigation/bt_plugins/verify_localization_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using localization_supervisor_msgs::msg::LocalizationStatus;
using localization_supervisor_msgs::srv::RequestVerify;

const char * const kTreeXml =
  R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <VerifyLocalization service="/test_supervisor/request_verify" timeout="2.0"/>
    </BehaviorTree>
  </root>)";

// 確かめの要求を受けて番号を返し、決めた結果を状態に出す偽の監督
class FakeSupervisor
{
public:
  explicit FakeSupervisor(const rclcpp::Node::SharedPtr & node)
  {
    pub_ = node->create_publisher<LocalizationStatus>("/localization/status", 10);
    srv_ = node->create_service<RequestVerify>(
      "/test_supervisor/request_verify",
      [this](const std::shared_ptr<RequestVerify::Request>,
      std::shared_ptr<RequestVerify::Response> response) {
        response->supported = supported;
        response->accepted = supported && accept;
        response->id = response->accepted ? ++last_id : 0;
      });
  }

  void publish()
  {
    LocalizationStatus msg;
    msg.state = state;
    msg.verify_id = last_id;
    msg.verify_result = result;
    pub_->publish(msg);
  }

  bool supported{true};
  bool accept{true};
  uint8_t state{LocalizationStatus::NORMAL};
  std::string result{"confirmed"};
  uint32_t last_id{0};

private:
  rclcpp::Publisher<LocalizationStatus>::SharedPtr pub_;
  rclcpp::Service<RequestVerify>::SharedPtr srv_;
};

class VerifyLocalizationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  VerifyLocalizationTest()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_navigator_test");
    executor_.add_node(node_);
    factory_.registerNodeType<mg_navigation::bt_plugins::VerifyLocalizationAction>(
      "VerifyLocalization");
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node_);
    tree_ = factory_.createTreeFromText(kTreeXml, blackboard);
  }

  // 偽の監督を回しながら tick し、RUNNING 以外が返るまで待つ
  BT::NodeStatus run(FakeSupervisor * supervisor)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (supervisor != nullptr) {
        supervisor->publish();
      }
      const auto status = tree_.tickRoot();
      if (status != BT::NodeStatus::RUNNING) {
        return status;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return BT::NodeStatus::RUNNING;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

// 監督の窓口が見つかるまで待つ (サービスの発見は非同期)
void waitForService(const rclcpp::Node::SharedPtr & node)
{
  auto client = node->create_client<RequestVerify>("/test_supervisor/request_verify");
  client->wait_for_service(std::chrono::seconds(2));
}

}  // namespace

TEST_F(VerifyLocalizationTest, NoSupervisorFails)
{
  EXPECT_EQ(run(nullptr), BT::NodeStatus::FAILURE);
}

TEST_F(VerifyLocalizationTest, ConfirmedSucceeds)
{
  FakeSupervisor supervisor(node_);
  waitForService(node_);
  EXPECT_EQ(run(&supervisor), BT::NodeStatus::SUCCESS);
}

TEST_F(VerifyLocalizationTest, RelocatedOrUndecidedFails)
{
  FakeSupervisor supervisor(node_);
  waitForService(node_);
  for (const std::string result : {"relocated", "undecided"}) {
    supervisor.result = result;
    EXPECT_EQ(run(&supervisor), BT::NodeStatus::FAILURE) << result;
  }
}

TEST_F(VerifyLocalizationTest, WithoutGateFallsBackToTheState)
{
  FakeSupervisor supervisor(node_);
  supervisor.supported = false;
  waitForService(node_);
  EXPECT_EQ(run(&supervisor), BT::NodeStatus::SUCCESS);
  supervisor.state = LocalizationStatus::SUSPECT;
  EXPECT_EQ(run(&supervisor), BT::NodeStatus::FAILURE);
}

TEST_F(VerifyLocalizationTest, NotAcceptedFails)
{
  FakeSupervisor supervisor(node_);
  supervisor.accept = false;
  waitForService(node_);
  EXPECT_EQ(run(&supervisor), BT::NodeStatus::FAILURE);
}
