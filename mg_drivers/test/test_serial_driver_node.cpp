#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "mg_drivers/base/serial_device.hpp"
#include "mg_drivers/base/serial_driver_node.hpp"

using mg_drivers::SerialDevice;
using mg_drivers::SerialDriverNode;
using mg_drivers::SerialError;

namespace
{
// 実際のシリアルポートを使わず、状態を操作できる機器
class FakeDevice : public SerialDevice
{
public:
  bool alive = true;
  bool present = true;
  SerialError probe_result = SerialError::NO_ERROR;
  int resets = 0;
  int closes = 0;
  int probes = 0;

  bool is_alive() const override { return alive; }
  bool is_device_present() const override { return present; }
  void close_serial() override
  {
    closes++;
    alive = false;
  }
  void reset_serial() override
  {
    resets++;
    alive = present;
  }
  SerialError probe() override
  {
    probes++;
    return probe_result;
  }
};

class TestNode : public SerialDriverNode
{
public:
  TestNode() : SerialDriverNode("test_serial_driver_node", "dev", rclcpp::NodeOptions())
  {
    device = std::make_shared<FakeDevice>();
    set_device(device);
  }

  using SerialDriverNode::handle_serial_error;
  using SerialDriverNode::health_check;
  using SerialDriverNode::is_connected;

  std::shared_ptr<FakeDevice> device;
};

class SerialDriverNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { node = std::make_shared<TestNode>(); }

  void fail(int times)
  {
    for (int i = 0; i < times; i++) {
      node->handle_serial_error(SerialError::WRITE_ERROR);
    }
  }

  std::shared_ptr<TestNode> node;
};
}  // namespace

TEST_F(SerialDriverNodeTest, HealthyDeviceIsConnectedWithoutProbeOrReset)
{
  node->health_check();

  EXPECT_TRUE(node->is_connected());
  EXPECT_EQ(node->device->probes, 0);
  EXPECT_EQ(node->device->resets, 0);
}

TEST_F(SerialDriverNodeTest, SingleErrorIsProbedAndCleared)
{
  fail(1);
  EXPECT_FALSE(node->is_connected());

  node->health_check();

  EXPECT_TRUE(node->is_connected());
  EXPECT_EQ(node->device->probes, 1);
  EXPECT_EQ(node->device->resets, 0);
}

TEST_F(SerialDriverNodeTest, FailedProbeKeepsDisconnected)
{
  node->device->probe_result = SerialError::RECEIVE_SIZE_ERROR;
  fail(1);

  node->health_check();

  EXPECT_FALSE(node->is_connected());
  EXPECT_EQ(node->device->resets, 0);
}

TEST_F(SerialDriverNodeTest, ReachingThresholdResetsOnlyOnce)
{
  fail(3);
  EXPECT_EQ(node->device->resets, 0);

  fail(1);  // 既定の閾値 4
  EXPECT_EQ(node->device->resets, 1);

  fail(3);
  EXPECT_EQ(node->device->resets, 1);
}

TEST_F(SerialDriverNodeTest, HealthCheckRetriesAfterThresholdUntilProbeSucceeds)
{
  node->device->probe_result = SerialError::WRITE_ERROR;
  fail(4);
  ASSERT_EQ(node->device->resets, 1);

  node->health_check();
  EXPECT_EQ(node->device->resets, 2);
  EXPECT_FALSE(node->is_connected());

  node->device->probe_result = SerialError::NO_ERROR;
  node->health_check();
  EXPECT_EQ(node->device->resets, 3);
  EXPECT_TRUE(node->is_connected());

  node->health_check();
  EXPECT_EQ(node->device->resets, 3);
}

TEST_F(SerialDriverNodeTest, SuccessfulCommunicationClearsErrors)
{
  fail(3);
  node->handle_serial_error(SerialError::NO_ERROR);
  EXPECT_TRUE(node->is_connected());

  fail(3);  // 数え直し: 閾値に達しない
  EXPECT_EQ(node->device->resets, 0);
}

TEST_F(SerialDriverNodeTest, DeviceDisappearingClosesPortAndReconnectsWhenBack)
{
  node->device->present = false;
  node->health_check();

  EXPECT_FALSE(node->is_connected());
  EXPECT_EQ(node->device->closes, 1);
  EXPECT_EQ(node->device->resets, 0);

  // デバイスが無いあいだは、開き直さない (閉じるのも 1 回だけ)
  node->health_check();
  EXPECT_EQ(node->device->closes, 1);
  EXPECT_EQ(node->device->resets, 0);

  node->device->present = true;
  node->health_check();

  EXPECT_EQ(node->device->resets, 1);
  EXPECT_EQ(node->device->probes, 1);
  EXPECT_TRUE(node->is_connected());
}

TEST_F(SerialDriverNodeTest, PortNotOpenAtStartupIsOpenedWhenDeviceAppears)
{
  node->device->alive = false;
  node->device->present = false;
  node->health_check();
  EXPECT_FALSE(node->is_connected());
  EXPECT_EQ(node->device->resets, 0);

  node->device->present = true;
  node->health_check();
  EXPECT_TRUE(node->is_connected());
  EXPECT_EQ(node->device->resets, 1);
}

TEST_F(SerialDriverNodeTest, PublishesConnectedState)
{
  std::optional<bool> received;
  auto sub = node->create_subscription<std_msgs::msg::Bool>(
    "~/connected", rclcpp::QoS(1), [&received](const std_msgs::msg::Bool::SharedPtr msg) {
      received = msg->data;
    });

  auto wait_for = [&](bool expected) {
    received.reset();
    node->health_check();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline && !received) {
      rclcpp::spin_some(node);
    }
    return received && *received == expected;
  };

  EXPECT_TRUE(wait_for(true));
  node->device->present = false;
  EXPECT_TRUE(wait_for(false));
}
