#include "mg_navigation/bt_plugins/verify_localization_action.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace mg_navigation
{
namespace bt_plugins
{

VerifyLocalizationAction::VerifyLocalizationAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string service = "/localization_supervisor_node/request_verify";
  getInput("service", service);
  client_ = node_->create_client<RequestVerify>(
    service, rmw_qos_profile_services_default, callback_group_);

  std::string topic = "/localization/status";
  getInput("topic", topic);
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  sub_ = node_->create_subscription<LocalizationStatus>(
    topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&VerifyLocalizationAction::statusCallback, this, std::placeholders::_1),
    sub_option);
}

void VerifyLocalizationAction::statusCallback(LocalizationStatus::SharedPtr msg)
{
  last_verify_id_ = msg->verify_id;
  last_verify_result_ = msg->verify_result;
  state_ = msg->state;
  received_ = node_->now().seconds();
}

BT::NodeStatus VerifyLocalizationAction::onStart()
{
  callback_group_executor_.spin_some();
  if (!client_->service_is_ready()) {
    return byState("no supervisor");
  }
  auto request = std::make_shared<RequestVerify::Request>();
  getInput("requester", request->requester);
  future_ = client_->async_send_request(request).future.share();
  waiting_response_ = true;
  id_ = 0;
  started_ = node_->now().seconds();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VerifyLocalizationAction::onRunning()
{
  callback_group_executor_.spin_some();
  double timeout = 15.0;
  getInput("timeout", timeout);
  if (waiting_response_) {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (node_->now().seconds() - started_ > timeout) {
        return finish(BT::NodeStatus::FAILURE, "no response");
      }
      return BT::NodeStatus::RUNNING;
    }
    waiting_response_ = false;
    const auto response = future_.get();
    if (!response->supported) {
      return byState("the supervisor has no gate");
    }
    if (!response->accepted) {
      return finish(BT::NodeStatus::FAILURE, "not accepted: " + response->message);
    }
    id_ = response->id;
  }
  if (last_verify_id_ == id_) {
    if (last_verify_result_ == "confirmed") {
      return finish(BT::NodeStatus::SUCCESS, "confirmed");
    }
    return finish(BT::NodeStatus::FAILURE, last_verify_result_);
  }
  if (node_->now().seconds() - started_ > timeout) {
    return finish(BT::NodeStatus::FAILURE, "no result");
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VerifyLocalizationAction::byState(const std::string & why)
{
  const bool fresh = received_ >= 0.0 && node_->now().seconds() - received_ <= 3.0;
  const bool normal = fresh && state_ == LocalizationStatus::NORMAL;
  return finish(
    normal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE,
    why + (normal ? ", state is NORMAL" : ", state is not NORMAL"));
}

void VerifyLocalizationAction::onHalted()
{
  waiting_response_ = false;
  id_ = 0;
}

BT::NodeStatus VerifyLocalizationAction::finish(BT::NodeStatus status, const std::string & why)
{
  RCLCPP_INFO(
    node_->get_logger(), "VerifyLocalization: %s (%s)",
    status == BT::NodeStatus::SUCCESS ? "verified" : "not verified", why.c_str());
  waiting_response_ = false;
  return status;
}

}  // namespace bt_plugins
}  // namespace mg_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mg_navigation::bt_plugins::VerifyLocalizationAction>(
    "VerifyLocalization");
}
