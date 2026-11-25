#pragma once

#include <cstdint>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "localization_supervisor_msgs/msg/localization_status.hpp"
#include "localization_supervisor_msgs/srv/request_verify.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mg_navigation
{
namespace bt_plugins
{

// 自己位置の監督に確かめの要求 (地図との照合を 1 回) を出し、結果を待つ BT 行動ノード。
// 計画用地図の障害物からの脱出など、走行側が自己位置を頼りに動く前に使う。
//
// SUCCESS は、照合で今の推定 (AMCL) をそのまま確かめたとき (脱出してよい)。
// FAILURE は、別の姿勢へ置き直した (計画し直す)、決まらない (監督が止めて照合する)、要求を受け付けられない
// (間隔の下限より早い)、timeout 秒以内に結果が来ないとき。
// 監督が確かめを扱えない (監督がいない、門がない構成) ときは、従来どおり、状態が NORMAL なら SUCCESS。
// 結果は、状態 (localization_supervisor_msgs/LocalizationStatus) の verify_id・verify_result で受け取る。
class VerifyLocalizationAction : public BT::StatefulActionNode
{
public:
  VerifyLocalizationAction(const std::string & name, const BT::NodeConfiguration & conf);

  VerifyLocalizationAction() = delete;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "service", "/localization_supervisor_node/request_verify", "Verify request service"),
      BT::InputPort<std::string>("topic", "/localization/status", "Status topic of the supervisor"),
      BT::InputPort<double>("timeout", 15.0, "Max time [s] to wait for the result"),
      BT::InputPort<std::string>("requester", "bt", "Requester name (for the record)"),
    };
  }

private:
  using RequestVerify = localization_supervisor_msgs::srv::RequestVerify;
  using LocalizationStatus = localization_supervisor_msgs::msg::LocalizationStatus;

  void statusCallback(LocalizationStatus::SharedPtr msg);
  BT::NodeStatus finish(BT::NodeStatus status, const std::string & why);
  // 確かめを扱えないときの、従来の判断 (直近の状態が NORMAL か)
  BT::NodeStatus byState(const std::string & why);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<RequestVerify>::SharedPtr client_;
  rclcpp::Subscription<LocalizationStatus>::SharedPtr sub_;
  rclcpp::Client<RequestVerify>::SharedFuture future_;
  bool waiting_response_{false};
  uint32_t id_{0};
  uint32_t last_verify_id_{0};
  uint8_t state_{0};
  double received_{-1.0};
  std::string last_verify_result_;
  double started_{0.0};
};

}  // namespace bt_plugins
}  // namespace mg_navigation
