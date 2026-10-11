#include "mg_drivers/base/serial_driver_node.hpp"

#include <chrono>
#include <functional>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using mg_drivers::SerialDriverNode;
using mg_drivers::SerialError;
using mg_drivers::SerialErrorStrings;

SerialDriverNode::SerialDriverNode(
  const std::string & node_name, const std::string & param_prefix,
  const rclcpp::NodeOptions & options)
: Node(node_name, options),
  device_label_(param_prefix),
  updater_(this),
  events_(this->get_logger())
{
  updater_.setHardwareID(device_label_);
  updater_.add("connection", this, &SerialDriverNode::produce_connection_diagnostics);

  const std::string param_name = param_prefix + ".error_recovery_count";
  // 既定値つきで宣言する (launch が値を渡さなくても、起動できるように)
  error_recovery_count_ = this->declare_parameter<int>(param_name, 4);

  connected_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/connected", rclcpp::QoS(1));
  health_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&SerialDriverNode::health_check, this));
}

void SerialDriverNode::set_device(std::shared_ptr<SerialDevice> device) { device_ = device; }

void SerialDriverNode::handle_serial_error(SerialError error)
{
  if (error == SerialError::NO_ERROR) {
    consecutive_errors_ = 0;
    return;
  }

  consecutive_errors_++;
  // 閾値に達した 1 回だけ開き直す。それ以降は health_check が 1 Hz で再試行する
  if (consecutive_errors_ == error_recovery_count_) {
    reset_device("consecutive errors");
  }
}

void SerialDriverNode::reset_device(const std::string & reason)
{
  reset_attempts_++;
  if (reset_attempts_ == 1 || reset_attempts_ % 10 == 0) {
    events_.Warn(
      "driver.reconnect_attempt",
      {{"device", device_label_}, {"reason", reason}, {"count", reset_attempts_}});
  }
  device_->reset_serial();
}

std::string SerialDriverNode::disconnect_reason() const
{
  if (!device_) {
    return "device not set";
  }
  if (!device_->is_device_present()) {
    return "device missing";
  }
  if (!device_->is_alive()) {
    return "port closed";
  }
  if (consecutive_errors_ > 0) {
    return "communication errors";
  }
  return "";
}

void SerialDriverNode::produce_connection_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!device_) {
    stat.summary(DiagnosticStatus::WARN, "device not set");
    return;
  }
  if (is_connected()) {
    stat.summary(DiagnosticStatus::OK, "connected");
  } else {
    stat.summary(DiagnosticStatus::ERROR, disconnect_reason());
  }
  stat.add("consecutive_errors", consecutive_errors_);
  stat.add("device_present", device_->is_device_present());
  stat.add("reconnect_attempts", reset_attempts_);
  stat.add("disconnect_count", disconnect_count_);
}

bool SerialDriverNode::is_connected() const
{
  return device_->is_alive() && device_->is_device_present() && consecutive_errors_ == 0;
}

void SerialDriverNode::probe_connection()
{
  if (!device_->is_alive()) {
    return;
  }

  const SerialError error = device_->probe();
  if (error != SerialError::NO_ERROR) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Serial probe failed: %s",
      SerialErrorStrings[static_cast<int>(error)].c_str());
  }
  handle_serial_error(error);
}

void SerialDriverNode::health_check()
{
  if (!device_) {
    return;
  }

  if (!device_->is_device_present()) {
    // デバイスが抜かれている。開いたままの fd は使えないので閉じ、戻ったら開き直す
    if (device_->is_alive()) {
      device_->close_serial();
    }
  } else if (!device_->is_alive() || consecutive_errors_ >= error_recovery_count_) {
    reset_device(device_->is_alive() ? "consecutive errors" : "port closed");
    probe_connection();
  } else if (consecutive_errors_ > 0) {
    probe_connection();
  }

  const bool connected = is_connected();
  if (!last_connected_ || *last_connected_ != connected) {
    if (connected) {
      events_.Info(
        "driver.connected", {{"device", device_label_}, {"after_attempts", reset_attempts_}});
      reset_attempts_ = 0;
    } else {
      disconnect_count_++;
      events_.Error(
        "driver.disconnected", {{"device", device_label_}, {"reason", disconnect_reason()}});
    }
  }
  last_connected_ = connected;

  std_msgs::msg::Bool msg;
  msg.data = connected;
  connected_pub_->publish(msg);
}
