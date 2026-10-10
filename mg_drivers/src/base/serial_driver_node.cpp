#include "mg_drivers/base/serial_driver_node.hpp"

#include <chrono>
#include <functional>

using mg_drivers::SerialDriverNode;
using mg_drivers::SerialError;
using mg_drivers::SerialErrorStrings;

SerialDriverNode::SerialDriverNode(
  const std::string & node_name, const std::string & param_prefix,
  const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
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
    RCLCPP_ERROR(this->get_logger(), "Serial error recovery start");
    device_->reset_serial();
  }
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
      RCLCPP_ERROR(this->get_logger(), "Serial device disappeared");
      device_->close_serial();
    }
  } else if (!device_->is_alive() || consecutive_errors_ >= error_recovery_count_) {
    device_->reset_serial();
    probe_connection();
  } else if (consecutive_errors_ > 0) {
    probe_connection();
  }

  const bool connected = is_connected();
  if (last_connected_ && *last_connected_ != connected) {
    if (connected) {
      RCLCPP_INFO(this->get_logger(), "Serial device connected");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial device disconnected");
    }
  }
  last_connected_ = connected;

  std_msgs::msg::Bool msg;
  msg.data = connected;
  connected_pub_->publish(msg);
}
