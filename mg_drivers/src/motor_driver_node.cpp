#include "mg_drivers/motor_driver_node.hpp"

using mg_drivers::MotorDriverNode;

MotorDriverNode::MotorDriverNode(rclcpp::NodeOptions options) : Node("motor_driver_node", options)
{
  init_ros_params();
  prepare_ros_communications();

  motor_driver_ = std::make_shared<mg_drivers::MotorDriver>(device_name_);
}

void MotorDriverNode::init_ros_params()
{
  // Declare parameters
  this->declare_parameter<std::string>("motor_driver.device_name");
  this->declare_parameter<double>("motor_driver.wheel_pitch");
  this->declare_parameter<double>("motor_driver.max_speed");
  this->declare_parameter<int>("motor_driver.error_recovery_count");

  // Get parameters
  this->get_parameter_or<std::string>(
    "motor_driver.device_name", device_name_, std::string("/dev/ttyUSB0"));

  this->get_parameter_or<double>("motor_driver.wheel_pitch", wheel_pitch_, 0.358);

  this->get_parameter_or<double>("motor_driver.max_speed", max_speed_, 0.4);

  this->get_parameter_or<int>("motor_driver.error_recovery_count", error_recovery_count_, 4);
}

void MotorDriverNode::prepare_ros_communications()
{
  // Create subscriber
  this->twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1),
    std::bind(&MotorDriverNode::twist_sub_cb, this, std::placeholders::_1));

  this->emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/emergency_stop", rclcpp::QoS(1),
    std::bind(&MotorDriverNode::emergency_stop_sub_cb, this, std::placeholders::_1));

  this->connected_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/connected", rclcpp::QoS(1));

  this->health_timer_ =
    this->create_wall_timer(std::chrono::seconds(1), std::bind(&MotorDriverNode::health_check, this));
}

mg_drivers::SpeedParameter MotorDriverNode::create_speed_parameter(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  SpeedParameter req;

  // Convert twist to speed
  double linear_x = msg->linear.x;
  double angular_z = msg->angular.z;

  if (linear_x > max_speed_) {
    linear_x = max_speed_;
  } else if (linear_x < -max_speed_) {
    linear_x = -max_speed_;
  }

  if (abs(angular_z) > 0.8) {
    angular_z = 0.8 * angular_z / abs(angular_z);
  }

  double left_speed = linear_x - angular_z * wheel_pitch_ / 2.0;
  double right_speed = linear_x + angular_z * wheel_pitch_ / 2.0;

  // Set speed
  req.left_wheel_speed = static_cast<int16_t>(left_speed * 1000.0);    // [mm/s]
  req.right_wheel_speed = static_cast<int16_t>(right_speed * 1000.0);  // [mm/s]

  return req;
}

mg_drivers::MotorDriverResponse MotorDriverNode::send_speed_command(
  mg_drivers::SpeedParameter param)
{
  // TODO: print log about req
  MotorDriverResponse res = motor_driver_->send_speed_command(param);
  // TODO: print log about res

  if (res.error != SerialError::NO_ERROR && res.error != SerialError::CHECKSUM_ERROR) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Failed to send speed command to motor driver: %s",
      SerialErrorStrings[static_cast<int>(res.error)].c_str());
  }

  return res;
}

void MotorDriverNode::twist_sub_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (emergency_stop_) {
    RCLCPP_WARN(this->get_logger(), "Emergency stop is active. Ignoring twist command.");
    return;
  }

  SpeedParameter req = create_speed_parameter(msg);

  handle_response(send_speed_command(req));
}

void MotorDriverNode::handle_response(const MotorDriverResponse & res)
{
  if (res.error == SerialError::NO_ERROR) {
    consecutive_errors_ = 0;
    return;
  }
  if (res.error == SerialError::CHECKSUM_ERROR) {
    return;
  }

  consecutive_errors_++;
  // 閾値に達した 1 回だけ開き直す。それ以降は health_check が 1 Hz で再試行する
  if (consecutive_errors_ == error_recovery_count_) {
    RCLCPP_ERROR(this->get_logger(), "Motor driver error recovery start");
    motor_driver_->reset_serial();
  }
}

bool MotorDriverNode::is_connected() const
{
  return motor_driver_->is_alive() && motor_driver_->is_device_present() &&
         consecutive_errors_ < error_recovery_count_;
}

void MotorDriverNode::recover_connection()
{
  motor_driver_->reset_serial();
  if (!motor_driver_->is_alive()) {
    return;
  }

  // 再接続の確認を兼ねて、停止の指令を送る
  SpeedParameter stop;
  stop.left_wheel_speed = 0;
  stop.right_wheel_speed = 0;
  MotorDriverResponse res = send_speed_command(stop);
  handle_response(res);
  if (res.error == SerialError::NO_ERROR) {
    RCLCPP_INFO(this->get_logger(), "Motor driver reconnected");
  }
}

void MotorDriverNode::health_check()
{
  if (!motor_driver_->is_device_present()) {
    // デバイスが抜かれている。開いたままの fd は使えないので閉じ、戻ったら開き直す
    if (motor_driver_->is_alive()) {
      RCLCPP_ERROR(this->get_logger(), "Motor driver device disappeared");
      motor_driver_->close_serial();
    }
  } else if (!motor_driver_->is_alive() || consecutive_errors_ >= error_recovery_count_) {
    recover_connection();
  }

  std_msgs::msg::Bool msg;
  msg.data = is_connected();
  connected_pub_->publish(msg);
}

void MotorDriverNode::emergency_stop_sub_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_ = msg->data;

  if (emergency_stop_) {
    RCLCPP_WARN(this->get_logger(), "Emergency stop is active. Stopping the robot.");

    SpeedParameter req;
    req.left_wheel_speed = 0;
    req.right_wheel_speed = 0;

    motor_driver_->send_speed_command(req);
  } else {
    RCLCPP_INFO(this->get_logger(), "Emergency stop is inactive. Resuming the robot.");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // options.allow_undeclared_parameters(true);
  // options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MotorDriverNode>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}