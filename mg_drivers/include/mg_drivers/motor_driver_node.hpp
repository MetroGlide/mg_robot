// ROS2 node of publish wheel odometry
#pragma once

// ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++
#include <cmath>
#include <memory>
#include <string>

// mg_drivers
#include "mg_drivers/devices/motor_driver.hpp"

namespace mg_drivers
{
class MotorDriverNode : public rclcpp::Node
{
public:
  explicit MotorDriverNode(rclcpp::NodeOptions options);
  virtual ~MotorDriverNode() {};

private:
  void init_ros_params();
  void prepare_ros_communications();

  void twist_sub_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

  void emergency_stop_sub_cb(const std_msgs::msg::Bool::SharedPtr msg);

  // 送信結果を連続エラー数に反映し、閾値に達したらシリアルを開き直す
  void handle_response(const MotorDriverResponse & res);
  // 1 Hz: デバイスの有無と接続状態を確認し、切れていれば再接続して connected を配信する
  void health_check();
  void recover_connection();
  bool is_connected() const;

  std::shared_ptr<mg_drivers::MotorDriver> motor_driver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::string device_name_;

  std::string twist_frame_id_;
  std::string base_frame_id_;

  double wheel_pitch_;  // [m]
  double max_speed_;    // [m/s]

  bool emergency_stop_;

  int error_recovery_count_;
  int consecutive_errors_ = 0;

  SpeedParameter create_speed_parameter(const geometry_msgs::msg::Twist::SharedPtr msg);
  MotorDriverResponse send_speed_command(const SpeedParameter param);
};

}  // namespace mg_drivers
