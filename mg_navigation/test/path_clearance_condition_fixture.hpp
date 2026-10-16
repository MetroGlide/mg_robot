#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace mg_navigation
{
namespace test
{

// PathClearanceMonitor を使う BT 条件ノードのテストの土台。
// odom 原点のロボット、x 方向にまっすぐな経路、controller_server と local_costmap の
// パラメータサービス、costmap_raw の配信を用意する
class PathClearanceConditionTest : public ::testing::Test
{
protected:
  static constexpr double kResolution = 0.05;
  static constexpr unsigned int kSize = 200;
  static constexpr double kOrigin = -5.0;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  PathClearanceConditionTest()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_navigator_test");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // bt_navigator の TransformListener と同じく、タイムアウト付きの lookup を許す
    tf_->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    tf_->setTransform(transform, "test", true);

    costmap_pub_ = node_->create_publisher<nav2_msgs::msg::Costmap>(
      "local_costmap/costmap_raw", rclcpp::SystemDefaultsQoS());
  }

  ~PathClearanceConditionTest() override
  {
    if (spin_thread_.joinable()) {
      executor_.cancel();
      spin_thread_.join();
    }
  }

  // 登録済みの factory_ から木を作る
  void createTree(const std::string & xml)
  {
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node_);
    blackboard->set("tf_buffer", tf_);
    blackboard->set("path", straightPath());
    tree_ = factory_.createTreeFromText(xml, blackboard);
  }

  static nav_msgs::msg::Path straightPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    for (double x = 0.0; x <= 5.0; x += 0.1) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.pose.position.x = x;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  // controller_server と local_costmap のパラメータサービスを用意する
  void startParameterServers(bool use_collision_detection)
  {
    controller_ = std::make_shared<rclcpp::Node>("controller_server");
    controller_->declare_parameter("FollowPath.use_velocity_scaled_lookahead_dist", false);
    controller_->declare_parameter("FollowPath.lookahead_dist", 1.0);
    controller_->declare_parameter("FollowPath.min_lookahead_dist", 0.5);
    controller_->declare_parameter("FollowPath.max_lookahead_dist", 1.5);
    controller_->declare_parameter("FollowPath.use_rotate_to_heading", false);
    controller_->declare_parameter("FollowPath.rotate_to_heading_min_angle", 0.785);
    controller_->declare_parameter("FollowPath.max_robot_pose_search_dist", 4.0);
    controller_->declare_parameter("FollowPath.use_collision_detection", use_collision_detection);

    costmap_ = std::make_shared<rclcpp::Node>("local_costmap", "local_costmap");
    costmap_->declare_parameter(
      "footprint", std::string("[[0.5, 0.4], [0.5, -0.4], [-0.3, -0.4], [-0.3, 0.4]]"));
    costmap_->declare_parameter("footprint_padding", 0.0);
    costmap_->declare_parameter("robot_radius", 0.3);
    costmap_->declare_parameter("track_unknown_space", false);
    costmap_->declare_parameter("robot_base_frame", std::string("base_link"));

    executor_.add_node(controller_);
    executor_.add_node(costmap_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  // 経路上の x に障害物を置いたコストマップ (x が負なら障害物なし) を配信する
  void publishCostmap(double obstacle_x)
  {
    nav2_msgs::msg::Costmap msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = node_->now();
    msg.metadata.resolution = kResolution;
    msg.metadata.size_x = kSize;
    msg.metadata.size_y = kSize;
    msg.metadata.origin.position.x = kOrigin;
    msg.metadata.origin.position.y = kOrigin;
    msg.data.assign(kSize * kSize, nav2_costmap_2d::FREE_SPACE);
    if (obstacle_x >= 0.0) {
      const int mx = static_cast<int>((obstacle_x - kOrigin) / kResolution);
      const int my = static_cast<int>((0.0 - kOrigin) / kResolution);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          msg.data[(my + dy) * kSize + (mx + dx)] = nav2_costmap_2d::LETHAL_OBSTACLE;
        }
      }
    }
    costmap_pub_->publish(msg);
  }

  // publish (コストマップの配信) をくり返しながら tick し、expected が返るまで待つ
  bool tickUntil(
    BT::NodeStatus expected, const std::function<void()> & publish,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      publish();
      if (tree_.tickRoot() == expected) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  // duration の間 tick し続け、ずっと expected だったら true
  bool tickFor(
    BT::NodeStatus expected, const std::function<void()> & publish,
    std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      publish();
      if (tree_.tickRoot() != expected) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;

  rclcpp::Node::SharedPtr controller_;
  rclcpp::Node::SharedPtr costmap_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

}  // namespace test
}  // namespace mg_navigation
