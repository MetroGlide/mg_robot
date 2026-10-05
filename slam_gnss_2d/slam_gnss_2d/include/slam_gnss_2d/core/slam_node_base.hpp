#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "slam_gnss_2d/core/config.hpp"
#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/core/graph_orchestrator.hpp"
#include "slam_gnss_2d/core/sensor_synchronizer.hpp"
#include "slam_gnss_2d/input/base.hpp"
#include "slam_gnss_2d/map_manager/base.hpp"
#include "slam_gnss_2d/map_manager/trajectory_noise_filter.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"
#include "slam_gnss_2d/ros/map_save_service.hpp"
#include "slam_gnss_2d/ros/pose_graph_service.hpp"
#include "slam_gnss_2d/ros/slam_visualizer.hpp"
#include "slam_gnss_2d/ros/tf_broadcaster.hpp"

namespace slam_gnss_2d {
namespace core {

class SlamNodeBase : public rclcpp::Node {
 public:
  explicit SlamNodeBase(const std::string& node_name);
  ~SlamNodeBase() override;

  virtual void init();
  virtual void finalize();

 protected:
  virtual std::pair<std::shared_ptr<input::ScanSourceBase>, std::shared_ptr<input::OdomSourceBase>>
  setup_io(const SlamConfig& cfg) = 0;

  virtual std::shared_ptr<input::GnssSourceBase>
  setup_gnss_source(const SlamConfig& cfg) = 0;

  virtual void on_frame(const SensorFrame& frame);
  void publish_map_timer(bool force = false);
  void publish_tf_timer();

  SlamConfig config_;
  std::shared_ptr<input::ScanSourceBase> scan_source_;
  std::shared_ptr<input::OdomSourceBase> odom_source_;
  std::shared_ptr<input::GnssSourceBase> gnss_source_;
  std::shared_ptr<pose_graph::PoseGraphBuilderBase> pose_graph_;
  std::shared_ptr<map_manager::MapRendererBase> renderer_;
  std::shared_ptr<GraphOrchestrator> orchestrator_;
  std::shared_ptr<SensorSynchronizer> synchronizer_;
  std::shared_ptr<ros::SlamVisualizer> visualizer_;
  std::shared_ptr<ros::SlamTfBroadcaster> tf_broadcaster_;
  std::shared_ptr<ros::MapSaveService> save_service_;
  std::shared_ptr<ros::PoseGraphService> pose_graph_service_;

  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  bool map_dirty_{false};
  int node_count_{0};
  std::chrono::steady_clock::time_point last_stat_time_;
  std::chrono::steady_clock::time_point last_map_publish_time_{std::chrono::steady_clock::time_point::min()};
  bool finalized_{false};
  bool skip_intermediate_rendering_{false};

  // 処理ステージごとの累積所要時間 [s] (finalize 時にログ出力)
  struct StageTimes {
    double process_frame_sec{0.0};
    double render_sec{0.0};
    double publish_sec{0.0};
  };
  StageTimes stage_times_;
};

}  // namespace core
}  // namespace slam_gnss_2d
