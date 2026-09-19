#include "slam_gnss_2d/core/slam_node_base.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "slam_gnss_2d/core/component_factory.hpp"
#include "slam_gnss_2d/core/config_loader.hpp"

namespace slam_gnss_2d {
namespace core {

SlamNodeBase::SlamNodeBase(const std::string& node_name)
    : rclcpp::Node(node_name),
      last_stat_time_(std::chrono::steady_clock::now()) {
  ConfigLoader::declare_params(*this);
}

SlamNodeBase::~SlamNodeBase() {
  if (synchronizer_) {
    synchronizer_->stop();
  }
}

void SlamNodeBase::init() {
  config_ = ConfigLoader::build_config(*this);

  auto [scan_src, odom_src] = setup_io(config_);
  scan_source_ = scan_src;
  odom_source_ = odom_src;

  pose_graph_ = build_pose_graph_builder(config_);
  renderer_ = build_renderer(config_);

  if (config_.gnss.enabled) {
    gnss_source_ = setup_gnss_source(config_);
  }

  synchronizer_ = std::make_shared<SensorSynchronizer>(
      scan_source_, odom_source_, gnss_source_);
  synchronizer_->set_frame_callback([this](const SensorFrame& frame) {
    this->on_frame(frame);
  });
  synchronizer_->start();

  orchestrator_ = std::make_shared<GraphOrchestrator>(
      pose_graph_,
      config_.gnss.enabled,
      config_.optimization.isam2.relinearize_threshold,
      config_.gnss.anchor.min_fix_status,
      config_.gnss.sigma.fix_m,
      config_.gnss.sigma.float_m,
      config_.gnss.sigma.factor_yaw_variance,
      config_.gnss.anchor.init_distance_m,
      config_.gnss.validation.max_sigma_m,
      config_.optimization.rerender_threshold_m,
      config_.gnss.min_interval_m,
      config_.gnss.anchor.sigma_m,
      config_.gnss.anchor.init_yaw_sigma_rad,
      config_.gnss.max_innovation_m,
      config_.gnss.robust_kernel,
      config_.gnss.robust_kernel_scale,
      config_.gnss.prior_min_fix_status,
      config_.gnss.dynamic_reanchor.enabled,
      config_.gnss.dynamic_reanchor.min_fix_status,
      config_.gnss.dynamic_reanchor.min_samples,
      config_.gnss.dynamic_reanchor.min_distance_m);

  auto node_shared = shared_from_this();
  visualizer_ = std::make_shared<ros::SlamVisualizer>(node_shared, config_.gnss.enabled);
  tf_broadcaster_ = std::make_shared<ros::SlamTfBroadcaster>(node_shared);
  save_service_ = std::make_shared<ros::MapSaveService>(node_shared, pose_graph_, orchestrator_);
  pose_graph_service_ = std::make_shared<ros::PoseGraphService>(node_shared, pose_graph_, orchestrator_);

  double map_period_sec = 1.0 / std::max(config_.map.publish_hz, 0.1);
  map_timer_ = create_wall_timer(
      std::chrono::duration<double>(map_period_sec),
      [this]() { this->publish_map_timer(); });

  tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { this->publish_tf_timer(); });

  RCLCPP_INFO(
      get_logger(),
      "%s started (scan_matching=%d, loop_closure=%d, matcher=%s, ref=%s)\n"
      "  scan: %s, odom: %s\n"
      "  map: dynamic @ %.2fm/px, margin=%.1fm",
      get_name(), config_.scan_matching.enabled, config_.loop_closure.enabled,
      config_.scan_matching.type.c_str(), config_.scan_matching.reference.c_str(),
      config_.topics.scan.c_str(), config_.topics.odom.c_str(),
      config_.map.resolution, config_.map.expansion_margin);
}

void SlamNodeBase::on_frame(const SensorFrame& frame) {
  auto result = orchestrator_->process_frame(frame);
  const auto& node = result.node;
  if (!node.has_value()) {
    RCLCPP_DEBUG(
        get_logger(),
        "Scan rejected: below threshold at (%.2f, %.2f)",
        frame.odom.x, frame.odom.y);
    return;
  }

  tf_broadcaster_->update(*node, frame.odom);
  node_count_++;
  if (node_count_ == 1 || node_count_ % 10 == 0) {
    RCLCPP_DEBUG(
        get_logger(),
        "Node #%d: x=%.2f y=%.2f yaw=%.1fdeg",
        node->index, node->x, node->y, node->yaw * 180.0 / M_PI);
  }

  if (result.loop_closed || result.rerender_required) {
    if (result.loop_closed) {
      visualizer_->publish_path_before_optimize();
    }
    auto nodes = pose_graph_->get_nodes();
    renderer_->rerender_all(nodes);
    visualizer_->rebuild_path(nodes);
    if (result.loop_closed) {
      RCLCPP_INFO(
          get_logger(),
          "Loop closed at node #%d: full rerender triggered",
          node->index);
    }
  } else {
    if (!renderer_->add_node(*node)) {
      renderer_->rerender_all(pose_graph_->get_nodes());
    }
    visualizer_->publish_path_increment(*node);
  }

  visualizer_->publish_pose_graph_markers(*pose_graph_);

  std::vector<PoseNode> new_nodes = {*node};
  std::vector<PoseEdge> new_seq_edges;
  if (result.new_seq_edge.has_value()) {
    new_seq_edges.push_back(*result.new_seq_edge);
  }
  std::vector<GnssPrior> new_priors;
  if (result.new_gnss_prior.has_value()) {
    new_priors.push_back(*result.new_gnss_prior);
  }

  visualizer_->publish_pose_graph_diff(
      new_nodes,
      new_seq_edges,
      new_priors,
      result.new_loop_edges,
      result.loop_closed,
      result.rerender_required);

  map_dirty_ = true;
}

void SlamNodeBase::publish_map_timer() {
  visualizer_->publish_anchor(*orchestrator_);

  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_stat_time_).count() >= 30.0) {
    auto stats = synchronizer_->get_stats();
    RCLCPP_DEBUG(
        get_logger(),
        "[stat] nodes=%d, scans=%d, odom_miss=%d",
        node_count_, stats.scans, stats.odom_miss);
    last_stat_time_ = now;
  }

  if (!map_dirty_) {
    return;
  }
  map_dirty_ = false;

  visualizer_->publish_map(*renderer_);
}

void SlamNodeBase::publish_tf_timer() {
  tf_broadcaster_->publish();
}

void SlamNodeBase::finalize() {
  if (finalized_) {
    return;
  }
  finalized_ = true;
  if (!orchestrator_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Finalizing SLAM node...");

  auto finalize_result = orchestrator_->finalize();

  if (finalize_result.rerender_required) {
    auto nodes = pose_graph_->get_nodes();
    renderer_->rerender_all(nodes);

    if (config_.trajectory_noise_filter.enabled) {
      map_manager::TrajectoryNoiseFilter noise_filter(config_.trajectory_noise_filter);
      noise_filter.apply(*renderer_, nodes);
    }

    visualizer_->rebuild_path(nodes);
    map_dirty_ = true;
    publish_map_timer();
    RCLCPP_INFO(get_logger(), "Final map optimization and rendering complete.");
  } else {
    map_dirty_ = true;
    publish_map_timer();
    RCLCPP_INFO(get_logger(), "Final map published.");
  }
}

}  // namespace core
}  // namespace slam_gnss_2d
