#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/gnss/anchor_manager.hpp"
#include "slam_gnss_2d/optimizer/isam2_optimizer.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"

namespace slam_gnss_2d {
namespace core {

struct FinalizeResult {
  bool rerender_required{false};
};

// process_frame 内の処理ステージごとの累積所要時間 [s]
struct OrchestratorStageTimes {
  double add_scan_sec{0.0};        // キーフレーム判定+スキャンマッチング+near-link
  double gnss_init_sec{0.0};       // GNSS初期化・エッジ登録・リアンカー・GNSS prior
  double optimize_sec{0.0};        // iSAM2 update + calculateEstimate
  double apply_poses_sec{0.0};     // 最適化結果のポーズグラフ反映
};

class GraphOrchestrator {
 public:
  GraphOrchestrator(
      pose_graph::PoseGraphBuilderPtr pose_graph,
      bool use_gnss,
      double isam2_relinearize_threshold = 0.1,
      int anchor_min_fix_status = 0,
      double gnss_fix_sigma_m = 0.02,
      double gnss_float_sigma_m = 0.5,
      double gnss_factor_yaw_variance = 1e8,
      double gnss_init_distance_m = 2.0,
      double gnss_max_sigma_m = 5.0,
      double rerender_threshold_m = 0.1,
      double gnss_min_interval_m = 1.0,
      double anchor_sigma_m = 0.05,
      double anchor_init_yaw_sigma_rad = 10.0,
      double gnss_max_innovation_m = 0.0,
      const std::string& gnss_robust_kernel = "huber",
      double gnss_robust_kernel_scale = 1.345,
      int gnss_prior_min_fix_status = 1,
      bool dynamic_reanchor_enabled = false,
      int dynamic_reanchor_min_fix_status = 2,
      int dynamic_reanchor_min_samples = 10,
      double dynamic_reanchor_min_distance_m = 10.0,
      double dynamic_reanchor_max_residual_rms_m = 1.0,
      bool batch_on_finalize = true,
      int batch_max_iterations = 100,
      double gnss_lever_arm_x = 0.0,
      double gnss_lever_arm_y = 0.0);

  ScanProcessResult process_frame(const SensorFrame& frame);
  FinalizeResult finalize();

  std::optional<std::pair<double, double>> anchor_latlon() const;
  std::optional<std::pair<double, double>> anchor_utm() const;
  std::optional<double> init_rotation() const;
  std::vector<PoseNode> get_all_nodes() const;
  std::vector<PoseEdge> get_all_edges() const;
  const OrchestratorStageTimes& stage_times() const { return stage_times_; }
  void set_between_robust_kernel(const std::string& kernel, double scale) {
    optimizer_.set_between_robust_kernel(kernel, scale);
  }

 private:
  OrchestratorStageTimes stage_times_;
  pose_graph::PoseGraphBuilderPtr pose_graph_;
  bool use_gnss_;
  optimizer::ISAM2Optimizer optimizer_;
  std::unique_ptr<gnss::GnssAnchorManager> anchor_manager_;

  int anchor_min_fix_status_;
  double gnss_fix_sigma_m_;
  double gnss_float_sigma_m_;
  double gnss_factor_yaw_variance_;
  double gnss_init_distance_m_;
  double gnss_max_sigma_m_;
  double rerender_threshold_m_;
  double gnss_min_interval_m_;
  double anchor_sigma_m_;
  double anchor_init_yaw_sigma_rad_;
  double gnss_max_innovation_m_{0.0};
  std::string gnss_robust_kernel_{"huber"};
  double gnss_robust_kernel_scale_{1.345};

  int gnss_prior_min_fix_status_{1};
  bool dynamic_reanchor_enabled_{false};
  int dynamic_reanchor_min_fix_status_{2};
  int dynamic_reanchor_min_samples_{10};
  double dynamic_reanchor_min_distance_m_{10.0};
  double dynamic_reanchor_max_residual_rms_m_{1.0};
  bool dynamic_reanchored_{false};
  bool batch_on_finalize_{true};
  int batch_max_iterations_{100};
  double gnss_lever_arm_x_{0.0};
  double gnss_lever_arm_y_{0.0};

  int last_node_index_{-1};
  size_t last_loop_edge_count_{0};
  bool initialized_{false};
  std::string state_;
  double init_rotation_{0.0};
  std::unordered_map<int, std::tuple<double, double, double>> last_rendered_poses_;
  std::optional<double> last_gnss_timestamp_;
  std::optional<std::pair<double, double>> last_gnss_pos_;
  bool first_render_done_{false};
  int gnss_prior_count_{0};
  int gnss_rejected_sigma_count_{0};
  int gnss_rejected_interval_count_{0};
  int gnss_rejected_innovation_count_{0};
  int gnss_rejected_status_count_{0};

  struct ReanchorSample {
    Eigen::Vector2d slam_pos;
    Eigen::Vector2d utm_pos;
  };
  std::deque<ReanchorSample> reanchor_samples_;

  mutable std::mutex mutex_;

  std::vector<std::pair<double, double>> init_gnss_pts_;
  std::vector<std::pair<double, double>> init_odom_pts_;

  void initialize_with_gnss_if_ready(const SensorFrame& frame);
  void initialize_optimizer_if_needed(const PoseNode& node);
  std::optional<PoseEdge> add_latest_seq_edge(const PoseNode& node);
  std::optional<PoseEdge> add_latest_seq_edge(
      const PoseNode& node, const std::vector<PoseEdge>& all_edges);
  std::pair<std::vector<PoseEdge>, bool> add_new_loop_edges();
  std::pair<std::vector<PoseEdge>, bool> add_new_loop_edges(
      const std::vector<PoseEdge>& all_edges);
  std::optional<GnssPrior> add_gnss_prior(const SensorFrame& frame, const PoseNode& node);
  bool try_dynamic_reanchor(const PoseNode& node, const SensorFrame& frame);
  bool apply_optimized_poses(const PoseNode& node, bool loop_closed);
  std::optional<PoseEdge> get_latest_seq_edge(int node_index) const;
  std::optional<PoseEdge> get_latest_seq_edge(
      int node_index, const std::vector<PoseEdge>& all_edges) const;
  double sigma_from_gnss(const GnssData& gnss) const;
  static std::optional<std::pair<double, double>> estimate_heading_pca(
      const std::vector<std::pair<double, double>>& pts);
};

using GraphOrchestratorPtr = std::shared_ptr<GraphOrchestrator>;

}  // namespace core
}  // namespace slam_gnss_2d
