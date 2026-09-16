#pragma once

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
      double rerender_threshold_m = 0.1);

  ScanProcessResult process_frame(const SensorFrame& frame);
  FinalizeResult finalize();

  std::optional<std::pair<double, double>> anchor_latlon() const;
  std::optional<std::pair<double, double>> anchor_utm() const;
  std::optional<double> init_rotation() const;
  std::vector<PoseNode> get_all_nodes() const;
  std::vector<PoseEdge> get_all_edges() const;

 private:
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

  int last_node_index_{-1};
  size_t last_loop_edge_count_{0};
  bool initialized_{false};
  std::string state_;
  double init_rotation_{0.0};
  std::unordered_map<int, std::tuple<double, double, double>> last_rendered_poses_;
  std::optional<double> last_gnss_timestamp_;
  bool first_render_done_{false};

  mutable std::mutex mutex_;

  void initialize_with_gnss_if_ready(const SensorFrame& frame);
  void initialize_optimizer_if_needed(const PoseNode& node);
  std::optional<PoseEdge> add_latest_seq_edge(const PoseNode& node);
  std::pair<std::vector<PoseEdge>, bool> add_new_loop_edges();
  std::optional<GnssPrior> add_gnss_prior(const SensorFrame& frame, const PoseNode& node);
  bool apply_optimized_poses(const PoseNode& node, bool loop_closed);
  std::optional<PoseEdge> get_latest_seq_edge(int node_index) const;
  double sigma_from_gnss(const GnssData& gnss) const;
};

using GraphOrchestratorPtr = std::shared_ptr<GraphOrchestrator>;

}  // namespace core
}  // namespace slam_gnss_2d
