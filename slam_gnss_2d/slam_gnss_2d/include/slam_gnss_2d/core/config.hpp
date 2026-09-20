#pragma once

#include <string>
#include <vector>

namespace slam_gnss_2d {
namespace core {

struct TopicsConfig {
  std::string scan{"/scan_top_lidar"};
  std::string odom{"/odom"};
};

struct MapConfig {
  double resolution{0.05};
  double expansion_margin{100.0};
  double publish_hz{1.0};
  std::string renderer{"overwrite"};  // "overwrite" | "counting"
  double hit_threshold{0.3};
  int min_hits{2};
  double hit_weight{1.0};
  double miss_weight{0.25};
  double miss_clearance_margin{0.0};
  double max_miss_ratio{2.0};
};

struct KeyframeConfig {
  double min_translation{1.0};
  double min_rotation{0.1};
};

struct IcpConfig {
  int max_iterations{100};
  double tolerance{1e-5};
  double max_correspondence_dist{1.0};
  std::string robust_kernel{"huber"};  // "none" | "huber" | "cauchy"
  double robust_kernel_scale{0.1};
  double motion_prior_weight_x{10.0};
  double motion_prior_weight_y{500.0};
  double motion_prior_weight_yaw{100.0};
  double tolerance_trans{-1.0};
  double tolerance_rot{-1.0};
};

struct NdtConfig {
  double cell_size{1.0};
  std::vector<double> cell_sizes{1.0};
  bool use_bilinear{false};
};

struct LocalMapConfig {
  int window{30};
  double radius{30.0};
};

struct CsmConfig {
  double linear_search_window{1.0};
  double angular_search_window{0.5};
  double linear_step{0.05};
  double angular_step{0.02};
};

struct MultiStartConfig {
  double angular_search_window_deg{20.0};
  double angular_step_deg{2.5};
  bool enable_straight_hypothesis{true};
  bool enable_const_vel_hypothesis{true};
};

struct MultiResCsmConfig {
  double linear_search_window{0.4};
  double angular_search_window_deg{15.0};
  double linear_step{0.02};
  double angular_step_deg{0.5};
  double grid_resolution{0.03};
  double score_threshold{0.1};
  bool enable_variance_penalty{true};
  double distance_variance_penalty{0.5};
  double angle_variance_penalty{1.0};
  double minimum_distance_penalty{0.5};
  double minimum_angle_penalty{0.9};
};

struct NearKeyframeLinkConfig {
  bool enabled{true};
  int buffer_size{10};
  double max_distance{2.0};
  int min_index_diff{2};
  int max_links_per_node{3};
  double max_translation_drift{0.4};
  double max_rotation_drift_deg{15.0};
  double min_eigenvalue{10.0};
};

struct ScanMatchingConfig {
  bool enabled{true};
  std::string type{"multi_res_csm"};            // "icp" | "ndt" | "csm" | "coarse_to_fine" | "multi_start_coarse_to_fine" | "multi_res_csm"
  std::string reference{"scan_to_local_map"};  // "scan_to_scan" | "scan_to_local_map"
  int max_failure_streak{5};
  double max_translation_drift{0.08};
  double yaw_information_multiplier{100.0};
  IcpConfig icp;
  NdtConfig ndt;
  CsmConfig csm;
  LocalMapConfig local_map;
  MultiStartConfig multi_start;
  MultiResCsmConfig multi_res_csm;
  NearKeyframeLinkConfig near_links;
};

struct LoopClosureConfig {
  bool enabled{true};
  double search_radius{2.0};
  int min_node_gap{50};
  int max_failure_streak{3};
  std::string matcher_type{"icp"};    // "icp" | "ndt" | "csm"
  double yaw_information_multiplier{100.0};
  IcpConfig icp;
  NdtConfig ndt;
  CsmConfig csm;
  double max_dyaw_deg{145.0};
  double crossing_reject_deg{45.0};
  double submap_radius{5.0};
  double max_score{0.0};
};

struct GnssTopicsConfig {
  std::string fix{"/gps/fix"};
  std::string navpvt{"/navpvt"};
};

struct GnssValidationConfig {
  double max_sigma_m{5.0};
};

struct GnssAnchorConfig {
  int min_fix_status{0};
  double sigma_m{0.05};
  double init_yaw_sigma_rad{10.0};
  double init_distance_m{2.0};
};

struct GnssSigmaConfig {
  double fix_m{0.02};
  double float_m{0.5};
  double factor_yaw_variance{1e8};
};

struct GnssDynamicReanchorConfig {
  bool enabled{false};
  int min_fix_status{2};       // 2: RTK Fixのみ
  int min_samples{10};         // アライメントに必要な最小サンプル数
  double min_distance_m{10.0}; // アライメントに必要な最小移動距離 [m]
  double max_residual_rms_m{1.0}; // アライメント採用判定の最大許容残差RMS [m]
};

struct GnssConfig {
  bool enabled{true};
  std::string source{"navpvt"};  // "navsat_fix" | "navpvt"
  GnssTopicsConfig topics;
  double navpvt_hacc_scale{1.0};
  double min_interval_m{1.0};
  double max_innovation_m{0.0};
  std::string robust_kernel{"huber"};  // "huber" | "cauchy"
  double robust_kernel_scale{1.345};
  int prior_min_fix_status{1};  // Prior採用最小測位ステータス (0: 全許可, 1: Float以上, 2: Fixのみ)
  GnssDynamicReanchorConfig dynamic_reanchor;
  GnssValidationConfig validation;
  GnssAnchorConfig anchor;
  GnssSigmaConfig sigma;
};

struct Isam2Config {
  double relinearize_threshold{0.1};
};

struct OptimizationConfig {
  std::string backend{"isam2"};  // "isam2" | "gtsam"
  Isam2Config isam2;
  double rerender_threshold_m{0.1};
  bool batch_on_finalize{true};
  int batch_max_iterations{100};
};

struct TrajectoryNoiseFilterConfig {
  bool enabled{false};
  std::string type{"clear"};  // "clear" | "attenuate"
  double radius_m{0.5};
};

struct SlamConfig {
  TopicsConfig topics;
  MapConfig map;
  KeyframeConfig keyframe;
  ScanMatchingConfig scan_matching;
  LoopClosureConfig loop_closure;
  GnssConfig gnss;
  OptimizationConfig optimization;
  TrajectoryNoiseFilterConfig trajectory_noise_filter;
  std::string save_dir{""};
};

}  // namespace core

// Alias in outer namespace
using core::TopicsConfig;
using core::MapConfig;
using core::KeyframeConfig;
using core::IcpConfig;
using core::NdtConfig;
using core::LocalMapConfig;
using core::CsmConfig;
using core::ScanMatchingConfig;
using core::LoopClosureConfig;
using core::GnssTopicsConfig;
using core::GnssValidationConfig;
using core::GnssAnchorConfig;
using core::GnssSigmaConfig;
using core::GnssConfig;
using core::Isam2Config;
using core::OptimizationConfig;
using core::TrajectoryNoiseFilterConfig;
using core::SlamConfig;

}  // namespace slam_gnss_2d
