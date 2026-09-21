#pragma once

#include <memory>
#include "slam_gnss_2d/core/config.hpp"
#include "slam_gnss_2d/map_manager/base.hpp"
#include "slam_gnss_2d/map_manager/counting_renderer.hpp"
#include "slam_gnss_2d/map_manager/overwrite_renderer.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"
#include "slam_gnss_2d/pose_graph/loop_closure_builder.hpp"
#include "slam_gnss_2d/pose_graph/odom_builder.hpp"
#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"
#include "slam_gnss_2d/scan_matching/base.hpp"
#include "slam_gnss_2d/scan_matching/coarse_to_fine_matcher.hpp"
#include "slam_gnss_2d/scan_matching/multi_res_csm_matcher.hpp"
#include "slam_gnss_2d/scan_matching/csm_matcher.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"
#include "slam_gnss_2d/scan_matching/multi_start_coarse_to_fine_matcher.hpp"
#include "slam_gnss_2d/scan_matching/multi_start_icp_matcher.hpp"
#include "slam_gnss_2d/scan_matching/ndt_matcher.hpp"
#include "slam_gnss_2d/scan_matching/reference_provider/base.hpp"
#include "slam_gnss_2d/scan_matching/reference_provider/local_map.hpp"
#include "slam_gnss_2d/scan_matching/reference_provider/scan_to_scan.hpp"

namespace slam_gnss_2d {
namespace core {

inline scan_matching::ScanMatcherPtr build_matcher(const SlamConfig& config) {
  if (config.scan_matching.type == "multi_res_csm") {
    auto fine = std::make_shared<scan_matching::ICPMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.icp.max_correspondence_dist,
        config.scan_matching.icp.robust_kernel,
        config.scan_matching.icp.robust_kernel_scale,
        config.scan_matching.yaw_information_multiplier,
        config.scan_matching.icp.motion_prior_weight_x,
        config.scan_matching.icp.motion_prior_weight_y,
        0.0,
        config.scan_matching.icp.tolerance_trans,
        config.scan_matching.icp.tolerance_rot);
    return std::make_shared<scan_matching::MultiResCSMMatcher>(
        fine,
        config.scan_matching.multi_res_csm.linear_search_window,
        config.scan_matching.multi_res_csm.angular_search_window_deg,
        config.scan_matching.multi_res_csm.linear_step,
        config.scan_matching.multi_res_csm.angular_step_deg,
        config.scan_matching.multi_res_csm.grid_resolution,
        config.scan_matching.multi_res_csm.score_threshold,
        config.scan_matching.multi_res_csm.enable_variance_penalty,
        config.scan_matching.multi_res_csm.distance_variance_penalty,
        config.scan_matching.multi_res_csm.angle_variance_penalty,
        config.scan_matching.multi_res_csm.minimum_distance_penalty,
        config.scan_matching.multi_res_csm.minimum_angle_penalty,
        config.scan_matching.num_threads);
  } else if (config.scan_matching.type == "multi_start_coarse_to_fine") {
    auto coarse = std::make_shared<scan_matching::NDTMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.ndt.cell_sizes,
        config.scan_matching.ndt.use_bilinear,
        config.scan_matching.yaw_information_multiplier);
    auto fine = std::make_shared<scan_matching::ICPMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.icp.max_correspondence_dist,
        config.scan_matching.icp.robust_kernel,
        config.scan_matching.icp.robust_kernel_scale,
        config.scan_matching.yaw_information_multiplier,
        config.scan_matching.icp.motion_prior_weight_x,
        config.scan_matching.icp.motion_prior_weight_y,
        config.scan_matching.icp.motion_prior_weight_yaw,
        config.scan_matching.icp.tolerance_trans,
        config.scan_matching.icp.tolerance_rot);
    return std::make_shared<scan_matching::MultiStartCoarseToFineMatcher>(
        coarse, fine,
        config.scan_matching.multi_start.angular_search_window_deg,
        config.scan_matching.multi_start.angular_step_deg,
        config.scan_matching.multi_start.enable_straight_hypothesis,
        config.scan_matching.multi_start.enable_const_vel_hypothesis);
  } else if (config.scan_matching.type == "multi_start_icp") {
    return std::make_shared<scan_matching::MultiStartICPMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.icp.max_correspondence_dist,
        config.scan_matching.icp.robust_kernel,
        config.scan_matching.icp.robust_kernel_scale,
        config.scan_matching.yaw_information_multiplier);
  } else if (config.scan_matching.type == "coarse_to_fine") {
    auto coarse = std::make_shared<scan_matching::NDTMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.ndt.cell_sizes,
        config.scan_matching.ndt.use_bilinear,
        config.scan_matching.yaw_information_multiplier);
    auto fine = std::make_shared<scan_matching::ICPMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.icp.max_correspondence_dist,
        config.scan_matching.icp.robust_kernel,
        config.scan_matching.icp.robust_kernel_scale,
        config.scan_matching.yaw_information_multiplier,
        config.scan_matching.icp.motion_prior_weight_x,
        config.scan_matching.icp.motion_prior_weight_y,
        config.scan_matching.icp.motion_prior_weight_yaw,
        config.scan_matching.icp.tolerance_trans,
        config.scan_matching.icp.tolerance_rot);
    return std::make_shared<scan_matching::CoarseToFineMatcher>(coarse, fine);
  } else if (config.scan_matching.type == "icp") {
    return std::make_shared<scan_matching::ICPMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.icp.max_correspondence_dist,
        config.scan_matching.icp.robust_kernel,
        config.scan_matching.icp.robust_kernel_scale,
        config.scan_matching.yaw_information_multiplier,
        config.scan_matching.icp.motion_prior_weight_x,
        config.scan_matching.icp.motion_prior_weight_y,
        config.scan_matching.icp.motion_prior_weight_yaw,
        config.scan_matching.icp.tolerance_trans,
        config.scan_matching.icp.tolerance_rot);
  } else if (config.scan_matching.type == "ndt") {
    return std::make_shared<scan_matching::NDTMatcher>(
        config.scan_matching.icp.max_iterations,
        config.scan_matching.icp.tolerance,
        config.scan_matching.ndt.cell_sizes,
        config.scan_matching.ndt.use_bilinear,
        config.scan_matching.yaw_information_multiplier);
  } else if (config.scan_matching.type == "csm") {
    return std::make_shared<scan_matching::CSMMatcher>(
        config.scan_matching.csm.linear_search_window,
        config.scan_matching.csm.angular_search_window,
        config.scan_matching.csm.linear_step,
        config.scan_matching.csm.angular_step,
        config.scan_matching.yaw_information_multiplier);
  }
  throw std::runtime_error("Unknown scan_matcher_type: " + config.scan_matching.type);
}

inline scan_matching::ScanMatcherPtr build_loop_matcher(const SlamConfig& config) {
  if (config.loop_closure.matcher_type == "multi_start_coarse_to_fine") {
    auto coarse = std::make_shared<scan_matching::NDTMatcher>(
        config.loop_closure.icp.max_iterations,
        config.loop_closure.icp.tolerance,
        config.loop_closure.ndt.cell_sizes,
        config.loop_closure.ndt.use_bilinear,
        config.loop_closure.yaw_information_multiplier);
    auto fine = std::make_shared<scan_matching::ICPMatcher>(
        config.loop_closure.icp.max_iterations,
        config.loop_closure.icp.tolerance,
        config.loop_closure.icp.max_correspondence_dist,
        config.loop_closure.icp.robust_kernel,
        config.loop_closure.icp.robust_kernel_scale,
        config.loop_closure.yaw_information_multiplier,
        config.loop_closure.icp.motion_prior_weight_x,
        config.loop_closure.icp.motion_prior_weight_y,
        config.loop_closure.icp.motion_prior_weight_yaw,
        config.loop_closure.icp.tolerance_trans,
        config.loop_closure.icp.tolerance_rot);
    return std::make_shared<scan_matching::MultiStartCoarseToFineMatcher>(
        coarse, fine,
        config.scan_matching.multi_start.angular_search_window_deg,
        config.scan_matching.multi_start.angular_step_deg,
        config.scan_matching.multi_start.enable_straight_hypothesis,
        config.scan_matching.multi_start.enable_const_vel_hypothesis);
  } else if (config.loop_closure.matcher_type == "multi_start_icp") {
    return std::make_shared<scan_matching::MultiStartICPMatcher>(
        config.loop_closure.icp.max_iterations,
        config.loop_closure.icp.tolerance,
        config.loop_closure.icp.max_correspondence_dist,
        config.loop_closure.icp.robust_kernel,
        config.loop_closure.icp.robust_kernel_scale,
        config.loop_closure.yaw_information_multiplier);
  } else if (config.loop_closure.matcher_type == "coarse_to_fine") {
    auto coarse = std::make_shared<scan_matching::NDTMatcher>(
        config.loop_closure.icp.max_iterations,
        config.loop_closure.icp.tolerance,
        config.loop_closure.ndt.cell_sizes,
        config.loop_closure.ndt.use_bilinear,
        config.loop_closure.yaw_information_multiplier);
    auto fine = std::make_shared<scan_matching::ICPMatcher>(
        config.loop_closure.icp.max_iterations,
        config.loop_closure.icp.tolerance,
        config.loop_closure.icp.max_correspondence_dist,
        config.loop_closure.icp.robust_kernel,
        config.loop_closure.icp.robust_kernel_scale,
        config.loop_closure.yaw_information_multiplier,
        config.loop_closure.icp.motion_prior_weight_x,
        config.loop_closure.icp.motion_prior_weight_y,
        config.loop_closure.icp.motion_prior_weight_yaw,
        config.loop_closure.icp.tolerance_trans,
        config.loop_closure.icp.tolerance_rot);
    return std::make_shared<scan_matching::CoarseToFineMatcher>(coarse, fine);
  } else if (config.loop_closure.matcher_type == "icp") {
    return std::make_shared<scan_matching::ICPMatcher>(
        config.loop_closure.icp.max_iterations,
        config.loop_closure.icp.tolerance,
        config.loop_closure.icp.max_correspondence_dist,
        config.loop_closure.icp.robust_kernel,
        config.loop_closure.icp.robust_kernel_scale,
        config.loop_closure.yaw_information_multiplier,
        config.loop_closure.icp.motion_prior_weight_x,
        config.loop_closure.icp.motion_prior_weight_y,
        config.loop_closure.icp.motion_prior_weight_yaw,
        config.loop_closure.icp.tolerance_trans,
        config.loop_closure.icp.tolerance_rot);
  } else if (config.loop_closure.matcher_type == "ndt") {
    return std::make_shared<scan_matching::NDTMatcher>(
        config.loop_closure.icp.max_iterations,
        config.loop_closure.icp.tolerance,
        config.loop_closure.ndt.cell_sizes,
        config.loop_closure.ndt.use_bilinear,
        config.loop_closure.yaw_information_multiplier);
  } else if (config.loop_closure.matcher_type == "csm") {
    return std::make_shared<scan_matching::CSMMatcher>(
        config.loop_closure.csm.linear_search_window,
        config.loop_closure.csm.angular_search_window,
        config.loop_closure.csm.linear_step,
        config.loop_closure.csm.angular_step,
        config.loop_closure.yaw_information_multiplier);
  }
  throw std::runtime_error("Unknown loop_closure_matcher_type: " + config.loop_closure.matcher_type);
}

inline scan_matching::ReferenceProviderPtr build_reference_provider(const SlamConfig& config) {
  if (config.scan_matching.reference == "scan_to_scan") {
    return std::make_shared<scan_matching::ScanToScanProvider>();
  } else if (config.scan_matching.reference == "scan_to_local_map") {
    return std::make_shared<scan_matching::LocalMapProvider>(
        config.scan_matching.local_map.window,
        config.scan_matching.local_map.radius);
  }
  throw std::runtime_error("Unknown scan_reference: " + config.scan_matching.reference);
}

inline pose_graph::PoseGraphBuilderPtr build_pose_graph_builder(const SlamConfig& config) {
  if (config.loop_closure.enabled) {
    auto inner = std::make_shared<pose_graph::ScanMatchingBuilder>(
        build_matcher(config),
        build_reference_provider(config),
        config.keyframe.min_translation,
        config.keyframe.min_rotation,
        config.scan_matching.max_failure_streak,
        config.scan_matching.max_translation_drift,
        config.scan_matching.near_links.enabled,
        config.scan_matching.near_links.buffer_size,
        config.scan_matching.near_links.max_distance,
        config.scan_matching.near_links.min_index_diff,
        config.scan_matching.near_links.max_links_per_node,
        config.scan_matching.near_links.max_translation_drift,
        config.scan_matching.near_links.max_rotation_drift_deg,
        config.scan_matching.near_links.min_eigenvalue,
        config.scan_matching.odom_fusion);
    return std::make_shared<pose_graph::LoopClosureBuilder>(
        inner,
        build_loop_matcher(config),
        config.loop_closure.search_radius,
        config.loop_closure.min_node_gap,
        config.loop_closure.max_failure_streak,
        config.loop_closure.max_dyaw_deg,
        config.loop_closure.crossing_reject_deg,
        config.loop_closure.submap_radius,
        config.loop_closure.max_score);
  } else if (config.scan_matching.enabled) {
    return std::make_shared<pose_graph::ScanMatchingBuilder>(
        build_matcher(config),
        build_reference_provider(config),
        config.keyframe.min_translation,
        config.keyframe.min_rotation,
        config.scan_matching.max_failure_streak,
        config.scan_matching.max_translation_drift,
        config.scan_matching.near_links.enabled,
        config.scan_matching.near_links.buffer_size,
        config.scan_matching.near_links.max_distance,
        config.scan_matching.near_links.min_index_diff,
        config.scan_matching.near_links.max_links_per_node,
        config.scan_matching.near_links.max_translation_drift,
        config.scan_matching.near_links.max_rotation_drift_deg,
        config.scan_matching.near_links.min_eigenvalue,
        config.scan_matching.odom_fusion);
  } else {
    return std::make_shared<pose_graph::OdomOnlyBuilder>(
        config.keyframe.min_translation,
        config.keyframe.min_rotation);
  }
}

inline map_manager::MapRendererPtr build_renderer(const SlamConfig& config) {
  if (config.map.renderer == "counting") {
    return std::make_shared<map_manager::CountingRenderer>(
        config.map.resolution,
        config.map.expansion_margin,
        config.map.hit_threshold,
        config.map.min_hits,
        config.map.hit_weight,
        config.map.miss_weight,
        config.map.miss_clearance_margin,
        config.map.max_miss_ratio);
  } else {
    return std::make_shared<map_manager::OverwriteRenderer>(
        config.map.resolution,
        config.map.expansion_margin);
  }
}

}  // namespace core
}  // namespace slam_gnss_2d
