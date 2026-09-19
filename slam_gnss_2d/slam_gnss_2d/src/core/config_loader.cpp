#include "slam_gnss_2d/core/config_loader.hpp"

namespace slam_gnss_2d {

template <typename T>
static void declare_param_if_not_declared(rclcpp::Node& node, const std::string& name, const T& default_val) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_val);
  }
}

void ConfigLoader::declare_params(rclcpp::Node& node) {
  declare_param_if_not_declared(node, "topics.scan", std::string("/scan_top_lidar"));
  declare_param_if_not_declared(node, "topics.odom", std::string("/odom"));

  declare_param_if_not_declared(node, "map.resolution", 0.05);
  declare_param_if_not_declared(node, "map.expansion_margin", 100.0);
  declare_param_if_not_declared(node, "map.publish_hz", 1.0);
  declare_param_if_not_declared(node, "map.renderer", std::string("overwrite"));
  declare_param_if_not_declared(node, "map.hit_threshold", 0.3);
  declare_param_if_not_declared(node, "map.min_hits", 2);

  declare_param_if_not_declared(node, "keyframe.min_translation", 1.0);
  declare_param_if_not_declared(node, "keyframe.min_rotation", 0.1);

  declare_param_if_not_declared(node, "scan_matching.enabled", true);
  declare_param_if_not_declared(node, "scan_matching.type", std::string("ndt"));
  declare_param_if_not_declared(node, "scan_matching.reference", std::string("scan_to_local_map"));
  declare_param_if_not_declared(node, "scan_matching.max_failure_streak", 5);
  declare_param_if_not_declared(node, "scan_matching.yaw_information_multiplier", 100.0);
  declare_param_if_not_declared(node, "scan_matching.icp.max_iterations", 100);
  declare_param_if_not_declared(node, "scan_matching.icp.tolerance", 1e-5);
  declare_param_if_not_declared(node, "scan_matching.icp.max_correspondence_dist", 1.0);
  declare_param_if_not_declared(node, "scan_matching.icp.robust_kernel", std::string("huber"));
  declare_param_if_not_declared(node, "scan_matching.icp.robust_kernel_scale", 0.1);
  declare_param_if_not_declared(node, "scan_matching.ndt.cell_size", 1.0);
  declare_param_if_not_declared(node, "scan_matching.ndt.cell_sizes", std::vector<double>{1.0});
  declare_param_if_not_declared(node, "scan_matching.ndt.use_bilinear", false);
  declare_param_if_not_declared(node, "scan_matching.csm.linear_search_window", 1.0);
  declare_param_if_not_declared(node, "scan_matching.csm.angular_search_window", 0.5);
  declare_param_if_not_declared(node, "scan_matching.csm.linear_step", 0.05);
  declare_param_if_not_declared(node, "scan_matching.csm.angular_step", 0.02);
  declare_param_if_not_declared(node, "scan_matching.local_map.window", 30);
  declare_param_if_not_declared(node, "scan_matching.local_map.radius", 30.0);
  declare_param_if_not_declared(node, "scan_matching.max_translation_drift", 0.08);
  declare_param_if_not_declared(node, "scan_matching.multi_start.angular_search_window_deg", 20.0);
  declare_param_if_not_declared(node, "scan_matching.multi_start.angular_step_deg", 2.5);
  declare_param_if_not_declared(node, "scan_matching.multi_start.enable_straight_hypothesis", true);
  declare_param_if_not_declared(node, "scan_matching.multi_start.enable_const_vel_hypothesis", true);

  declare_param_if_not_declared(node, "loop_closure.enabled", true);
  declare_param_if_not_declared(node, "loop_closure.search_radius", 2.0);
  declare_param_if_not_declared(node, "loop_closure.min_node_gap", 50);
  declare_param_if_not_declared(node, "loop_closure.max_failure_streak", 3);
  declare_param_if_not_declared(node, "loop_closure.matcher_type", std::string("icp"));
  declare_param_if_not_declared(node, "loop_closure.yaw_information_multiplier", 100.0);
  declare_param_if_not_declared(node, "loop_closure.icp.max_iterations", 100);
  declare_param_if_not_declared(node, "loop_closure.icp.tolerance", 1e-5);
  declare_param_if_not_declared(node, "loop_closure.icp.max_correspondence_dist", 1.0);
  declare_param_if_not_declared(node, "loop_closure.icp.robust_kernel", std::string("huber"));
  declare_param_if_not_declared(node, "loop_closure.icp.robust_kernel_scale", 0.1);
  declare_param_if_not_declared(node, "loop_closure.ndt.cell_size", 1.0);
  declare_param_if_not_declared(node, "loop_closure.ndt.cell_sizes", std::vector<double>{1.0});
  declare_param_if_not_declared(node, "loop_closure.ndt.use_bilinear", false);
  declare_param_if_not_declared(node, "loop_closure.csm.linear_search_window", 1.0);
  declare_param_if_not_declared(node, "loop_closure.csm.angular_search_window", 0.5);
  declare_param_if_not_declared(node, "loop_closure.csm.linear_step", 0.05);
  declare_param_if_not_declared(node, "loop_closure.csm.angular_step", 0.02);
  declare_param_if_not_declared(node, "loop_closure.max_dyaw_deg", 145.0);
  declare_param_if_not_declared(node, "loop_closure.crossing_reject_deg", 45.0);
  declare_param_if_not_declared(node, "loop_closure.submap_radius", 5.0);
  declare_param_if_not_declared(node, "loop_closure.max_score", 0.0);

  declare_param_if_not_declared(node, "gnss.enabled", true);
  declare_param_if_not_declared(node, "gnss.source", std::string("navpvt"));
  declare_param_if_not_declared(node, "gnss.topics.fix", std::string("/gps/fix"));
  declare_param_if_not_declared(node, "gnss.topics.navpvt", std::string("/navpvt"));
  declare_param_if_not_declared(node, "gnss.navpvt_hacc_scale", 1.0);
  declare_param_if_not_declared(node, "gnss.min_interval_m", 1.0);
  declare_param_if_not_declared(node, "gnss.max_innovation_m", 0.0);
  declare_param_if_not_declared(node, "gnss.robust_kernel", std::string("huber"));
  declare_param_if_not_declared(node, "gnss.robust_kernel_scale", 1.345);
  declare_param_if_not_declared(node, "gnss.prior.min_fix_status", 1);
  declare_param_if_not_declared(node, "gnss.dynamic_reanchor.enabled", false);
  declare_param_if_not_declared(node, "gnss.dynamic_reanchor.min_fix_status", 2);
  declare_param_if_not_declared(node, "gnss.dynamic_reanchor.min_samples", 10);
  declare_param_if_not_declared(node, "gnss.dynamic_reanchor.min_distance_m", 10.0);
  declare_param_if_not_declared(node, "gnss.validation.max_sigma_m", 5.0);
  declare_param_if_not_declared(node, "save_dir", std::string(""));

  declare_param_if_not_declared(node, "gnss.anchor.min_fix_status", 0);
  declare_param_if_not_declared(node, "gnss.anchor.sigma_m", 0.05);
  declare_param_if_not_declared(node, "gnss.anchor.init_yaw_sigma_rad", 10.0);
  declare_param_if_not_declared(node, "gnss.anchor.init_distance_m", 2.0);
  declare_param_if_not_declared(node, "gnss.sigma.fix_m", 0.02);
  declare_param_if_not_declared(node, "gnss.sigma.float_m", 0.5);
  declare_param_if_not_declared(node, "gnss.sigma.factor_yaw_variance", 1e8);

  declare_param_if_not_declared(node, "optimization.backend", std::string("isam2"));
  declare_param_if_not_declared(node, "optimization.isam2.relinearize_threshold", 0.1);
  declare_param_if_not_declared(node, "optimization.rerender_threshold_m", 0.1);

  declare_param_if_not_declared(node, "trajectory_noise_filter.enabled", false);
  declare_param_if_not_declared(node, "trajectory_noise_filter.type", std::string("clear"));
  declare_param_if_not_declared(node, "trajectory_noise_filter.radius_m", 0.5);
}

SlamConfig ConfigLoader::build_config(const rclcpp::Node& node) {
  SlamConfig cfg;

  cfg.topics.scan = node.get_parameter("topics.scan").as_string();
  cfg.topics.odom = node.get_parameter("topics.odom").as_string();

  cfg.map.resolution = node.get_parameter("map.resolution").as_double();
  cfg.map.expansion_margin = node.get_parameter("map.expansion_margin").as_double();
  cfg.map.publish_hz = node.get_parameter("map.publish_hz").as_double();
  cfg.map.renderer = node.get_parameter("map.renderer").as_string();
  cfg.map.hit_threshold = node.get_parameter("map.hit_threshold").as_double();
  cfg.map.min_hits = node.get_parameter("map.min_hits").as_int();

  cfg.keyframe.min_translation = node.get_parameter("keyframe.min_translation").as_double();
  cfg.keyframe.min_rotation = node.get_parameter("keyframe.min_rotation").as_double();

  cfg.scan_matching.enabled = node.get_parameter("scan_matching.enabled").as_bool();
  cfg.scan_matching.type = node.get_parameter("scan_matching.type").as_string();
  cfg.scan_matching.reference = node.get_parameter("scan_matching.reference").as_string();
  cfg.scan_matching.max_failure_streak = node.get_parameter("scan_matching.max_failure_streak").as_int();
  cfg.scan_matching.yaw_information_multiplier =
      node.get_parameter("scan_matching.yaw_information_multiplier").as_double();
  cfg.scan_matching.icp.max_iterations = node.get_parameter("scan_matching.icp.max_iterations").as_int();
  cfg.scan_matching.icp.tolerance = node.get_parameter("scan_matching.icp.tolerance").as_double();
  cfg.scan_matching.icp.max_correspondence_dist =
      node.get_parameter("scan_matching.icp.max_correspondence_dist").as_double();
  cfg.scan_matching.icp.robust_kernel = node.get_parameter("scan_matching.icp.robust_kernel").as_string();
  cfg.scan_matching.icp.robust_kernel_scale = node.get_parameter("scan_matching.icp.robust_kernel_scale").as_double();
  cfg.scan_matching.ndt.cell_size = node.get_parameter("scan_matching.ndt.cell_size").as_double();
  cfg.scan_matching.ndt.cell_sizes = node.get_parameter("scan_matching.ndt.cell_sizes").as_double_array();
  cfg.scan_matching.ndt.use_bilinear = node.get_parameter("scan_matching.ndt.use_bilinear").as_bool();
  cfg.scan_matching.csm.linear_search_window = node.get_parameter("scan_matching.csm.linear_search_window").as_double();
  cfg.scan_matching.csm.angular_search_window =
      node.get_parameter("scan_matching.csm.angular_search_window").as_double();
  cfg.scan_matching.csm.linear_step = node.get_parameter("scan_matching.csm.linear_step").as_double();
  cfg.scan_matching.csm.angular_step = node.get_parameter("scan_matching.csm.angular_step").as_double();
  cfg.scan_matching.local_map.window = node.get_parameter("scan_matching.local_map.window").as_int();
  cfg.scan_matching.local_map.radius = node.get_parameter("scan_matching.local_map.radius").as_double();
  cfg.scan_matching.max_translation_drift =
      node.get_parameter("scan_matching.max_translation_drift").as_double();
  cfg.scan_matching.multi_start.angular_search_window_deg =
      node.get_parameter("scan_matching.multi_start.angular_search_window_deg").as_double();
  cfg.scan_matching.multi_start.angular_step_deg =
      node.get_parameter("scan_matching.multi_start.angular_step_deg").as_double();
  cfg.scan_matching.multi_start.enable_straight_hypothesis =
      node.get_parameter("scan_matching.multi_start.enable_straight_hypothesis").as_bool();
  cfg.scan_matching.multi_start.enable_const_vel_hypothesis =
      node.get_parameter("scan_matching.multi_start.enable_const_vel_hypothesis").as_bool();

  cfg.loop_closure.enabled = node.get_parameter("loop_closure.enabled").as_bool();
  cfg.loop_closure.search_radius = node.get_parameter("loop_closure.search_radius").as_double();
  cfg.loop_closure.min_node_gap = node.get_parameter("loop_closure.min_node_gap").as_int();
  cfg.loop_closure.max_failure_streak = node.get_parameter("loop_closure.max_failure_streak").as_int();
  cfg.loop_closure.matcher_type = node.get_parameter("loop_closure.matcher_type").as_string();
  cfg.loop_closure.yaw_information_multiplier =
      node.get_parameter("loop_closure.yaw_information_multiplier").as_double();
  cfg.loop_closure.icp.max_iterations = node.get_parameter("loop_closure.icp.max_iterations").as_int();
  cfg.loop_closure.icp.tolerance = node.get_parameter("loop_closure.icp.tolerance").as_double();
  cfg.loop_closure.icp.max_correspondence_dist =
      node.get_parameter("loop_closure.icp.max_correspondence_dist").as_double();
  cfg.loop_closure.icp.robust_kernel = node.get_parameter("loop_closure.icp.robust_kernel").as_string();
  cfg.loop_closure.icp.robust_kernel_scale = node.get_parameter("loop_closure.icp.robust_kernel_scale").as_double();
  cfg.loop_closure.ndt.cell_size = node.get_parameter("loop_closure.ndt.cell_size").as_double();
  cfg.loop_closure.ndt.cell_sizes = node.get_parameter("loop_closure.ndt.cell_sizes").as_double_array();
  cfg.loop_closure.ndt.use_bilinear = node.get_parameter("loop_closure.ndt.use_bilinear").as_bool();
  cfg.loop_closure.csm.linear_search_window = node.get_parameter("loop_closure.csm.linear_search_window").as_double();
  cfg.loop_closure.csm.angular_search_window =
      node.get_parameter("loop_closure.csm.angular_search_window").as_double();
  cfg.loop_closure.csm.linear_step = node.get_parameter("loop_closure.csm.linear_step").as_double();
  cfg.loop_closure.csm.angular_step = node.get_parameter("loop_closure.csm.angular_step").as_double();
  cfg.loop_closure.max_dyaw_deg = node.get_parameter("loop_closure.max_dyaw_deg").as_double();
  cfg.loop_closure.crossing_reject_deg = node.get_parameter("loop_closure.crossing_reject_deg").as_double();
  cfg.loop_closure.submap_radius = node.get_parameter("loop_closure.submap_radius").as_double();
  cfg.loop_closure.max_score = node.get_parameter("loop_closure.max_score").as_double();

  cfg.gnss.enabled = node.get_parameter("gnss.enabled").as_bool();
  cfg.gnss.source = node.get_parameter("gnss.source").as_string();
  cfg.gnss.topics.fix = node.get_parameter("gnss.topics.fix").as_string();
  cfg.gnss.topics.navpvt = node.get_parameter("gnss.topics.navpvt").as_string();
  cfg.gnss.navpvt_hacc_scale = node.get_parameter("gnss.navpvt_hacc_scale").as_double();
  cfg.gnss.min_interval_m = node.get_parameter("gnss.min_interval_m").as_double();
  cfg.gnss.max_innovation_m = node.get_parameter("gnss.max_innovation_m").as_double();
  cfg.gnss.robust_kernel = node.get_parameter("gnss.robust_kernel").as_string();
  cfg.gnss.robust_kernel_scale = node.get_parameter("gnss.robust_kernel_scale").as_double();
  cfg.gnss.prior_min_fix_status = node.get_parameter("gnss.prior.min_fix_status").as_int();
  cfg.gnss.dynamic_reanchor.enabled = node.get_parameter("gnss.dynamic_reanchor.enabled").as_bool();
  cfg.gnss.dynamic_reanchor.min_fix_status = node.get_parameter("gnss.dynamic_reanchor.min_fix_status").as_int();
  cfg.gnss.dynamic_reanchor.min_samples = node.get_parameter("gnss.dynamic_reanchor.min_samples").as_int();
  cfg.gnss.dynamic_reanchor.min_distance_m = node.get_parameter("gnss.dynamic_reanchor.min_distance_m").as_double();
  cfg.gnss.validation.max_sigma_m = node.get_parameter("gnss.validation.max_sigma_m").as_double();
  cfg.save_dir = node.get_parameter("save_dir").as_string();

  cfg.gnss.anchor.min_fix_status = node.get_parameter("gnss.anchor.min_fix_status").as_int();
  cfg.gnss.anchor.sigma_m = node.get_parameter("gnss.anchor.sigma_m").as_double();
  cfg.gnss.anchor.init_yaw_sigma_rad = node.get_parameter("gnss.anchor.init_yaw_sigma_rad").as_double();
  cfg.gnss.anchor.init_distance_m = node.get_parameter("gnss.anchor.init_distance_m").as_double();
  cfg.gnss.sigma.fix_m = node.get_parameter("gnss.sigma.fix_m").as_double();
  cfg.gnss.sigma.float_m = node.get_parameter("gnss.sigma.float_m").as_double();
  cfg.gnss.sigma.factor_yaw_variance = node.get_parameter("gnss.sigma.factor_yaw_variance").as_double();

  cfg.optimization.backend = node.get_parameter("optimization.backend").as_string();
  cfg.optimization.isam2.relinearize_threshold =
      node.get_parameter("optimization.isam2.relinearize_threshold").as_double();
  cfg.optimization.rerender_threshold_m = node.get_parameter("optimization.rerender_threshold_m").as_double();

  cfg.trajectory_noise_filter.enabled = node.get_parameter("trajectory_noise_filter.enabled").as_bool();
  cfg.trajectory_noise_filter.type = node.get_parameter("trajectory_noise_filter.type").as_string();
  cfg.trajectory_noise_filter.radius_m = node.get_parameter("trajectory_noise_filter.radius_m").as_double();

  return cfg;
}

}  // namespace slam_gnss_2d
