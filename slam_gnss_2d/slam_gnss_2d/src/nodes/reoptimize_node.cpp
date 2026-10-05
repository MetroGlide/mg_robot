#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <slam_gnss_2d_msgs/srv/save_slam_map.hpp>

#include "slam_gnss_2d/core/component_factory.hpp"
#include "slam_gnss_2d/core/config_loader.hpp"
#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/core/geometry.hpp"
#include "slam_gnss_2d/core/slam_data_saver.hpp"
#include "slam_gnss_2d/input/ros2/bag_reader.hpp"
#include "slam_gnss_2d/map_manager/base.hpp"
#include "slam_gnss_2d/map_manager/trajectory_noise_filter.hpp"
#include "slam_gnss_2d/optimizer/gtsam_optimizer.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"
#include "slam_gnss_2d/ros/slam_visualizer.hpp"
#include "slam_gnss_2d/tools/reoptimize_geometry.hpp"
#include "slam_gnss_2d/tools/reoptimize_io.hpp"
#include "slam_gnss_2d/tools/reoptimize_loop_search.hpp"

namespace slam_gnss_2d {

class ReadOnlyPoseGraph : public pose_graph::PoseGraphBuilderBase {
 public:
  ReadOnlyPoseGraph(
      const std::vector<core::PoseNode>& nodes,
      const std::vector<core::PoseEdge>& seq_edges,
      const std::vector<core::PoseEdge>& loop_edges)
      : nodes_(nodes), seq_edges_(seq_edges), loop_edges_(loop_edges) {}

  std::optional<core::PoseNode> add_scan(
      [[maybe_unused]] const core::ScanDataPtr& scan,
      [[maybe_unused]] const core::OdomData& odom) override {
    return std::nullopt;
  }

  std::vector<core::PoseNode> get_nodes() const override { return nodes_; }
  std::vector<core::PoseEdge> get_edges() const override {
    auto edges = seq_edges_;
    edges.insert(edges.end(), loop_edges_.begin(), loop_edges_.end());
    return edges;
  }
  std::vector<core::PoseEdge> get_loop_edges() const { return loop_edges_; }
  void reset() override {}
  bool loop_just_closed() override { return false; }

 private:
  std::vector<core::PoseNode> nodes_;
  std::vector<core::PoseEdge> seq_edges_;
  std::vector<core::PoseEdge> loop_edges_;
};

class ReoptimizeNode : public rclcpp::Node {
 public:
  ReoptimizeNode() : rclcpp::Node("reoptimize_node") {
    declare_parameter("input_dir", "");
    declare_parameter("bag_path", "");
    declare_parameter("enable_re_scan_matching", false);
    declare_parameter("enable_new_loop_search", false);
    declare_parameter("multi_guess_shift_m", 0.2);
    declare_parameter("multi_guess_shift_deg", 5.0);
    declare_parameter("save_dir", "");

    ConfigLoader::declare_params(*this);
    config_ = ConfigLoader::build_config(*this);

    auto node_shared = shared_from_this();
    visualizer_ = std::make_shared<ros::SlamVisualizer>(node_shared, true);

    save_srv_ = create_service<slam_gnss_2d_msgs::srv::SaveSlamMap>(
        "slam_gnss_2d/save_slam_map",
        [this](
            const std::shared_ptr<slam_gnss_2d_msgs::srv::SaveSlamMap::Request> req,
            std::shared_ptr<slam_gnss_2d_msgs::srv::SaveSlamMap::Response> res) {
          this->handle_save_slam_map(req, res);
        });

    timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() { this->run_optimization(); });
  }

 private:
  core::SlamConfig config_;
  std::shared_ptr<ros::SlamVisualizer> visualizer_;
  rclcpp::Service<slam_gnss_2d_msgs::srv::SaveSlamMap>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<core::PoseNode> optimized_nodes_;
  std::vector<core::PoseEdge> new_edges_;
  YAML::Node gnss_transform_data_;
  std::string bag_path_;
  std::shared_ptr<map_manager::MapRendererBase> renderer_;
  std::shared_ptr<core::GraphOrchestrator> dummy_orchestrator_;

  std::optional<core::MatchResult> match_with_multi_guess(
      scan_matching::ScanMatcherBase& matcher,
      const core::ScanDataPtr& dst_scan,
      const core::OdomData& base_guess,
      const std::optional<core::OdomData>& gnss_guess,
      double shift_m,
      double shift_rad) {
    std::vector<core::OdomData> candidates = {
        base_guess,
        core::OdomData{base_guess.timestamp, base_guess.x + shift_m, base_guess.y, base_guess.yaw},
        core::OdomData{base_guess.timestamp, base_guess.x - shift_m, base_guess.y, base_guess.yaw},
        core::OdomData{base_guess.timestamp, base_guess.x, base_guess.y + shift_m, base_guess.yaw},
        core::OdomData{base_guess.timestamp, base_guess.x, base_guess.y - shift_m, base_guess.yaw},
        core::OdomData{base_guess.timestamp, base_guess.x, base_guess.y, base_guess.yaw + shift_rad},
        core::OdomData{base_guess.timestamp, base_guess.x, base_guess.y, base_guess.yaw - shift_rad},
    };
    if (gnss_guess.has_value()) {
      candidates.push_back(*gnss_guess);
      candidates.push_back(core::OdomData{
          gnss_guess->timestamp, gnss_guess->x, gnss_guess->y, gnss_guess->yaw + shift_rad});
    }

    std::optional<core::MatchResult> best_result;
    double best_score = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < candidates.size(); ++i) {
      auto res = matcher.match(dst_scan, candidates[i]);
      if (!res.converged) {
        continue;
      }
      if (res.score < best_score || (res.score == best_score && i == 0)) {
        best_score = res.score;
        best_result = res;
      }
    }
    return best_result;
  }

  void run_optimization() {
    timer_->cancel();
    std::string input_dir = get_parameter("input_dir").as_string();
    if (input_dir.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'input_dir' is required!");
      return;
    }

    nlohmann::json pg_data;
    try {
      std::tie(pg_data, gnss_transform_data_) = tools::load_pose_graph_and_transform(input_dir);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load input files: %s", e.what());
      return;
    }

    try {
      bag_path_ = tools::resolve_bag_path(get_parameter("bag_path").as_string(), pg_data);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return;
    }

    renderer_ = core::build_renderer(config_);

    // 1. Extract scans
    input::BagScanSource scan_source(bag_path_, config_.topics.scan);
    scan_source.start();
    std::vector<core::ScanDataPtr> all_scans;
    scan_source.set_scan_callback([&all_scans](const core::ScanDataPtr& s) {
      all_scans.push_back(s);
    });
    while (scan_source.step()) {}
    scan_source.stop();
    RCLCPP_INFO(get_logger(), "Loaded %zu scans from bag", all_scans.size());

    // 2. Reconstruct old nodes
    std::vector<core::PoseNode> old_nodes;
    std::unordered_map<int, core::ScanDataPtr> node_scans;
    for (const auto& n_json : pg_data["nodes"]) {
      int idx = n_json["index"];
      double ts = n_json["timestamp"];
      auto scan = tools::find_nearest_scan(all_scans, ts, 0.1);
      core::PoseNode node{
          idx, ts, n_json["x"], n_json["y"], n_json["yaw"], scan};
      old_nodes.push_back(node);
      if (scan) {
        node_scans[idx] = scan;
      }
    }

    visualizer_->rebuild_path(old_nodes);
    visualizer_->publish_path_before_optimize();

    // 3. GNSS Priors
    std::vector<core::GnssPrior> gnss_priors;
    double anchor_easting = 0.0, anchor_northing = 0.0;
    bool has_anchor = false;
    if (gnss_transform_data_["anchor_utm"]) {
      anchor_easting = gnss_transform_data_["anchor_utm"]["easting"].as<double>();
      anchor_northing = gnss_transform_data_["anchor_utm"]["northing"].as<double>();
      has_anchor = true;
    }

    if (config_.gnss.enabled && has_anchor) {
      std::shared_ptr<input::GnssSourceBase> gnss_source;
      if (config_.gnss.source == "navpvt") {
        gnss_source = std::make_shared<input::BagNavPVTSource>(
            bag_path_, config_.gnss.topics.navpvt, config_.gnss.navpvt_hacc_scale);
      } else {
        gnss_source = std::make_shared<input::BagGnssSource>(
            bag_path_, config_.gnss.topics.fix);
      }
      gnss_source->start();

      for (const auto& node : old_nodes) {
        auto gnss = gnss_source->get_gnss_at(node.timestamp);
        if (!gnss.has_value()) continue;
        double sigma_xy = tools::sigma_from_covariance_or_status(*gnss, config_);
        if (sigma_xy <= 0.0 || sigma_xy > config_.gnss.validation.max_sigma_m) continue;

        double gx = gnss->x - anchor_easting;
        double gy = gnss->y - anchor_northing;
        double pos_var = sigma_xy * sigma_xy;
        Eigen::Matrix2d info_2x2 = Eigen::Matrix2d::Zero();
        info_2x2(0, 0) = 1.0 / pos_var;
        info_2x2(1, 1) = 1.0 / pos_var;
        gnss_priors.push_back(core::GnssPrior{node.index, gx, gy, info_2x2});
      }
      gnss_source->stop();
      RCLCPP_INFO(get_logger(), "Built %zu GNSS prior constraints", gnss_priors.size());
    }

    // 4. Edges
    std::vector<core::PoseEdge> seq_edges_list;
    std::vector<core::PoseEdge> loop_edges_list;
    bool enable_re_sm = get_parameter("enable_re_scan_matching").as_bool();

    auto parse_edge = [](const nlohmann::json& e_json) {
      Eigen::Matrix3d info;
      const auto& arr = e_json["information"];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          info(r, c) = arr[r * 3 + c];
        }
      }
      return core::PoseEdge{
          e_json["from"], e_json["to"],
          e_json["dx"], e_json["dy"], e_json["dyaw"],
          info, e_json.value("score", 0.0), e_json.value("is_odom_fallback", false)};
    };

    if (!enable_re_sm) {
      for (const auto& ej : pg_data["sequential_edges"]) {
        auto edge = parse_edge(ej);
        seq_edges_list.push_back(edge);
        new_edges_.push_back(edge);
      }
      for (const auto& ej : pg_data["loop_edges"]) {
        auto edge = parse_edge(ej);
        loop_edges_list.push_back(edge);
        new_edges_.push_back(edge);
      }
    } else {
      auto matcher = core::build_matcher(config_);
      auto loop_matcher = core::build_loop_matcher(config_);
      double shift_m = get_parameter("multi_guess_shift_m").as_double();
      double shift_rad = get_parameter("multi_guess_shift_deg").as_double() * M_PI / 180.0;

      for (const auto& ej : pg_data["sequential_edges"]) {
        int f_idx = ej["from"];
        int t_idx = ej["to"];
        if (node_scans.find(f_idx) == node_scans.end() || node_scans.find(t_idx) == node_scans.end()) {
          auto edge = parse_edge(ej);
          seq_edges_list.push_back(edge);
          new_edges_.push_back(edge);
          continue;
        }

        std::vector<Eigen::Vector2d> src_pts;
        if (config_.scan_matching.reference == "scan_to_local_map") {
          auto sub = tools::build_submap_points(
              f_idx, old_nodes, node_scans, config_.scan_matching.local_map.radius);
          src_pts = sub.value_or(core::scan_to_points(node_scans[f_idx]));
        } else {
          src_pts = core::scan_to_points(node_scans[f_idx]);
        }

        core::OdomData initial_guess{
            old_nodes[t_idx].timestamp, ej["dx"], ej["dy"], ej["dyaw"]};
        matcher->set_target_cloud(src_pts);
        auto res = match_with_multi_guess(
            *matcher, node_scans[t_idx], initial_guess, std::nullopt, shift_m, shift_rad);

        if (res.has_value()) {
          core::PoseEdge edge{
              f_idx, t_idx, res->dx, res->dy, res->dyaw, res->information, res->score, false};
          seq_edges_list.push_back(edge);
          new_edges_.push_back(edge);
        } else {
          auto edge = parse_edge(ej);
          seq_edges_list.push_back(edge);
          new_edges_.push_back(edge);
        }
      }

      for (const auto& ej : pg_data["loop_edges"]) {
        int f_idx = ej["from"];
        int t_idx = ej["to"];
        if (node_scans.find(f_idx) == node_scans.end() || node_scans.find(t_idx) == node_scans.end()) {
          auto edge = parse_edge(ej);
          loop_edges_list.push_back(edge);
          new_edges_.push_back(edge);
          continue;
        }

        std::vector<Eigen::Vector2d> src_pts;
        if (config_.loop_closure.submap_radius > 0.0) {
          auto sub = tools::build_submap_points(
              f_idx, old_nodes, node_scans, config_.loop_closure.submap_radius);
          src_pts = sub.value_or(core::scan_to_points(node_scans[f_idx]));
        } else {
          src_pts = core::scan_to_points(node_scans[f_idx]);
        }

        core::OdomData initial_guess{
            old_nodes[t_idx].timestamp, ej["dx"], ej["dy"], ej["dyaw"]};
        loop_matcher->set_target_cloud(src_pts);
        auto res = loop_matcher->match(node_scans[t_idx], initial_guess);

        if (res.converged && (config_.loop_closure.max_score <= 0.0 || res.score <= config_.loop_closure.max_score)) {
          core::PoseEdge edge{
              f_idx, t_idx, res.dx, res.dy, res.dyaw, res.information, res.score, false};
          loop_edges_list.push_back(edge);
          new_edges_.push_back(edge);
        } else {
          auto edge = parse_edge(ej);
          loop_edges_list.push_back(edge);
          new_edges_.push_back(edge);
        }
      }
    }

    // 5. GTSAM Optimizer
    optimizer::GTSAMOptimizer gtsam_opt;
    auto opt_pass1 = gtsam_opt.optimize(old_nodes, new_edges_, gnss_priors);

    bool enable_new_loop = get_parameter("enable_new_loop_search").as_bool();
    if (enable_new_loop && enable_re_sm) {
      auto loop_matcher = core::build_loop_matcher(config_);
      auto add_loops = tools::search_new_loop_edges(
          opt_pass1, node_scans, loop_edges_list, *loop_matcher,
          config_.loop_closure.search_radius, config_.loop_closure.min_node_gap,
          config_.loop_closure.submap_radius, config_.loop_closure.max_score,
          config_.loop_closure.max_dyaw_deg, config_.loop_closure.crossing_reject_deg,
          [this](const std::string& msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); });
      new_edges_.insert(new_edges_.end(), add_loops.begin(), add_loops.end());
      loop_edges_list.insert(loop_edges_list.end(), add_loops.begin(), add_loops.end());
      optimized_nodes_ = gtsam_opt.optimize(old_nodes, new_edges_, gnss_priors);
    } else {
      optimized_nodes_ = opt_pass1;
    }

    renderer_->rerender_all(optimized_nodes_);
    if (config_.trajectory_noise_filter.enabled) {
      map_manager::TrajectoryNoiseFilter nf(config_.trajectory_noise_filter);
      nf.apply(*renderer_, optimized_nodes_);
    }

    visualizer_->publish_map(*renderer_);
    visualizer_->rebuild_path(optimized_nodes_);

    auto ro_pg = std::make_shared<ReadOnlyPoseGraph>(optimized_nodes_, seq_edges_list, loop_edges_list);
    visualizer_->publish_pose_graph_markers(*ro_pg);
    RCLCPP_INFO(get_logger(), "Re-optimization complete. Waiting for save service call...");
  }

  void handle_save_slam_map(
      const std::shared_ptr<slam_gnss_2d_msgs::srv::SaveSlamMap::Request> req,
      std::shared_ptr<slam_gnss_2d_msgs::srv::SaveSlamMap::Response> res) {
    std::string output_dir = req->map_dir;
    if (output_dir.empty()) {
      if (has_parameter("save_dir")) {
        output_dir = get_parameter("save_dir").as_string();
      }
    }
    if (output_dir.empty()) {
      auto now = std::chrono::system_clock::now();
      auto tt = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << "/root/ros2_data/slam_maps/"
         << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S") << "_opt";
      output_dir = ss.str();
    }

    std::filesystem::path p(output_dir);
    if (p.is_relative()) {
      p = std::filesystem::absolute(p);
    }
    output_dir = p.lexically_normal().string();

    try {
      std::filesystem::create_directories(output_dir);
      core::SlamDataSaver::save_pose_graph(output_dir, optimized_nodes_, new_edges_, bag_path_);
      if (gnss_transform_data_["anchor"]) {
        double lat = gnss_transform_data_["anchor"]["latitude"].as<double>();
        double lon = gnss_transform_data_["anchor"]["longitude"].as<double>();
        double easting = gnss_transform_data_["anchor_utm"]["easting"].as<double>();
        double northing = gnss_transform_data_["anchor_utm"]["northing"].as<double>();
        int zone = gnss_transform_data_["anchor_utm"]["zone"].as<int>();
        std::string hemisphere = gnss_transform_data_["anchor_utm"]["hemisphere"].as<std::string>();
        double rot = gnss_transform_data_["rotation_rad"].as<double>();
        core::SlamDataSaver::save_gnss_transform(
            output_dir, lat, lon, easting, northing, zone, hemisphere, rot, "gtsam_batch");
      }
      res->success = true;
      res->message = "Optimized map saved to " + output_dir;
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    } catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Failed to save: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
  }
};

}  // namespace slam_gnss_2d

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam_gnss_2d::ReoptimizeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
