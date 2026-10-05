#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_gnss_2d_msgs/msg/pose_graph_diff.hpp"
#include "slam_gnss_2d_msgs/srv/get_pose_graph.hpp"

namespace slam_gnss_2d {

class PoseGraphPreviewNode : public rclcpp::Node {
 public:
  PoseGraphPreviewNode() : Node("pose_graph_preview_node") {
    declare_parameter<std::string>("pose_graph_file", "");
    pose_graph_file_ = get_parameter("pose_graph_file").as_string();

    srv_ = create_service<slam_gnss_2d_msgs::srv::GetPoseGraph>(
        "slam_gnss_2d/get_pose_graph",
        std::bind(&PoseGraphPreviewNode::handle_get_pose_graph, this,
                  std::placeholders::_1, std::placeholders::_2));

    diff_pub_ = create_publisher<slam_gnss_2d_msgs::msg::PoseGraphDiff>(
        "slam_gnss_2d/pose_graph_diff", 1);

    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

    graph_diff_.full_refresh_needed = true;

    if (!pose_graph_file_.empty() && std::filesystem::exists(pose_graph_file_)) {
      RCLCPP_INFO(get_logger(), "Loading pose graph from %s", pose_graph_file_.c_str());
      load_pose_graph();
      load_map();

      timer_ = create_wall_timer(
          std::chrono::seconds(2),
          std::bind(&PoseGraphPreviewNode::publish_initial_data, this));
    } else {
      RCLCPP_WARN(get_logger(), "pose_graph_file not found or empty: %s",
                  pose_graph_file_.c_str());
    }
  }

 private:
  void load_pose_graph() {
    try {
      std::ifstream ifs(pose_graph_file_);
      if (!ifs.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open pose_graph_file: %s",
                     pose_graph_file_.c_str());
        return;
      }
      nlohmann::json data = nlohmann::json::parse(ifs);

      if (data.contains("nodes") && data["nodes"].is_array()) {
        for (const auto& n : data["nodes"]) {
          graph_diff_.new_node_indices.push_back(n.value("index", 0));
          graph_diff_.new_node_x.push_back(n.value("x", 0.0));
          graph_diff_.new_node_y.push_back(n.value("y", 0.0));
          graph_diff_.new_node_yaw.push_back(n.value("yaw", 0.0));
          graph_diff_.new_node_timestamps.push_back(n.value("timestamp", 0.0));
        }
      }

      if (data.contains("sequential_edges") && data["sequential_edges"].is_array()) {
        for (const auto& e : data["sequential_edges"]) {
          graph_diff_.seq_edge_from.push_back(e.value("from", 0));
          graph_diff_.seq_edge_to.push_back(e.value("to", 0));
          bool is_fallback = e.value("is_odom_fallback", false);
          graph_diff_.seq_edge_type.push_back(is_fallback ? 1 : 0);
          graph_diff_.seq_edge_score.push_back(e.value("score", 0.0));
          graph_diff_.seq_edge_info_diag.push_back(0.0);
          graph_diff_.seq_edge_info_diag.push_back(0.0);
          graph_diff_.seq_edge_info_diag.push_back(0.0);
        }
      }

      if (data.contains("loop_edges") && data["loop_edges"].is_array()) {
        for (const auto& e : data["loop_edges"]) {
          graph_diff_.loop_edge_from.push_back(e.value("from", 0));
          graph_diff_.loop_edge_to.push_back(e.value("to", 0));
          graph_diff_.loop_edge_score.push_back(e.value("score", 0.0));
          graph_diff_.loop_edge_info_diag.push_back(0.0);
          graph_diff_.loop_edge_info_diag.push_back(0.0);
          graph_diff_.loop_edge_info_diag.push_back(0.0);
        }
      }

      RCLCPP_INFO(get_logger(),
                  "Loaded %zu nodes, %zu seq edges, %zu loop edges.",
                  graph_diff_.new_node_indices.size(),
                  graph_diff_.seq_edge_from.size(),
                  graph_diff_.loop_edge_from.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load pose_graph.json: %s", e.what());
    }
  }

  void load_map() {
    try {
      std::filesystem::path graph_path(pose_graph_file_);
      std::filesystem::path map_dir = graph_path.parent_path();
      std::filesystem::path map_yaml_path = map_dir / "map.yaml";

      if (!std::filesystem::exists(map_yaml_path)) {
        RCLCPP_WARN(get_logger(), "map.yaml not found at %s", map_yaml_path.c_str());
        return;
      }

      YAML::Node map_info = YAML::LoadFile(map_yaml_path.string());
      std::string image_subpath = map_info["image"].as<std::string>();
      std::filesystem::path image_path = map_dir / image_subpath;

      if (!std::filesystem::exists(image_path)) {
        RCLCPP_WARN(get_logger(), "Map image not found at %s", image_path.c_str());
        return;
      }

      cv::Mat img = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
      if (img.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to read image %s", image_path.c_str());
        return;
      }

      cv::Mat img_flipped;
      cv::flip(img, img_flipped, 0);

      double occ_th = map_info["occupied_thresh"] ? map_info["occupied_thresh"].as<double>() : 0.65;
      double free_th = map_info["free_thresh"] ? map_info["free_thresh"].as<double>() : 0.196;
      int negate = map_info["negate"] ? map_info["negate"].as<int>() : 0;

      nav_msgs::msg::OccupancyGrid map_msg;
      map_msg.header.stamp = now();
      map_msg.header.frame_id = "map";
      map_msg.info.resolution = map_info["resolution"].as<double>();
      map_msg.info.width = img_flipped.cols;
      map_msg.info.height = img_flipped.rows;
      map_msg.info.origin.position.x = map_info["origin"][0].as<double>();
      map_msg.info.origin.position.y = map_info["origin"][1].as<double>();
      map_msg.info.origin.position.z = map_info["origin"][2].as<double>();
      map_msg.info.origin.orientation.w = 1.0;

      map_msg.data.resize(img_flipped.rows * img_flipped.cols);

      for (int r = 0; r < img_flipped.rows; ++r) {
        for (int c = 0; c < img_flipped.cols; ++c) {
          uint8_t pixel = img_flipped.at<uint8_t>(r, c);
          double val = static_cast<double>(pixel) / 255.0;
          if (negate != 0) {
            val = 1.0 - val;
          }
          double occ = 1.0 - val;

          int8_t cell_val = -1;
          if (occ > occ_th) {
            cell_val = 100;
          } else if (occ < free_th) {
            cell_val = 0;
          }
          map_msg.data[r * img_flipped.cols + c] = cell_val;
        }
      }

      map_msg_ = map_msg;
      has_map_msg_ = true;
      RCLCPP_INFO(get_logger(), "Loaded map %s (%ux%u)",
                  image_path.c_str(), map_msg.info.width, map_msg.info.height);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load map: %s", e.what());
    }
  }

  void handle_get_pose_graph(
      const std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Request> /*request*/,
      std::shared_ptr<slam_gnss_2d_msgs::srv::GetPoseGraph::Response> response) {
    response->graph = graph_diff_;
    response->total_nodes = graph_diff_.new_node_indices.size();
    response->total_seq_edges = graph_diff_.seq_edge_from.size();
    response->total_loop_edges = graph_diff_.loop_edge_from.size();
    response->icp_attempt_count = 0;
    response->icp_success_count = 0;
    int fallback_count = 0;
    for (auto t : graph_diff_.seq_edge_type) {
      if (t == 1) {
        fallback_count++;
      }
    }
    response->odom_fallback_count = fallback_count;
    response->loop_attempt_count = 0;
    response->loop_success_count = 0;
  }

  void publish_initial_data() {
    diff_pub_->publish(graph_diff_);
    if (has_map_msg_) {
      map_msg_.header.stamp = now();
      map_pub_->publish(map_msg_);
      RCLCPP_INFO(get_logger(), "Published initial pose graph and map.");
    } else {
      RCLCPP_INFO(get_logger(), "Published initial pose graph (no map).");
    }
    timer_->cancel();
  }

  std::string pose_graph_file_;
  rclcpp::Service<slam_gnss_2d_msgs::srv::GetPoseGraph>::SharedPtr srv_;
  rclcpp::Publisher<slam_gnss_2d_msgs::msg::PoseGraphDiff>::SharedPtr diff_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  slam_gnss_2d_msgs::msg::PoseGraphDiff graph_diff_;
  nav_msgs::msg::OccupancyGrid map_msg_;
  bool has_map_msg_{false};
};

}  // namespace slam_gnss_2d

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam_gnss_2d::PoseGraphPreviewNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
