#include "slam_gnss_2d/core/slam_data_saver.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace slam_gnss_2d {
namespace core {

namespace {

std::string get_current_iso_time() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%dT%H:%M:%S");
  return ss.str();
}

}  // namespace

std::string SlamDataSaver::save_gnss_transform(
    const std::string& output_dir,
    double anchor_lat,
    double anchor_lon,
    double anchor_utm_easting,
    double anchor_utm_northing,
    int utm_zone,
    const std::string& utm_hemisphere,
    double rotation_rad,
    const std::string& backend_name) {
  std::filesystem::create_directories(output_dir);
  std::string filepath = (std::filesystem::path(output_dir) / "gnss_transform.yaml").string();

  YAML::Emitter out;
  out << YAML::BeginMap;

  out << YAML::Key << "anchor";
  out << YAML::BeginMap;
  out << YAML::Key << "latitude" << YAML::Value << anchor_lat;
  out << YAML::Key << "longitude" << YAML::Value << anchor_lon;
  out << YAML::EndMap;

  out << YAML::Key << "anchor_utm";
  out << YAML::BeginMap;
  out << YAML::Key << "easting" << YAML::Value << anchor_utm_easting;
  out << YAML::Key << "northing" << YAML::Value << anchor_utm_northing;
  out << YAML::Key << "zone" << YAML::Value << utm_zone;
  out << YAML::Key << "hemisphere" << YAML::Value << utm_hemisphere;
  out << YAML::EndMap;

  out << YAML::Key << "rotation_rad" << YAML::Value << rotation_rad;

  out << YAML::Key << "metadata";
  out << YAML::BeginMap;
  out << YAML::Key << "created_at" << YAML::Value << get_current_iso_time();
  out << YAML::Key << "slam_backend" << YAML::Value << backend_name;
  out << YAML::EndMap;

  out << YAML::EndMap;

  std::ofstream fout(filepath);
  fout << out.c_str() << std::endl;
  return filepath;
}

std::string SlamDataSaver::save_pose_graph(
    const std::string& output_dir,
    const std::vector<PoseNode>& nodes,
    const std::vector<PoseEdge>& edges,
    const std::optional<std::string>& bag_path) {
  std::filesystem::create_directories(output_dir);
  std::string filepath = (std::filesystem::path(output_dir) / "pose_graph.json").string();

  nlohmann::json root;

  nlohmann::json nodes_json = nlohmann::json::array();
  for (const auto& n : nodes) {
    nodes_json.push_back({
        {"index", n.index},
        {"timestamp", n.timestamp},
        {"x", n.x},
        {"y", n.y},
        {"yaw", n.yaw},
    });
  }

  nlohmann::json seq_edges_json = nlohmann::json::array();
  nlohmann::json loop_edges_json = nlohmann::json::array();

  for (const auto& e : edges) {
    std::vector<double> info_vec;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        info_vec.push_back(e.information(r, c));
      }
    }
    nlohmann::json edge_item = {
        {"from", e.from_index},
        {"to", e.to_index},
        {"dx", e.dx},
        {"dy", e.dy},
        {"dyaw", e.dyaw},
        {"score", e.score},
        {"is_odom_fallback", e.is_odom_fallback},
        {"information", info_vec},
    };

    if (std::abs(e.to_index - e.from_index) == 1) {
      seq_edges_json.push_back(edge_item);
    } else {
      loop_edges_json.push_back(edge_item);
    }
  }

  root["metadata"] = {
      {"created_at", get_current_iso_time()},
      {"num_nodes", nodes.size()},
      {"num_sequential_edges", seq_edges_json.size()},
      {"num_loop_edges", loop_edges_json.size()},
      {"bag_path", bag_path.value_or("")},
  };
  root["nodes"] = nodes_json;
  root["sequential_edges"] = seq_edges_json;
  root["loop_edges"] = loop_edges_json;
  root["gnss_priors"] = nlohmann::json::array();

  std::ofstream fout(filepath);
  fout << root.dump(2) << std::endl;
  return filepath;
}

}  // namespace core
}  // namespace slam_gnss_2d
