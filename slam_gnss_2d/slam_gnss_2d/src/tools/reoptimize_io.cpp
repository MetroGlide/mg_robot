#include "slam_gnss_2d/tools/reoptimize_io.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace slam_gnss_2d {
namespace tools {

std::pair<nlohmann::json, YAML::Node> load_pose_graph_and_transform(
    const std::string& input_dir) {
  auto pg_path = std::filesystem::path(input_dir) / "pose_graph.json";
  auto gt_path = std::filesystem::path(input_dir) / "gnss_transform.yaml";

  if (!std::filesystem::exists(pg_path)) {
    throw std::runtime_error("pose_graph.json not found in " + input_dir);
  }
  if (!std::filesystem::exists(gt_path)) {
    throw std::runtime_error("gnss_transform.yaml not found in " + input_dir);
  }

  std::ifstream pg_file(pg_path.string());
  nlohmann::json pg_data;
  pg_file >> pg_data;

  YAML::Node gt_data = YAML::LoadFile(gt_path.string());

  return {pg_data, gt_data};
}

std::string resolve_bag_path(
    const std::string& configured_bag_path,
    const nlohmann::json& pose_graph_data) {
  std::string bag_path = configured_bag_path;
  if (bag_path.empty() && pose_graph_data.contains("metadata")) {
    bag_path = pose_graph_data["metadata"].value("bag_path", "");
  }

  if (bag_path.empty() || !std::filesystem::exists(bag_path)) {
    throw std::runtime_error("ROS Bag path '" + bag_path + "' is invalid or file does not exist");
  }

  return bag_path;
}

}  // namespace tools
}  // namespace slam_gnss_2d
