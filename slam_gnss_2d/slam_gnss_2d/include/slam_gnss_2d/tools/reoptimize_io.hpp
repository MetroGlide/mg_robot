#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace slam_gnss_2d {
namespace tools {

std::pair<nlohmann::json, YAML::Node> load_pose_graph_and_transform(
    const std::string& input_dir);

std::string resolve_bag_path(
    const std::string& configured_bag_path,
    const nlohmann::json& pose_graph_data);

}  // namespace tools
}  // namespace slam_gnss_2d
