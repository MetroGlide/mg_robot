#include "slam_gnss_2d/ros/map_save_service.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "slam_gnss_2d/core/slam_data_saver.hpp"

namespace slam_gnss_2d {
namespace ros {

MapSaveService::MapSaveService(
    rclcpp::Node::SharedPtr node,
    pose_graph::PoseGraphBuilderPtr pose_graph,
    core::GraphOrchestratorPtr orchestrator)
    : node_(node), pose_graph_(pose_graph), orchestrator_(orchestrator) {
  save_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "slam_gnss_2d/save_slam_map",
      [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        this->handle_save(request, response);
      });
}

void MapSaveService::handle_save(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::string output_dir = "/root/ros2_data/slam_maps/latest";
  if (node_->has_parameter("save_dir")) {
    auto val = node_->get_parameter("save_dir").as_string();
    if (!val.empty()) {
      output_dir = val;
    }
  }

  try {
    auto nodes = pose_graph_->get_nodes();
    auto edges = pose_graph_->get_edges();

    std::optional<std::string> bag_path = std::nullopt;
    if (node_->has_parameter("bag_path")) {
      auto val = node_->get_parameter("bag_path").as_string();
      if (!val.empty()) {
        bag_path = val;
      }
    }

    std::string pg_path = core::SlamDataSaver::save_pose_graph(
        output_dir, nodes, edges, bag_path);
    std::string message_parts = "PoseGraph saved: " + pg_path;

    auto anchor_latlon = orchestrator_->anchor_latlon();
    if (anchor_latlon.has_value()) {
      auto [lat, lon] = *anchor_latlon;
      auto anchor_utm = orchestrator_->anchor_utm();
      double easting = anchor_utm ? anchor_utm->first : 0.0;
      double northing = anchor_utm ? anchor_utm->second : 0.0;
      int zone = static_cast<int>(std::floor((lon + 180.0) / 6.0)) + 1;
      std::string hemisphere = (lat >= 0.0) ? "north" : "south";
      double rotation_rad = orchestrator_->init_rotation().value_or(0.0);

      std::string backend = "gtsam";
      if (node_->has_parameter("optimization.backend")) {
        backend = node_->get_parameter("optimization.backend").as_string();
      }

      std::string gnss_path = core::SlamDataSaver::save_gnss_transform(
          output_dir, lat, lon, easting, northing, zone, hemisphere, rotation_rad, backend);
      message_parts += "; GNSS transform saved: " + gnss_path;
    }

    response->success = true;
    response->message = message_parts;
    RCLCPP_INFO(node_->get_logger(), "SLAM map saved successfully: %s", response->message.c_str());
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Failed to save SLAM map: ") + e.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
  }
}

}  // namespace ros
}  // namespace slam_gnss_2d
