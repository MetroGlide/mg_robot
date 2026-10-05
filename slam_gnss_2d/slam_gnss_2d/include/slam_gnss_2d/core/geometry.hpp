#pragma once

#include <Eigen/Core>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tuple>
#include <utility>
#include <vector>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {

double angle_diff(double a, double b);
double normalize_angle(double angle);
double quaternion_to_yaw(double x, double y, double z, double w);
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

std::tuple<double, double, double> compose_pose(
    double x1, double y1, double yaw1, double dx, double dy, double dyaw);
std::tuple<double, double, double> delta_pose(
    double x1, double y1, double yaw1, double x2, double y2, double yaw2);

std::vector<Eigen::Vector2d> scan_to_points(const core::ScanData& scan);
std::vector<Eigen::Vector2d> scan_to_points(const core::ConstScanDataPtr& scan);
std::vector<Eigen::Vector2d> scan_to_points(const core::ScanDataPtr& scan);

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
scan_to_points_and_normals(const core::ScanData& scan);
std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
scan_to_points_and_normals(const core::ConstScanDataPtr& scan);
std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
scan_to_points_and_normals(const core::ScanDataPtr& scan);

std::pair<double, double> world_delta_to_local(double dx_w, double dy_w, double reference_yaw);
std::pair<double, double> local_delta_to_world(double dx_local, double dy_local, double reference_yaw);

std::vector<Eigen::Vector2d> points_local_to_world(
    const std::vector<Eigen::Vector2d>& local_pts, double origin_x, double origin_y, double yaw);

std::vector<Eigen::Vector2d> points_world_to_local(
    const std::vector<Eigen::Vector2d>& world_pts, double origin_x, double origin_y, double yaw);

// 先頭ノードを原点 (0, 0) に移し、全ノードを原点まわりに rot [rad] だけ回転した新しいノード列を返す
// (各ノードの yaw にも rot を加える)。先頭ノード自身も原点 (0, 0) に移る。
std::vector<core::PoseNode> rebase_and_rotate_nodes(std::vector<core::PoseNode> nodes, double rot);

std::vector<Eigen::Vector2d> normals_local_to_world(
    const std::vector<Eigen::Vector2d>& local_normals, double yaw);

std::vector<Eigen::Vector2d> normals_world_to_local(
    const std::vector<Eigen::Vector2d>& world_normals, double yaw);

namespace core {
using slam_gnss_2d::angle_diff;
using slam_gnss_2d::normalize_angle;
using slam_gnss_2d::quaternion_to_yaw;
using slam_gnss_2d::yaw_to_quaternion;
using slam_gnss_2d::compose_pose;
using slam_gnss_2d::delta_pose;
using slam_gnss_2d::scan_to_points;
using slam_gnss_2d::world_delta_to_local;
using slam_gnss_2d::local_delta_to_world;
using slam_gnss_2d::points_local_to_world;
using slam_gnss_2d::points_world_to_local;
using slam_gnss_2d::rebase_and_rotate_nodes;
using slam_gnss_2d::scan_to_points_and_normals;
using slam_gnss_2d::normals_local_to_world;
using slam_gnss_2d::normals_world_to_local;
}  // namespace core

}  // namespace slam_gnss_2d
