#include "slam_gnss_2d/core/geometry.hpp"

#include <cmath>

namespace slam_gnss_2d {

double angle_diff(double a, double b) {
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double quaternion_to_yaw(double x, double y, double z, double w) {
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

std::tuple<double, double, double> compose_pose(
    double x1, double y1, double yaw1, double dx, double dy, double dyaw) {
  double cy = std::cos(yaw1);
  double sy = std::sin(yaw1);
  double x2 = x1 + cy * dx - sy * dy;
  double y2 = y1 + sy * dx + cy * dy;
  double yaw2 = normalize_angle(yaw1 + dyaw);
  return {x2, y2, yaw2};
}

std::tuple<double, double, double> delta_pose(
    double x1, double y1, double yaw1, double x2, double y2, double yaw2) {
  double dx_w = x2 - x1;
  double dy_w = y2 - y1;
  double cy = std::cos(yaw1);
  double sy = std::sin(yaw1);
  double dx = cy * dx_w + sy * dy_w;
  double dy = -sy * dx_w + cy * dy_w;
  double dyaw = normalize_angle(yaw2 - yaw1);
  return {dx, dy, dyaw};
}

std::vector<Eigen::Vector2d> scan_to_points(const core::ScanData& scan) {
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (r >= scan.range_min && r <= scan.range_max) {
      double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      double lx = r * std::cos(angle) + scan.lidar_x;
      double ly = r * std::sin(angle) + scan.lidar_y;
      pts.emplace_back(lx, ly);
    }
  }
  return pts;
}

std::vector<Eigen::Vector2d> scan_to_points(const core::ConstScanDataPtr& scan) {
  if (!scan) return {};
  return scan_to_points(*scan);
}

std::vector<Eigen::Vector2d> scan_to_points(const core::ScanDataPtr& scan) {
  if (!scan) return {};
  return scan_to_points(*scan);
}

std::pair<double, double> world_delta_to_local(double dx_w, double dy_w, double reference_yaw) {
  double cos_yaw = std::cos(-reference_yaw);
  double sin_yaw = std::sin(-reference_yaw);
  return {cos_yaw * dx_w - sin_yaw * dy_w, sin_yaw * dx_w + cos_yaw * dy_w};
}

std::pair<double, double> local_delta_to_world(double dx_local, double dy_local, double reference_yaw) {
  double cos_yaw = std::cos(reference_yaw);
  double sin_yaw = std::sin(reference_yaw);
  return {cos_yaw * dx_local - sin_yaw * dy_local, sin_yaw * dx_local + cos_yaw * dy_local};
}

std::vector<Eigen::Vector2d> points_local_to_world(
    const std::vector<Eigen::Vector2d>& local_pts, double origin_x, double origin_y, double yaw) {
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  std::vector<Eigen::Vector2d> world_pts;
  world_pts.reserve(local_pts.size());

  for (const auto& pt : local_pts) {
    double wx = cos_yaw * pt.x() - sin_yaw * pt.y() + origin_x;
    double wy = sin_yaw * pt.x() + cos_yaw * pt.y() + origin_y;
    world_pts.emplace_back(wx, wy);
  }
  return world_pts;
}

std::vector<Eigen::Vector2d> points_world_to_local(
    const std::vector<Eigen::Vector2d>& world_pts, double origin_x, double origin_y, double yaw) {
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  std::vector<Eigen::Vector2d> local_pts;
  local_pts.reserve(world_pts.size());

  for (const auto& pt : world_pts) {
    double dx = pt.x() - origin_x;
    double dy = pt.y() - origin_y;
    double lx = cos_yaw * dx + sin_yaw * dy;
    double ly = -sin_yaw * dx + cos_yaw * dy;
    local_pts.emplace_back(lx, ly);
  }
  return local_pts;
}

}  // namespace slam_gnss_2d
