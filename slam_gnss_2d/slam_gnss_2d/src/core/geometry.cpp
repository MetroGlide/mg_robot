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
      double angle = beam_angle(scan, i);
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

std::vector<core::PoseNode> rebase_and_rotate_nodes(std::vector<core::PoseNode> nodes, double rot) {
  if (nodes.empty()) {
    return nodes;
  }
  const double origin_x = nodes.front().x;
  const double origin_y = nodes.front().y;
  const double c = std::cos(rot);
  const double s = std::sin(rot);
  for (auto& n : nodes) {
    const double dx = n.x - origin_x;
    const double dy = n.y - origin_y;
    n.x = c * dx - s * dy;
    n.y = s * dx + c * dy;
    n.yaw += rot;
  }
  return nodes;
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
scan_to_points_and_normals(const core::ScanData& scan) {
  std::vector<Eigen::Vector2d> pts;
  std::vector<int> beam_indices;
  pts.reserve(scan.ranges.size());
  beam_indices.reserve(scan.ranges.size());

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (r >= scan.range_min && r <= scan.range_max) {
      double angle = beam_angle(scan, i);
      double lx = r * std::cos(angle) + scan.lidar_x;
      double ly = r * std::sin(angle) + scan.lidar_y;
      pts.emplace_back(lx, ly);
      beam_indices.push_back(static_cast<int>(i));
    }
  }

  std::vector<Eigen::Vector2d> normals;
  normals.reserve(pts.size());

  const int n = static_cast<int>(pts.size());
  const double max_jump_dist_sq = 0.25 * 0.25;  // 25cm以上の距離跳躍は別物体とみなす

  for (int i = 0; i < n; ++i) {
    Eigen::Vector2d tangent(0.0, 0.0);
    bool found = false;

    // 前後2点までの有効な連続ビームを探す
    int prev_idx = -1;
    for (int step = 1; step <= 2; ++step) {
      int cand = i - step;
      if (cand >= 0 && (beam_indices[i] - beam_indices[cand] <= step + 1)) {
        if ((pts[i] - pts[cand]).squaredNorm() < max_jump_dist_sq * step * step) {
          prev_idx = cand;
          break;
        }
      }
    }

    int next_idx = -1;
    for (int step = 1; step <= 2; ++step) {
      int cand = i + step;
      if (cand < n && (beam_indices[cand] - beam_indices[i] <= step + 1)) {
        if ((pts[cand] - pts[i]).squaredNorm() < max_jump_dist_sq * step * step) {
          next_idx = cand;
          break;
        }
      }
    }

    if (prev_idx >= 0 && next_idx >= 0) {
      tangent = pts[next_idx] - pts[prev_idx];
      found = true;
    } else if (next_idx >= 0) {
      tangent = pts[next_idx] - pts[i];
      found = true;
    } else if (prev_idx >= 0) {
      tangent = pts[i] - pts[prev_idx];
      found = true;
    }

    if (found && tangent.squaredNorm() > 1e-8) {
      Eigen::Vector2d normal(-tangent.y(), tangent.x());
      normal.normalize();

      // センサー原点方向を向くように法線の向きを統一
      Eigen::Vector2d view(scan.lidar_x - pts[i].x(), scan.lidar_y - pts[i].y());
      if (normal.dot(view) < 0.0) {
        normal = -normal;
      }
      normals.push_back(normal);
    } else {
      // フォールバック法線（視線方向）
      Eigen::Vector2d view(scan.lidar_x - pts[i].x(), scan.lidar_y - pts[i].y());
      double norm = view.norm();
      if (norm > 1e-4) {
        normals.push_back(view / norm);
      } else {
        normals.push_back(Eigen::Vector2d(0.0, 1.0));
      }
    }
  }

  return {pts, normals};
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
scan_to_points_and_normals(const core::ConstScanDataPtr& scan) {
  if (!scan) return {{}, {}};
  return scan_to_points_and_normals(*scan);
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>>
scan_to_points_and_normals(const core::ScanDataPtr& scan) {
  if (!scan) return {{}, {}};
  return scan_to_points_and_normals(*scan);
}

std::vector<Eigen::Vector2d> normals_local_to_world(
    const std::vector<Eigen::Vector2d>& local_normals, double yaw) {
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  std::vector<Eigen::Vector2d> world_normals;
  world_normals.reserve(local_normals.size());

  for (const auto& n : local_normals) {
    double wx = cos_yaw * n.x() - sin_yaw * n.y();
    double wy = sin_yaw * n.x() + cos_yaw * n.y();
    world_normals.emplace_back(wx, wy);
  }
  return world_normals;
}

std::vector<Eigen::Vector2d> normals_world_to_local(
    const std::vector<Eigen::Vector2d>& world_normals, double yaw) {
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);
  std::vector<Eigen::Vector2d> local_normals;
  local_normals.reserve(world_normals.size());

  for (const auto& n : world_normals) {
    double lx = cos_yaw * n.x() + sin_yaw * n.y();
    double ly = -sin_yaw * n.x() + cos_yaw * n.y();
    local_normals.emplace_back(lx, ly);
  }
  return local_normals;
}

}  // namespace slam_gnss_2d
