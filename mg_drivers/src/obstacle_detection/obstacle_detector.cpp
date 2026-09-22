#include "obstacle_detection/obstacle_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace obstacle_detection
{

namespace
{

// グリッドセル数の上限（設定ミスによるメモリ枯渇の防止）
constexpr double kMaxGridCells = 4.0e6;

using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point & from, const Clock::time_point & to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}

}  // namespace

void ObstacleDetector::validate(const ObstacleDetectionParams & p)
{
  auto fail = [](const std::string & message) {
    throw std::invalid_argument("obstacle_detection params: " + message);
  };

  if (p.stride < 1) fail("stride must be >= 1");
  if (p.cropbox_x_min >= p.cropbox_x_max) fail("cropbox_x_min must be < cropbox_x_max");
  if (p.cropbox_y_min >= p.cropbox_y_max) fail("cropbox_y_min must be < cropbox_y_max");
  if (p.cropbox_z_min >= p.cropbox_z_max) fail("cropbox_z_min must be < cropbox_z_max");
  if (p.grid_size <= 0.0) fail("grid_size must be > 0");
  if (p.delta_z_threshold < 0.0) fail("delta_z_threshold must be >= 0");
  if (p.min_points_per_cell < 1) fail("min_points_per_cell must be >= 1");
  if (p.z_outlier_trim < 0 || p.z_outlier_trim > 1) fail("z_outlier_trim must be 0 or 1");
  if (p.cluster_tolerance <= 0.0) fail("cluster_tolerance must be > 0");
  if (p.min_cluster_cells < 1) fail("min_cluster_cells must be >= 1");

  const double cells = std::ceil((p.cropbox_x_max - p.cropbox_x_min) / p.grid_size) *
                       std::ceil((p.cropbox_y_max - p.cropbox_y_min) / p.grid_size);
  if (cells > kMaxGridCells) {
    fail("grid is too large (" + std::to_string(cells) + " cells); increase grid_size");
  }
}

ObstacleDetector::ObstacleDetector(const ObstacleDetectionParams & params) : params_(params)
{
  validate(params_);

  grid_width_ = static_cast<int>(
    std::ceil((params_.cropbox_x_max - params_.cropbox_x_min) / params_.grid_size));
  grid_height_ = static_cast<int>(
    std::ceil((params_.cropbox_y_max - params_.cropbox_y_min) / params_.grid_size));
  link_radius_ =
    std::max(1, static_cast<int>(std::floor(params_.cluster_tolerance / params_.grid_size + 1e-6)));
  min_points_needed_ = std::max(params_.min_points_per_cell, 2 * params_.z_outlier_trim + 2);

  const size_t num_cells = static_cast<size_t>(grid_width_) * grid_height_;
  grid_.assign(num_cells, GridCell{});
  labels_.assign(num_cells, kNone);
}

void ObstacleDetector::process(
  const PointCloudView & input, const Eigen::Isometry3f & sensor_to_base)
{
  stats_ = FrameStats{};
  stats_.input_points = input.num_points;
  obstacle_points_.clear();
  clusters_.clear();

  const auto t0 = Clock::now();
  accumulate(input, sensor_to_base);
  const auto t1 = Clock::now();
  judgeCells();
  const auto t2 = Clock::now();
  clusterCells();
  const auto t3 = Clock::now();
  extractPoints();
  const auto t4 = Clock::now();

  stats_.cropped_points = cropped_.size();
  stats_.output_points = obstacle_points_.size();
  stats_.num_clusters = clusters_.size();
  stats_.accumulate_ms = elapsedMs(t0, t1);
  stats_.judge_ms = elapsedMs(t1, t2);
  stats_.cluster_ms = elapsedMs(t2, t3);
  stats_.extract_ms = elapsedMs(t3, t4);
}

// 変換・ROI 判定・セルへの集計を 1 パスで行う
void ObstacleDetector::accumulate(
  const PointCloudView & input, const Eigen::Isometry3f & sensor_to_base)
{
  // 前フレームで使ったセルだけを初期化する
  for (const int idx : touched_cells_) {
    grid_[idx] = GridCell{};
    labels_[idx] = kNone;
  }
  touched_cells_.clear();
  candidate_cells_.clear();
  cropped_.clear();

  const Eigen::Matrix3f r = sensor_to_base.linear();
  const Eigen::Vector3f t = sensor_to_base.translation();
  const float x_min = static_cast<float>(params_.cropbox_x_min);
  const float x_max = static_cast<float>(params_.cropbox_x_max);
  const float y_min = static_cast<float>(params_.cropbox_y_min);
  const float y_max = static_cast<float>(params_.cropbox_y_max);
  const float z_min = static_cast<float>(params_.cropbox_z_min);
  const float z_max = static_cast<float>(params_.cropbox_z_max);
  const float inv_grid = static_cast<float>(1.0 / params_.grid_size);
  const float inf = std::numeric_limits<float>::infinity();

  for (size_t i = 0; i < input.num_points; i += params_.stride) {
    const uint8_t * base = input.data + i * input.point_step;
    float sx;
    float sy;
    float sz;
    std::memcpy(&sx, base + input.x_offset, sizeof(float));
    std::memcpy(&sy, base + input.y_offset, sizeof(float));
    std::memcpy(&sz, base + input.z_offset, sizeof(float));

    const float x = r(0, 0) * sx + r(0, 1) * sy + r(0, 2) * sz + t.x();
    const float y = r(1, 0) * sx + r(1, 1) * sy + r(1, 2) * sz + t.y();
    const float z = r(2, 0) * sx + r(2, 1) * sy + r(2, 2) * sz + t.z();

    // NaN / Inf は比較が偽になりここで除外される
    if (!(x >= x_min && x < x_max && y >= y_min && y < y_max && z >= z_min && z <= z_max)) {
      continue;
    }

    const int ix = std::min(static_cast<int>((x - x_min) * inv_grid), grid_width_ - 1);
    const int iy = std::min(static_cast<int>((y - y_min) * inv_grid), grid_height_ - 1);
    const int idx = ix + iy * grid_width_;

    GridCell & cell = grid_[idx];
    if (cell.count == 0) {
      cell.z_min1 = z;
      cell.z_min2 = inf;
      cell.z_max1 = z;
      cell.z_max2 = -inf;
      cell.x_min = x;
      cell.x_max = x;
      cell.y_min = y;
      cell.y_max = y;
      touched_cells_.push_back(idx);
    } else {
      if (z < cell.z_min1) {
        cell.z_min2 = cell.z_min1;
        cell.z_min1 = z;
      } else if (z < cell.z_min2) {
        cell.z_min2 = z;
      }
      if (z > cell.z_max1) {
        cell.z_max2 = cell.z_max1;
        cell.z_max1 = z;
      } else if (z > cell.z_max2) {
        cell.z_max2 = z;
      }
      cell.x_min = std::min(cell.x_min, x);
      cell.x_max = std::max(cell.x_max, x);
      cell.y_min = std::min(cell.y_min, y);
      cell.y_max = std::max(cell.y_max, y);
    }
    ++cell.count;

    cropped_.push_back({{x, y, z}, idx});
  }
}

// ΔZ が閾値を超えたセルを障害物セル候補にする
void ObstacleDetector::judgeCells()
{
  const float threshold = static_cast<float>(params_.delta_z_threshold);
  const bool trim = params_.z_outlier_trim > 0;

  for (const int idx : touched_cells_) {
    const GridCell & cell = grid_[idx];
    if (cell.count < min_points_needed_) continue;

    const float z_low = trim ? cell.z_min2 : cell.z_min1;
    const float z_high = trim ? cell.z_max2 : cell.z_max1;
    if (z_high - z_low > threshold) {
      labels_[idx] = kUnvisited;
      candidate_cells_.push_back(idx);
    }
  }
  stats_.candidate_cells = candidate_cells_.size();
}

// 障害物セル候補を 2D グリッド上の連結成分にまとめ、小さい成分を除去する
void ObstacleDetector::clusterCells()
{
  for (const int seed : candidate_cells_) {
    if (labels_[seed] != kUnvisited) continue;

    const int cluster_id = static_cast<int>(clusters_.size());
    ObstacleCluster cluster;
    cluster.min_x = cluster.min_y = cluster.min_z = std::numeric_limits<float>::max();
    cluster.max_x = cluster.max_y = cluster.max_z = std::numeric_limits<float>::lowest();

    component_.clear();
    stack_.clear();
    labels_[seed] = cluster_id;
    stack_.push_back(seed);

    while (!stack_.empty()) {
      const int idx = stack_.back();
      stack_.pop_back();
      component_.push_back(idx);

      const GridCell & cell = grid_[idx];
      cluster.min_x = std::min(cluster.min_x, cell.x_min);
      cluster.max_x = std::max(cluster.max_x, cell.x_max);
      cluster.min_y = std::min(cluster.min_y, cell.y_min);
      cluster.max_y = std::max(cluster.max_y, cell.y_max);
      cluster.min_z = std::min(cluster.min_z, cell.z_min1);
      cluster.max_z = std::max(cluster.max_z, cell.z_max1);

      const int ix = idx % grid_width_;
      const int iy = idx / grid_width_;
      const int x_begin = std::max(0, ix - link_radius_);
      const int x_end = std::min(grid_width_ - 1, ix + link_radius_);
      const int y_begin = std::max(0, iy - link_radius_);
      const int y_end = std::min(grid_height_ - 1, iy + link_radius_);
      for (int ny = y_begin; ny <= y_end; ++ny) {
        for (int nx = x_begin; nx <= x_end; ++nx) {
          const int n_idx = nx + ny * grid_width_;
          if (labels_[n_idx] == kUnvisited) {
            labels_[n_idx] = cluster_id;
            stack_.push_back(n_idx);
          }
        }
      }
    }

    if (static_cast<int>(component_.size()) < params_.min_cluster_cells) {
      for (const int idx : component_) labels_[idx] = kNone;
      continue;
    }
    cluster.num_cells = static_cast<int>(component_.size());
    clusters_.push_back(cluster);
  }
}

// 残ったクラスタに属するセルの点を出力する
void ObstacleDetector::extractPoints()
{
  for (const CroppedPoint & cp : cropped_) {
    const int label = labels_[cp.cell];
    if (label < 0) continue;
    obstacle_points_.push_back(cp.p);
    ++clusters_[label].num_points;
  }
}

}  // namespace obstacle_detection
