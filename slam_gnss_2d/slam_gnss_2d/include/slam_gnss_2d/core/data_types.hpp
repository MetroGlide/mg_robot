#pragma once

#include <Eigen/Core>
#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace slam_gnss_2d {
namespace core {

struct SubmapPatch {
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  cv::Mat hit_patch;
  cv::Mat miss_patch;
};

using SubmapPatchPtr = std::shared_ptr<SubmapPatch>;
using ConstSubmapPatchPtr = std::shared_ptr<const SubmapPatch>;

struct ScanData {
  double timestamp{0.0};
  std::vector<float> ranges;
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.1};
  double range_max{30.0};
  double lidar_x{0.0};
  double lidar_y{0.0};
  double lidar_yaw{0.0};
};

using ScanDataPtr = std::shared_ptr<ScanData>;
using ConstScanDataPtr = std::shared_ptr<const ScanData>;

struct OdomData {
  double timestamp{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GnssData {
  double timestamp{0.0};
  double x{0.0};
  double y{0.0};
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Zero()};
  int fix_status{-1};
  double latitude{0.0};
  double longitude{0.0};
};

struct SensorFrame {
  ScanDataPtr scan{nullptr};
  OdomData odom;
  std::optional<GnssData> gnss{std::nullopt};
};

struct PoseNode {
  int index{0};
  double timestamp{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  ConstScanDataPtr scan{nullptr};
  SubmapPatchPtr submap_patch{nullptr};
};

struct MatchResult {
  double dx{0.0};
  double dy{0.0};
  double dyaw{0.0};
  bool converged{false};
  Eigen::Matrix3d information{Eigen::Matrix3d::Zero()};
  double score{0.0};

  bool success() const { return converged; }
};

struct PoseEdge {
  int from_index{0};
  int to_index{0};
  double dx{0.0};
  double dy{0.0};
  double dyaw{0.0};
  Eigen::Matrix3d information{Eigen::Matrix3d::Zero()};
  double score{0.0};
  bool is_odom_fallback{false};
};

struct GnssPrior {
  int node_index{0};
  double x{0.0};
  double y{0.0};
  Eigen::Matrix2d information{Eigen::Matrix2d::Zero()};
};

struct ScanProcessResult {
  std::optional<PoseNode> node{std::nullopt};
  bool loop_closed{false};
  bool rerender_required{false};
  std::optional<PoseEdge> new_seq_edge{std::nullopt};
  std::vector<PoseEdge> new_loop_edges;
  std::optional<GnssPrior> new_gnss_prior{std::nullopt};
};

}  // namespace core

// Alias in outer namespace
using core::ScanData;
using core::ScanDataPtr;
using core::ConstScanDataPtr;
using core::OdomData;
using core::GnssData;
using core::SensorFrame;
using core::PoseNode;
using core::MatchResult;
using core::PoseEdge;
using core::GnssPrior;
using core::ScanProcessResult;

}  // namespace slam_gnss_2d
