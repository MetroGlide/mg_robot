#include "slam_gnss_2d/scan_matching/ndt_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

namespace {

constexpr int kMinCorrespondences = 5;
constexpr double kExponentCutoff = -3.0;

inline int64_t encode_cell_key(int32_t cx, int32_t cy) {
  return (static_cast<int64_t>(cx) << 32) | (static_cast<uint32_t>(cy) & 0xFFFFFFFFULL);
}

}  // namespace

NDTMatcher::NDTMatcher(
    int max_iterations,
    double tolerance,
    const std::vector<double>& cell_sizes,
    bool use_bilinear,
    double yaw_information_multiplier)
    : max_iterations_(max_iterations),
      tolerance_(tolerance),
      cell_sizes_(cell_sizes),
      use_bilinear_(use_bilinear),
      yaw_information_multiplier_(yaw_information_multiplier) {
  std::sort(cell_sizes_.begin(), cell_sizes_.end(), std::greater<double>());
}

void NDTMatcher::set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) {
  src_pts_ = src_pts;
  pyramids_.clear();

  for (double cs : cell_sizes_) {
    PyramidLevel level;
    level.cell_size = cs;
    double inv_cs = 1.0 / cs;

    std::unordered_map<int64_t, std::vector<Eigen::Vector2d>> cell_points;
    for (const auto& pt : src_pts_) {
      int32_t cx = static_cast<int32_t>(std::floor(pt.x() * inv_cs));
      int32_t cy = static_cast<int32_t>(std::floor(pt.y() * inv_cs));
      int64_t key = encode_cell_key(cx, cy);
      cell_points[key].push_back(pt);
    }

    for (const auto& kv : cell_points) {
      const auto& pts = kv.second;
      if (pts.size() < 3) {
        continue;
      }
      Eigen::Vector2d mean = Eigen::Vector2d::Zero();
      for (const auto& p : pts) {
        mean += p;
      }
      mean /= static_cast<double>(pts.size());

      Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
      for (const auto& p : pts) {
        Eigen::Vector2d diff = p - mean;
        cov += diff * diff.transpose();
      }
      cov /= static_cast<double>(pts.size() - 1);
      cov(0, 0) += 1e-3;
      cov(1, 1) += 1e-3;

      Eigen::Matrix2d sigma_inv = cov.inverse();
      level.cells[kv.first] = NdtCell{mean, sigma_inv};
    }

    pyramids_.push_back(std::move(level));
  }
}

core::MatchResult NDTMatcher::match_single_resolution(
    const std::vector<Eigen::Vector2d>& dst_pts,
    double init_tx, double init_ty, double init_theta,
    const PyramidLevel& level) {
  if (level.cells.empty() || dst_pts.size() < kMinCorrespondences) {
    return core::MatchResult{
        init_tx, init_ty, init_theta, false, Eigen::Matrix3d::Zero(), 0.0};
  }

  double tx = init_tx;
  double ty = init_ty;
  double theta = init_theta;
  double inv_cs = 1.0 / level.cell_size;

  Eigen::Matrix3d H_final = Eigen::Matrix3d::Zero();
  int n_valid_final = 0;
  bool converged = false;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    double c = std::cos(theta);
    double s = std::sin(theta);

    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    int n_valid = 0;

    for (const auto& p_orig : dst_pts) {
      double px = c * p_orig.x() - s * p_orig.y() + tx;
      double py = s * p_orig.x() + c * p_orig.y() + ty;

      int32_t cx = static_cast<int32_t>(std::floor(px * inv_cs));
      int32_t cy = static_cast<int32_t>(std::floor(py * inv_cs));
      int64_t key = encode_cell_key(cx, cy);

      auto it = level.cells.find(key);
      if (it == level.cells.end()) {
        continue;
      }

      const auto& cell = it->second;
      Eigen::Vector2d d(px - cell.mean.x(), py - cell.mean.y());
      double exponent = -0.5 * d.dot(cell.sigma_inv * d);
      if (exponent < kExponentCutoff) {
        continue;
      }

      double exp_val = std::exp(exponent);
      n_valid++;

      Eigen::Vector2d dp_dtheta(-s * p_orig.x() - c * p_orig.y(),
                                c * p_orig.x() - s * p_orig.y());

      Eigen::Matrix<double, 2, 3> J;
      J(0, 0) = 1.0;
      J(0, 1) = 0.0;
      J(0, 2) = dp_dtheta.x();
      J(1, 0) = 0.0;
      J(1, 1) = 1.0;
      J(1, 2) = dp_dtheta.y();

      Eigen::Vector2d sigma_d = cell.sigma_inv * d;
      g += exp_val * (J.transpose() * sigma_d);
      H += exp_val * (J.transpose() * cell.sigma_inv * J);
    }

    if (n_valid < kMinCorrespondences) {
      break;
    }

    H_final = H;
    n_valid_final = n_valid;

    Eigen::Matrix3d H_reg = H + Eigen::Matrix3d::Identity() * 1e-6;
    Eigen::Vector3d delta = H_reg.ldlt().solve(-g);

    tx += delta.x();
    ty += delta.y();
    theta += delta.z();

    if (delta.norm() < tolerance_) {
      converged = true;
      break;
    }
  }

  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  if (n_valid_final > 0) {
    double info_scale = 400.0;
    information = (H_final / static_cast<double>(n_valid_final)) * info_scale +
                  Eigen::Matrix3d::Identity() * 1e-4;
    for (int r = 0; r < 2; ++r) {
      for (int col = 0; col < 2; ++col) {
        information(r, col) = std::clamp(information(r, col), -1000.0, 1000.0);
      }
    }
    information(2, 2) = std::clamp(information(2, 2), 0.0, 5000.0) * yaw_information_multiplier_;
  }

  double score = 0.0;
  if (converged) {
    double c_f = std::cos(theta);
    double s_f = std::sin(theta);
    double score_sum = 0.0;
    int score_count = 0;
    for (const auto& p_orig : dst_pts) {
      double px = c_f * p_orig.x() - s_f * p_orig.y() + tx;
      double py = s_f * p_orig.x() + c_f * p_orig.y() + ty;
      int32_t cx = static_cast<int32_t>(std::floor(px * inv_cs));
      int32_t cy = static_cast<int32_t>(std::floor(py * inv_cs));
      int64_t key = encode_cell_key(cx, cy);

      auto it = level.cells.find(key);
      if (it != level.cells.end()) {
        Eigen::Vector2d d(px - it->second.mean.x(), py - it->second.mean.y());
        double exp_f = -0.5 * d.dot(it->second.sigma_inv * d);
        if (exp_f >= kExponentCutoff) {
          score_sum += -exp_f;
          score_count++;
        }
      }
    }
    if (score_count >= kMinCorrespondences) {
      score = score_sum / static_cast<double>(score_count);
    }
  }

  return core::MatchResult{tx, ty, theta, converged, information, score};
}

core::MatchResult NDTMatcher::match(
    const core::ConstScanDataPtr& dst,
    const core::OdomData& initial_guess) {
  if (src_pts_.empty() || pyramids_.empty() || !dst) {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }

  std::vector<Eigen::Vector2d> dst_pts = core::scan_to_points(dst);
  if (static_cast<int>(dst_pts.size()) < kMinCorrespondences) {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }

  double tx = initial_guess.x;
  double ty = initial_guess.y;
  double theta = initial_guess.yaw;

  bool final_converged = false;
  Eigen::Matrix3d final_info = Eigen::Matrix3d::Zero();
  double final_score = 0.0;

  for (size_t i = 0; i < pyramids_.size(); ++i) {
    const auto& level = pyramids_[i];
    auto res = match_single_resolution(dst_pts, tx, ty, theta, level);
    tx = res.dx;
    ty = res.dy;
    theta = res.dyaw;
    final_converged = res.converged;
    final_info = res.information;
    final_score = res.score;

    if (!res.converged && i == 0) {
      break;
    }
  }

  return core::MatchResult{
      tx, ty, theta, final_converged, final_info, final_score};
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
