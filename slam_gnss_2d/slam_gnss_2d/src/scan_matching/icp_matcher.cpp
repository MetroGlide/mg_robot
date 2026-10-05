#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <nanoflann.hpp>
#include <omp.h>

#ifndef _OPENMP
#error "OpenMP is NOT enabled! Ensure -fopenmp is passed to compiler."
#endif

#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

namespace {

constexpr int kMinCorrespondences = 10;
constexpr int kNormalNeighbors = 5;

struct PointCloud2DAdaptor {
  const std::vector<Eigen::Vector2d>& pts;
  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return (dim == 0) ? pts[idx].x() : pts[idx].y();
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const { return false; }
};

using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud2DAdaptor>,
    PointCloud2DAdaptor, 2>;

std::vector<Eigen::Vector2d> apply_transform(
    const std::vector<Eigen::Vector2d>& pts,
    double tx, double ty, double theta) {
  double c = std::cos(theta);
  double s = std::sin(theta);
  std::vector<Eigen::Vector2d> out(pts.size());
  #pragma omp parallel for schedule(static)
  for (size_t i = 0; i < pts.size(); ++i) {
    const auto& p = pts[i];
    out[i] = Eigen::Vector2d(c * p.x() - s * p.y() + tx, s * p.x() + c * p.y() + ty);
  }
  return out;
}

std::vector<Eigen::Vector2d> estimate_all_normals(
    const std::vector<Eigen::Vector2d>& pts,
    const KDTree2D& tree,
    int k = kNormalNeighbors) {
  std::vector<Eigen::Vector2d> normals(pts.size());

  #pragma omp parallel for schedule(dynamic, 64)
  for (size_t i = 0; i < pts.size(); ++i) {
    std::vector<uint32_t> indices(k);
    std::vector<double> dist_sqs(k);
    double query_pt[2] = {pts[i].x(), pts[i].y()};
    tree.knnSearch(query_pt, k, &indices[0], &dist_sqs[0]);

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (int j = 0; j < k; ++j) {
      mean += pts[indices[j]];
    }
    mean /= static_cast<double>(k);

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (int j = 0; j < k; ++j) {
      Eigen::Vector2d diff = pts[indices[j]] - mean;
      cov += diff * diff.transpose();
    }
    cov /= static_cast<double>(std::max(k - 1, 1));

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov);
    if (eigensolver.info() == Eigen::Success) {
      normals[i] = eigensolver.eigenvectors().col(0).normalized();
    } else {
      normals[i] = Eigen::Vector2d(0.0, 1.0);
    }
  }
  return normals;
}

}  // namespace

struct ICPMatcher::Impl {
  std::unique_ptr<PointCloud2DAdaptor> adaptor;
  std::unique_ptr<KDTree2D> tree;
};

ICPMatcher::ICPMatcher(
    int max_iterations,
    double tolerance,
    double max_correspondence_dist,
    const std::string& robust_kernel,
    double robust_kernel_scale,
    double yaw_information_multiplier,
    double motion_prior_weight_x,
    double motion_prior_weight_y,
    double motion_prior_weight_yaw,
    double tolerance_trans,
    double tolerance_rot)
    : max_iterations_(max_iterations),
      tolerance_(tolerance),
      tolerance_trans_(tolerance_trans > 0.0 ? tolerance_trans : tolerance),
      tolerance_rot_(tolerance_rot > 0.0 ? tolerance_rot : tolerance),
      max_correspondence_dist_(max_correspondence_dist),
      robust_kernel_(robust_kernel),
      robust_kernel_scale_(robust_kernel_scale),
      yaw_information_multiplier_(yaw_information_multiplier),
      motion_prior_weight_x_(motion_prior_weight_x),
      motion_prior_weight_y_(motion_prior_weight_y),
      motion_prior_weight_yaw_(motion_prior_weight_yaw),
      impl_(std::make_unique<Impl>()) {}

ICPMatcher::~ICPMatcher() = default;

void ICPMatcher::set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) {
  src_pts_ = src_pts;
  if (static_cast<int>(src_pts_.size()) >= kNormalNeighbors) {
    impl_->adaptor = std::make_unique<PointCloud2DAdaptor>(PointCloud2DAdaptor{src_pts_});
    impl_->tree = std::make_unique<KDTree2D>(
        2, *impl_->adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    impl_->tree->buildIndex();
    src_normals_ = estimate_all_normals(src_pts_, *impl_->tree, kNormalNeighbors);
  } else {
    impl_->tree.reset();
    impl_->adaptor.reset();
    src_normals_.clear();
  }
}

void ICPMatcher::set_target_cloud_with_normals(
    const std::vector<Eigen::Vector2d>& src_pts,
    const std::vector<Eigen::Vector2d>& src_normals) {
  src_pts_ = src_pts;
  src_normals_ = src_normals;
  if (static_cast<int>(src_pts_.size()) >= kNormalNeighbors &&
      src_pts_.size() == src_normals_.size()) {
    impl_->adaptor = std::make_unique<PointCloud2DAdaptor>(PointCloud2DAdaptor{src_pts_});
    impl_->tree = std::make_unique<KDTree2D>(
        2, *impl_->adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    impl_->tree->buildIndex();
  } else {
    set_target_cloud(src_pts);
  }
}

core::MatchResult ICPMatcher::match(
    const core::ConstScanDataPtr& dst,
    const core::OdomData& initial_guess) {
  if (!dst) {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }
  std::vector<Eigen::Vector2d> dst_pts = core::scan_to_points(dst);
  return match(dst_pts, initial_guess);
}

core::MatchResult ICPMatcher::match(
    const std::vector<Eigen::Vector2d>& dst_pts,
    const core::OdomData& initial_guess) {
  if (src_pts_.empty() || !impl_->tree || src_normals_.empty()) {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }

  if (static_cast<int>(dst_pts.size()) < kMinCorrespondences) {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }

  double tx = initial_guess.x;
  double ty = initial_guess.y;
  double theta = initial_guess.yaw;
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  bool converged = false;
  int last_valid_count = 0;
  double max_dist_sq = max_correspondence_dist_ * max_correspondence_dist_;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    std::vector<Eigen::Vector2d> p_trans = apply_transform(dst_pts, tx, ty, theta);

    int num_threads = omp_get_max_threads();
    std::vector<Eigen::Matrix3d> thread_H(num_threads, Eigen::Matrix3d::Zero());
    std::vector<Eigen::Vector3d> thread_b(num_threads, Eigen::Vector3d::Zero());
    std::vector<int> thread_valid(num_threads, 0);

    #pragma omp parallel
    {
      int tid = omp_get_thread_num();
      Eigen::Matrix3d& H_local = thread_H[tid];
      Eigen::Vector3d& b_local = thread_b[tid];
      int& count_local = thread_valid[tid];

      #pragma omp for schedule(static)
      for (size_t i = 0; i < p_trans.size(); ++i) {
        double query_pt[2] = {p_trans[i].x(), p_trans[i].y()};
        uint32_t ret_idx = 0;
        double out_dist_sq = 0.0;
        if (impl_->tree->knnSearch(query_pt, 1, &ret_idx, &out_dist_sq) > 0) {
          if (out_dist_sq < max_dist_sq) {
            const auto& p_curr = p_trans[i];
            const auto& q = src_pts_[ret_idx];
            const auto& n = src_normals_[ret_idx];

            double rpx = p_curr.x() - tx;
            double rpy = p_curr.y() - ty;
            Eigen::Vector3d J(n.x(), n.y(), -n.x() * rpy + n.y() * rpx);
            double r = n.x() * (p_curr.x() - q.x()) + n.y() * (p_curr.y() - q.y());

            double w = 1.0;
            if (robust_kernel_ == "huber") {
              double abs_r = std::abs(r);
              if (abs_r > robust_kernel_scale_) {
                w = robust_kernel_scale_ / abs_r;
              }
            } else if (robust_kernel_ == "cauchy") {
              double s = r / robust_kernel_scale_;
              w = 1.0 / (1.0 + s * s);
            }

            H_local += (w * J) * J.transpose();
            b_local += (w * J) * r;
            count_local++;
          }
        }
      }
    }

    // スレッド順序でリダクションし完全な決定論性を保証
    H.setZero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    last_valid_count = 0;
    for (int t = 0; t < num_threads; ++t) {
      H += thread_H[t];
      b += thread_b[t];
      last_valid_count += thread_valid[t];
    }

    if (last_valid_count < kMinCorrespondences) {
      return core::MatchResult{
          tx, ty, theta, false, Eigen::Matrix3d::Zero(), 0.0};
    }

    Eigen::Matrix3d W_motion = Eigen::Matrix3d::Zero();
    W_motion(0, 0) = motion_prior_weight_x_;
    W_motion(1, 1) = motion_prior_weight_y_;
    W_motion(2, 2) = motion_prior_weight_yaw_;

    double dyaw_motion = core::angle_diff(theta, initial_guess.yaw);
    Eigen::Vector3d err_motion(tx - initial_guess.x, ty - initial_guess.y, dyaw_motion);

    Eigen::Matrix3d H_reg = H + W_motion + Eigen::Matrix3d::Identity() * 1e-4;
    Eigen::Vector3d b_reg = b + W_motion * err_motion;
    Eigen::Vector3d delta = H_reg.ldlt().solve(-b_reg);

    tx += delta.x();
    ty += delta.y();
    theta += delta.z();

    double delta_trans = std::hypot(delta.x(), delta.y());
    double delta_rot = std::abs(delta.z());
    if (delta_trans < tolerance_trans_ && delta_rot < tolerance_rot_) {
      converged = true;
      break;
    }
  }

  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  if (last_valid_count > 0) {
    double info_scale = 400.0;
    Eigen::Matrix3d norm_H = (H / static_cast<double>(last_valid_count)) * info_scale;

    // 並進 2x2 ブロックの幾何異方性を保持しつつ正則化
    Eigen::Matrix2d H_trans = norm_H.block<2, 2>(0, 0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(H_trans);
    if (es.info() == Eigen::Success) {
      Eigen::Vector2d vals = es.eigenvalues();
      vals(0) = std::clamp(vals(0), 20.0, 1000.0);
      vals(1) = std::clamp(vals(1), 20.0, 1000.0);
      H_trans = es.eigenvectors() * vals.asDiagonal() * es.eigenvectors().transpose();
    } else {
      H_trans(0, 0) = std::clamp(H_trans(0, 0), 20.0, 1000.0);
      H_trans(1, 1) = std::clamp(H_trans(1, 1), 20.0, 1000.0);
    }
    information.block<2, 2>(0, 0) = H_trans;

    // 回転成分
    double rot_info = std::clamp(norm_H(2, 2), 20.0, 5000.0) * yaw_information_multiplier_;
    information(2, 2) = rot_info;
  }

  double score = 0.0;
  if (converged) {
    std::vector<Eigen::Vector2d> p_final = apply_transform(dst_pts, tx, ty, theta);
    int num_threads = omp_get_max_threads();
    std::vector<double> thread_cost(num_threads, 0.0);
    std::vector<int> thread_valid_f(num_threads, 0);

    #pragma omp parallel
    {
      int tid = omp_get_thread_num();
      double& cost_local = thread_cost[tid];
      int& valid_local = thread_valid_f[tid];

      #pragma omp for schedule(static)
      for (size_t i = 0; i < p_final.size(); ++i) {
        double query_pt[2] = {p_final[i].x(), p_final[i].y()};
        uint32_t ret_idx = 0;
        double out_dist_sq = 0.0;
        if (impl_->tree->knnSearch(query_pt, 1, &ret_idx, &out_dist_sq) > 0) {
          if (out_dist_sq < max_dist_sq) {
            const auto& q = src_pts_[ret_idx];
            const auto& n = src_normals_[ret_idx];
            double r = n.x() * (p_final[i].x() - q.x()) + n.y() * (p_final[i].y() - q.y());
            if (robust_kernel_ == "huber") {
              double abs_r = std::abs(r);
              if (abs_r <= robust_kernel_scale_) {
                cost_local += 0.5 * r * r;
              } else {
                cost_local += robust_kernel_scale_ * (abs_r - 0.5 * robust_kernel_scale_);
              }
            } else if (robust_kernel_ == "cauchy") {
              double s = r / robust_kernel_scale_;
              cost_local += 0.5 * robust_kernel_scale_ * robust_kernel_scale_ * std::log1p(s * s);
            } else {
              cost_local += std::abs(r);
            }
            valid_local++;
          }
        }
      }
    }

    double cost_sum = 0.0;
    int valid_f = 0;
    for (int t = 0; t < num_threads; ++t) {
      cost_sum += thread_cost[t];
      valid_f += thread_valid_f[t];
    }

    double c_max = (robust_kernel_ == "huber")
        ? robust_kernel_scale_ * (max_correspondence_dist_ - 0.5 * robust_kernel_scale_)
        : max_correspondence_dist_;
    int n_total = static_cast<int>(dst_pts.size());
    if (valid_f >= kMinCorrespondences && n_total > 0) {
      double total_cost = cost_sum + static_cast<double>(n_total - valid_f) * c_max;
      score = total_cost / static_cast<double>(n_total);
    } else {
      score = c_max;
    }
  }

  return core::MatchResult{tx, ty, theta, converged, information, score};
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
