#include "slam_gnss_2d/scan_matching/csm_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <nanoflann.hpp>

#include "slam_gnss_2d/core/geometry.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

namespace {

constexpr int kMinCorrespondences = 10;

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

}  // namespace

struct CSMMatcher::Impl {
  std::unique_ptr<PointCloud2DAdaptor> adaptor;
  std::unique_ptr<KDTree2D> tree;
};

CSMMatcher::CSMMatcher(
    double linear_search_window,
    double angular_search_window,
    double linear_step,
    double angular_step,
    double yaw_information_multiplier)
    : linear_search_window_(linear_search_window),
      angular_search_window_(angular_search_window),
      linear_step_(linear_step),
      angular_step_(angular_step),
      yaw_information_multiplier_(yaw_information_multiplier),
      impl_(std::make_unique<Impl>()) {}

CSMMatcher::~CSMMatcher() = default;

void CSMMatcher::set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) {
  src_pts_ = src_pts;
  if (static_cast<int>(src_pts_.size()) >= kMinCorrespondences) {
    impl_->adaptor = std::make_unique<PointCloud2DAdaptor>(PointCloud2DAdaptor{src_pts_});
    impl_->tree = std::make_unique<KDTree2D>(
        2, *impl_->adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    impl_->tree->buildIndex();
  } else {
    impl_->tree.reset();
    impl_->adaptor.reset();
  }
}

core::MatchResult CSMMatcher::match(
    const core::ConstScanDataPtr& dst,
    const core::OdomData& initial_guess) {
  if (src_pts_.empty() || !impl_->tree || !dst) {
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

  double best_score = -std::numeric_limits<double>::infinity();
  double best_tx = initial_guess.x;
  double best_ty = initial_guess.y;
  double best_theta = initial_guess.yaw;

  int x_steps = static_cast<int>(linear_search_window_ / linear_step_);
  int y_steps = static_cast<int>(linear_search_window_ / linear_step_);
  int yaw_steps = static_cast<int>(angular_search_window_ / angular_step_);

  std::vector<double> x_offsets;
  x_offsets.reserve(2 * x_steps + 1);
  for (int i = -x_steps; i <= x_steps; ++i) {
    x_offsets.push_back(i * linear_step_);
  }

  std::vector<double> y_offsets;
  y_offsets.reserve(2 * y_steps + 1);
  for (int i = -y_steps; i <= y_steps; ++i) {
    y_offsets.push_back(i * linear_step_);
  }

  std::vector<double> yaw_offsets;
  yaw_offsets.reserve(2 * yaw_steps + 1);
  for (int i = -yaw_steps; i <= yaw_steps; ++i) {
    yaw_offsets.push_back(i * angular_step_);
  }

  double sigma_sq = std::pow(linear_step_ * 2.0, 2.0);
  double max_dist_sq = std::pow(linear_step_ * 3.0, 2.0);

  for (double dyaw : yaw_offsets) {
    double theta = initial_guess.yaw + dyaw;
    double c = std::cos(theta);
    double s = std::sin(theta);

    std::vector<Eigen::Vector2d> rotated_dst;
    rotated_dst.reserve(dst_pts.size());
    for (const auto& p : dst_pts) {
      rotated_dst.emplace_back(c * p.x() - s * p.y(), s * p.x() + c * p.y());
    }

    for (double dx : x_offsets) {
      for (double dy : y_offsets) {
        double tx = initial_guess.x + dx;
        double ty = initial_guess.y + dy;

        double score = 0.0;
        int valid_count = 0;

        for (const auto& p : rotated_dst) {
          double query_pt[2] = {p.x() + tx, p.y() + ty};
          uint32_t ret_idx = 0;
          double out_dist_sq = 0.0;
          if (impl_->tree->knnSearch(query_pt, 1, &ret_idx, &out_dist_sq) > 0) {
            if (out_dist_sq <= max_dist_sq) {
              valid_count++;
              score += std::exp(-0.5 * out_dist_sq / sigma_sq);
            }
          }
        }

        if (valid_count >= kMinCorrespondences && score > best_score) {
          best_score = score;
          best_tx = tx;
          best_ty = ty;
          best_theta = theta;
        }
      }
    }
  }

  bool converged = (best_score > 0.0);
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  if (converged) {
    double factor = (best_score / static_cast<double>(dst_pts.size())) * 100.0;
    information = Eigen::Matrix3d::Identity() * factor;
    information(2, 2) *= yaw_information_multiplier_;
  }

  return core::MatchResult{
      best_tx, best_ty, best_theta, converged, information,
      converged ? best_score : 0.0};
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
