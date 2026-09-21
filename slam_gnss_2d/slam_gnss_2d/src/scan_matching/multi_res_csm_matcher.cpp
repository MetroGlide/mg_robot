#include "slam_gnss_2d/scan_matching/multi_res_csm_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <nanoflann.hpp>
#include <omp.h>
#include <rclcpp/logging.hpp>

#ifndef _OPENMP
#error "OpenMP is NOT enabled! Ensure -fopenmp is passed to compiler."
#endif

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

class LikelihoodGrid {
 public:
  LikelihoodGrid(double resolution, double sigma)
      : resolution_(resolution), inv_resolution_(1.0 / resolution), sigma_(sigma) {}

  void build(const std::vector<Eigen::Vector2d>& pts, double margin) {
    if (pts.empty()) {
      clear();
      return;
    }

    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    for (const auto& p : pts) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }

    origin_x_ = min_x - margin;
    origin_y_ = min_y - margin;
    double extent_x = (max_x + margin) - origin_x_;
    double extent_y = (max_y + margin) - origin_y_;

    width_ = static_cast<int>(std::ceil(extent_x * inv_resolution_)) + 1;
    height_ = static_cast<int>(std::ceil(extent_y * inv_resolution_)) + 1;

    grid_.assign(width_ * height_, 0.0f);

    double sigma_sq2 = 2.0 * sigma_ * sigma_;
    double max_eval_dist = 3.0 * sigma_;
    double max_eval_dist_sq = max_eval_dist * max_eval_dist;
    int kernel_cells = static_cast<int>(std::ceil(max_eval_dist * inv_resolution_));

    int num_threads = std::min(4, omp_get_max_threads());
    if (num_threads <= 1 || pts.size() < 64) {
      for (const auto& p : pts) {
        int center_gx = static_cast<int>((p.x() - origin_x_) * inv_resolution_);
        int center_gy = static_cast<int>((p.y() - origin_y_) * inv_resolution_);

        int min_gx = std::max(0, center_gx - kernel_cells);
        int max_gx = std::min(width_ - 1, center_gx + kernel_cells);
        int min_gy = std::max(0, center_gy - kernel_cells);
        int max_gy = std::min(height_ - 1, center_gy + kernel_cells);

        for (int gy = min_gy; gy <= max_gy; ++gy) {
          double y = origin_y_ + (gy + 0.5) * resolution_;
          double dy = y - p.y();
          double dy_sq = dy * dy;
          int row_offset = gy * width_;

          for (int gx = min_gx; gx <= max_gx; ++gx) {
            double x = origin_x_ + (gx + 0.5) * resolution_;
            double dx = x - p.x();
            double dist_sq = dx * dx + dy_sq;

            if (dist_sq <= max_eval_dist_sq) {
              float val = static_cast<float>(std::exp(-dist_sq / sigma_sq2));
              float& cell = grid_[row_offset + gx];
              if (val > cell) {
                cell = val;
              }
            }
          }
        }
      }
    } else {
      if (thread_grids_.size() != static_cast<size_t>(num_threads)) {
        thread_grids_.resize(num_threads);
      }
      for (int t = 0; t < num_threads; ++t) {
        if (thread_grids_[t].size() != grid_.size()) {
          thread_grids_[t].assign(grid_.size(), 0.0f);
        } else {
          std::fill(thread_grids_[t].begin(), thread_grids_[t].end(), 0.0f);
        }
      }

      #pragma omp parallel num_threads(num_threads)
      {
        int tid = omp_get_thread_num();
        auto& local_grid = thread_grids_[tid];

        #pragma omp for schedule(dynamic, 32)
        for (size_t i = 0; i < pts.size(); ++i) {
          const auto& p = pts[i];
          int center_gx = static_cast<int>((p.x() - origin_x_) * inv_resolution_);
          int center_gy = static_cast<int>((p.y() - origin_y_) * inv_resolution_);

          int min_gx = std::max(0, center_gx - kernel_cells);
          int max_gx = std::min(width_ - 1, center_gx + kernel_cells);
          int min_gy = std::max(0, center_gy - kernel_cells);
          int max_gy = std::min(height_ - 1, center_gy + kernel_cells);

          for (int gy = min_gy; gy <= max_gy; ++gy) {
            double y = origin_y_ + (gy + 0.5) * resolution_;
            double dy = y - p.y();
            double dy_sq = dy * dy;
            int row_offset = gy * width_;

            for (int gx = min_gx; gx <= max_gx; ++gx) {
              double x = origin_x_ + (gx + 0.5) * resolution_;
              double dx = x - p.x();
              double dist_sq = dx * dx + dy_sq;

              if (dist_sq <= max_eval_dist_sq) {
                float val = static_cast<float>(std::exp(-dist_sq / sigma_sq2));
                float& cell = local_grid[row_offset + gx];
                if (val > cell) {
                  cell = val;
                }
              }
            }
          }
        }
      }

      #pragma omp parallel for schedule(static) num_threads(num_threads)
      for (size_t i = 0; i < grid_.size(); ++i) {
        float m = 0.0f;
        for (int t = 0; t < num_threads; ++t) {
          if (thread_grids_[t][i] > m) {
            m = thread_grids_[t][i];
          }
        }
        grid_[i] = m;
      }
    }
  }

  inline double origin_x() const { return origin_x_; }
  inline double origin_y() const { return origin_y_; }
  inline double resolution() const { return resolution_; }
  inline double inv_resolution() const { return inv_resolution_; }
  inline int width() const { return width_; }
  inline int height() const { return height_; }
  inline const float* data() const { return grid_.data(); }

  inline float lookup(double x, double y) const {
    int gx = static_cast<int>((x - origin_x_) * inv_resolution_);
    int gy = static_cast<int>((y - origin_y_) * inv_resolution_);
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
      return 0.0f;
    }
    return grid_[gy * width_ + gx];
  }

  inline float lookup_bilinear(double x, double y) const {
    double fx = (x - origin_x_) * inv_resolution_ - 0.5;
    double fy = (y - origin_y_) * inv_resolution_ - 0.5;
    int gx0 = static_cast<int>(std::floor(fx));
    int gy0 = static_cast<int>(std::floor(fy));
    int gx1 = gx0 + 1;
    int gy1 = gy0 + 1;
    if (gx0 < 0 || gx1 >= width_ || gy0 < 0 || gy1 >= height_) {
      return lookup(x, y);
    }
    float wx1 = static_cast<float>(fx - gx0);
    float wx0 = 1.0f - wx1;
    float wy1 = static_cast<float>(fy - gy0);
    float wy0 = 1.0f - wy1;
    return wx0 * wy0 * grid_[gy0 * width_ + gx0] +
           wx1 * wy0 * grid_[gy0 * width_ + gx1] +
           wx0 * wy1 * grid_[gy1 * width_ + gx0] +
           wx1 * wy1 * grid_[gy1 * width_ + gx1];
  }

  void clear() {
    grid_.clear();
    for (auto& tg : thread_grids_) {
      tg.clear();
    }
    thread_grids_.clear();
    width_ = 0;
    height_ = 0;
  }

  bool is_valid() const { return !grid_.empty(); }

 private:
  double resolution_{0.03};
  double inv_resolution_{1.0 / 0.03};
  double sigma_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int width_{0};
  int height_{0};
  std::vector<float> grid_;
  std::vector<std::vector<float>> thread_grids_;
};

}  // namespace

struct MultiResCSMMatcher::Impl {
  std::shared_ptr<ICPMatcher> fine_matcher;
  double linear_search_window;
  double angular_search_window_rad;
  double linear_step;
  double angular_step_rad;
  double grid_resolution;
  double score_threshold;
  bool enable_variance_penalty;
  double distance_variance_penalty;
  double angle_variance_penalty;
  double minimum_distance_penalty;
  double minimum_angle_penalty;

  std::vector<Eigen::Vector2d> target_pts;
  std::vector<Eigen::Vector2d> target_normals;
  LikelihoodGrid likelihood_grid;

  Impl(
      std::shared_ptr<ICPMatcher> fine,
      double l_win,
      double a_win_deg,
      double l_step,
      double a_step_deg,
      double grid_res,
      double score_thresh,
      bool enable_var_pen,
      double dist_var_pen,
      double angle_var_pen,
      double min_dist_pen,
      double min_angle_pen)
      : fine_matcher(std::move(fine)),
        linear_search_window(l_win),
        angular_search_window_rad(a_win_deg * M_PI / 180.0),
        linear_step(std::max(0.005, l_step)),
        angular_step_rad(std::max(0.05 * M_PI / 180.0, a_step_deg * M_PI / 180.0)),
        grid_resolution(grid_res),
        score_threshold(score_thresh),
        enable_variance_penalty(enable_var_pen),
        distance_variance_penalty(dist_var_pen),
        angle_variance_penalty(angle_var_pen),
        minimum_distance_penalty(min_dist_pen),
        minimum_angle_penalty(min_angle_pen),
        likelihood_grid(grid_res, grid_res * 1.5) {}
};

MultiResCSMMatcher::MultiResCSMMatcher(
    std::shared_ptr<ICPMatcher> fine_matcher,
    double linear_search_window,
    double angular_search_window_deg,
    double linear_step,
    double angular_step_deg,
    double grid_resolution,
    double score_threshold,
    bool enable_variance_penalty,
    double distance_variance_penalty,
    double angle_variance_penalty,
    double minimum_distance_penalty,
    double minimum_angle_penalty)
    : impl_(std::make_unique<Impl>(
          std::move(fine_matcher),
          linear_search_window,
          angular_search_window_deg,
          linear_step,
          angular_step_deg,
          grid_resolution,
          score_threshold,
          enable_variance_penalty,
          distance_variance_penalty,
          angle_variance_penalty,
          minimum_distance_penalty,
          minimum_angle_penalty)) {}

MultiResCSMMatcher::~MultiResCSMMatcher() = default;

void MultiResCSMMatcher::set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) {
  impl_->target_pts = src_pts;
  impl_->target_normals.clear();
  if (impl_->fine_matcher) {
    impl_->fine_matcher->set_target_cloud(src_pts);
  }
  impl_->likelihood_grid.build(src_pts, impl_->linear_search_window + 1.0);
}

void MultiResCSMMatcher::set_target_cloud_with_normals(
    const std::vector<Eigen::Vector2d>& src_pts,
    const std::vector<Eigen::Vector2d>& src_normals) {
  impl_->target_pts = src_pts;
  impl_->target_normals = src_normals;
  if (impl_->fine_matcher) {
    impl_->fine_matcher->set_target_cloud_with_normals(src_pts, src_normals);
  }
  impl_->likelihood_grid.build(src_pts, impl_->linear_search_window + 1.0);
}

core::MatchResult MultiResCSMMatcher::match(
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

core::MatchResult MultiResCSMMatcher::match(
    const std::vector<Eigen::Vector2d>& dst_pts,
    const core::OdomData& initial_guess) {
  if (impl_->target_pts.empty() || !impl_->likelihood_grid.is_valid() ||
      static_cast<int>(dst_pts.size()) < kMinCorrespondences) {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }

  RCLCPP_INFO_ONCE(
      rclcpp::get_logger("slam_gnss_2d.multi_res_csm"),
      "MultiResCSMMatcher OpenMP initialized (%d threads) with Scan Barycenter 2-stage CSM",
      omp_get_max_threads());

  // 1. 点群重心 (Scan Barycenter) の算出と重心基準シフト
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& p : dst_pts) {
    centroid += p;
  }
  centroid /= static_cast<double>(dst_pts.size());

  std::vector<Eigen::Vector2d> centered_dst(dst_pts.size());
  for (size_t i = 0; i < dst_pts.size(); ++i) {
    centered_dst[i] = dst_pts[i] - centroid;
  }

  // 初期推定における重心位置 (initial_guess の姿勢 yaw で回転させた重心を原点に加算)
  double cos_init = std::cos(initial_guess.yaw);
  double sin_init = std::sin(initial_guess.yaw);
  double init_cx = initial_guess.x + cos_init * centroid.x() - sin_init * centroid.y();
  double init_cy = initial_guess.y + sin_init * centroid.x() + cos_init * centroid.y();

  const auto& grid = impl_->likelihood_grid;
  double inv_pts = 1.0 / static_cast<double>(dst_pts.size());

  // --- Stage 1: Coarse CSM (広域グリッド相関探索) ---
  int x_steps = static_cast<int>(std::ceil(impl_->linear_search_window / impl_->linear_step));
  int y_steps = static_cast<int>(std::ceil(impl_->linear_search_window / impl_->linear_step));
  int yaw_steps = static_cast<int>(std::ceil(impl_->angular_search_window_rad / impl_->angular_step_rad));

  std::vector<double> x_offsets;
  x_offsets.reserve(2 * x_steps + 1);
  for (int i = -x_steps; i <= x_steps; ++i) {
    x_offsets.push_back(i * impl_->linear_step);
  }

  std::vector<double> y_offsets;
  y_offsets.reserve(2 * y_steps + 1);
  for (int i = -y_steps; i <= y_steps; ++i) {
    y_offsets.push_back(i * impl_->linear_step);
  }

  std::vector<double> yaw_candidates;
  yaw_candidates.reserve(2 * yaw_steps + 3);
  yaw_candidates.push_back(initial_guess.yaw);
  yaw_candidates.push_back(0.0); // 直進仮説 (dyaw = 0)
  for (int i = -yaw_steps; i <= yaw_steps; ++i) {
    double y_val = core::normalize_angle(initial_guess.yaw + i * impl_->angular_step_rad);
    yaw_candidates.push_back(y_val);
  }

  std::sort(yaw_candidates.begin(), yaw_candidates.end());
  yaw_candidates.erase(
      std::unique(yaw_candidates.begin(), yaw_candidates.end(),
                  [](double a, double b) { return std::abs(core::angle_diff(a, b)) < 1e-4; }),
      yaw_candidates.end());

  int num_angles = static_cast<int>(yaw_candidates.size());
  std::vector<std::vector<Eigen::Vector2d>> rotated_centered(num_angles);
  for (int a = 0; a < num_angles; ++a) {
    double theta = yaw_candidates[a];
    double c = std::cos(theta);
    double s = std::sin(theta);
    rotated_centered[a].resize(centered_dst.size());
    for (size_t i = 0; i < centered_dst.size(); ++i) {
      rotated_centered[a][i] = Eigen::Vector2d(
          c * centered_dst[i].x() - s * centered_dst[i].y(),
          s * centered_dst[i].x() + c * centered_dst[i].y());
    }
  }

  struct Candidate {
    double score{-1.0};
    double cx{0.0};
    double cy{0.0};
    double theta{0.0};
  };

  int max_threads = omp_get_max_threads();
  std::vector<Candidate> thread_best_stage1(max_threads);
  for (int t = 0; t < max_threads; ++t) {
    thread_best_stage1[t].cx = init_cx;
    thread_best_stage1[t].cy = init_cy;
    thread_best_stage1[t].theta = initial_guess.yaw;
  }

  #pragma omp parallel for schedule(dynamic, 1)
  for (int a = 0; a < num_angles; ++a) {
    int tid = omp_get_thread_num();
    double theta = yaw_candidates[a];
    const auto& r_pts = rotated_centered[a];
    size_t n_pts = r_pts.size();

    double angle_penalty = 1.0;
    if (impl_->enable_variance_penalty && impl_->angle_variance_penalty > 0.0) {
      double dyaw = core::angle_diff(theta, initial_guess.yaw);
      double var_yaw = impl_->angle_variance_penalty * impl_->angle_variance_penalty;
      angle_penalty = std::exp(-0.5 * (dyaw * dyaw) / var_yaw);
      angle_penalty = std::max(impl_->minimum_angle_penalty, angle_penalty);
    }

    size_t num_x = x_offsets.size();
    size_t num_y = y_offsets.size();
    double grid_ox = grid.origin_x();
    double grid_oy = grid.origin_y();
    double grid_inv_res = grid.inv_resolution();
    int grid_w = grid.width();
    int grid_h = grid.height();
    const float* grid_data = grid.data();

    // 1D事前計算: 各 dx に対する全点の gx 座標テーブル
    std::vector<int> gx_table(num_x * n_pts);
    std::vector<uint8_t> gx_all_valid(num_x, 1);
    for (size_t dx_idx = 0; dx_idx < num_x; ++dx_idx) {
      double cx = init_cx + x_offsets[dx_idx];
      int* gx_row = &gx_table[dx_idx * n_pts];
      bool all_valid = true;
      for (size_t i = 0; i < n_pts; ++i) {
        int gx = static_cast<int>((r_pts[i].x() + cx - grid_ox) * grid_inv_res);
        if (gx >= 0 && gx < grid_w) {
          gx_row[i] = gx;
        } else {
          gx_row[i] = -1;
          all_valid = false;
        }
      }
      gx_all_valid[dx_idx] = all_valid ? 1 : 0;
    }

    // 1D事前計算: 各 dy に対する全点の gy オフセットテーブル
    std::vector<int> gy_table(num_y * n_pts);
    std::vector<uint8_t> gy_all_valid(num_y, 1);
    for (size_t dy_idx = 0; dy_idx < num_y; ++dy_idx) {
      double cy = init_cy + y_offsets[dy_idx];
      int* gy_row = &gy_table[dy_idx * n_pts];
      bool all_valid = true;
      for (size_t i = 0; i < n_pts; ++i) {
        int gy = static_cast<int>((r_pts[i].y() + cy - grid_oy) * grid_inv_res);
        if (gy >= 0 && gy < grid_h) {
          gy_row[i] = gy * grid_w;
        } else {
          gy_row[i] = -1;
          all_valid = false;
        }
      }
      gy_all_valid[dy_idx] = all_valid ? 1 : 0;
    }

    // 並進探索ループ (内側は純粋なポインタ参照・加算)
    for (size_t dx_idx = 0; dx_idx < num_x; ++dx_idx) {
      double dx = x_offsets[dx_idx];
      double cx = init_cx + dx;
      const int* gx_row = &gx_table[dx_idx * n_pts];
      bool x_valid = (gx_all_valid[dx_idx] != 0);

      for (size_t dy_idx = 0; dy_idx < num_y; ++dy_idx) {
        double dy = y_offsets[dy_idx];
        double cy = init_cy + dy;
        const int* gy_row = &gy_table[dy_idx * n_pts];
        bool y_valid = (gy_all_valid[dy_idx] != 0);

        double dist_penalty = 1.0;
        if (impl_->enable_variance_penalty && impl_->distance_variance_penalty > 0.0) {
          double dist_sq = dx * dx + dy * dy;
          double var_d = impl_->distance_variance_penalty * impl_->distance_variance_penalty;
          dist_penalty = std::exp(-0.5 * dist_sq / var_d);
          dist_penalty = std::max(impl_->minimum_distance_penalty, dist_penalty);
        }

        double sum_score = 0.0;
        if (x_valid && y_valid) {
          #pragma omp simd reduction(+:sum_score)
          for (size_t i = 0; i < n_pts; ++i) {
            sum_score += grid_data[gy_row[i] + gx_row[i]];
          }
        } else {
          for (size_t i = 0; i < n_pts; ++i) {
            int gy_off = gy_row[i];
            int gx = gx_row[i];
            if (gy_off >= 0 && gx >= 0) {
              sum_score += grid_data[gy_off + gx];
            }
          }
        }

        double norm_score = sum_score * inv_pts * angle_penalty * dist_penalty;

        if (norm_score > thread_best_stage1[tid].score) {
          thread_best_stage1[tid].score = norm_score;
          thread_best_stage1[tid].cx = cx;
          thread_best_stage1[tid].cy = cy;
          thread_best_stage1[tid].theta = theta;
        }
      }
    }
  }

  Candidate best_stage1;
  best_stage1.cx = init_cx;
  best_stage1.cy = init_cy;
  best_stage1.theta = initial_guess.yaw;
  best_stage1.score = -1.0;

  for (int t = 0; t < max_threads; ++t) {
    if (thread_best_stage1[t].score > best_stage1.score) {
      best_stage1 = thread_best_stage1[t];
    }
  }

  // --- Stage 2: Fine CSM (解像度1cm / 角度0.2度 局所精密探索) ---
  double fine_linear_step = 0.01; // 1cm 刻み
  int fine_lin_steps = std::max(1, static_cast<int>(std::round(impl_->linear_step / fine_linear_step)));
  std::vector<double> fine_x_offsets;
  for (int i = -fine_lin_steps; i <= fine_lin_steps; ++i) {
    fine_x_offsets.push_back(i * fine_linear_step);
  }
  std::vector<double> fine_y_offsets = fine_x_offsets;

  double fine_ang_step_rad = 0.2 * M_PI / 180.0; // 0.2度 刻み
  int fine_ang_steps = std::max(1, static_cast<int>(std::round(impl_->angular_step_rad / fine_ang_step_rad)));
  std::vector<double> fine_yaw_candidates;
  for (int i = -fine_ang_steps; i <= fine_ang_steps; ++i) {
    fine_yaw_candidates.push_back(core::normalize_angle(best_stage1.theta + i * fine_ang_step_rad));
  }

  int num_fine_angles = static_cast<int>(fine_yaw_candidates.size());
  std::vector<std::vector<Eigen::Vector2d>> fine_rotated_centered(num_fine_angles);
  for (int a = 0; a < num_fine_angles; ++a) {
    double theta = fine_yaw_candidates[a];
    double c = std::cos(theta);
    double s = std::sin(theta);
    fine_rotated_centered[a].resize(centered_dst.size());
    for (size_t i = 0; i < centered_dst.size(); ++i) {
      fine_rotated_centered[a][i] = Eigen::Vector2d(
          c * centered_dst[i].x() - s * centered_dst[i].y(),
          s * centered_dst[i].x() + c * centered_dst[i].y());
    }
  }

  std::vector<Candidate> thread_best_stage2(max_threads);
  for (int t = 0; t < max_threads; ++t) {
    thread_best_stage2[t] = best_stage1;
  }

  #pragma omp parallel for schedule(dynamic, 1)
  for (int a = 0; a < num_fine_angles; ++a) {
    int tid = omp_get_thread_num();
    double theta = fine_yaw_candidates[a];
    const auto& r_pts = fine_rotated_centered[a];
    size_t n_pts = r_pts.size();

    double angle_penalty = 1.0;
    if (impl_->enable_variance_penalty && impl_->angle_variance_penalty > 0.0) {
      double dyaw = core::angle_diff(theta, initial_guess.yaw);
      double var_yaw = impl_->angle_variance_penalty * impl_->angle_variance_penalty;
      angle_penalty = std::exp(-0.5 * (dyaw * dyaw) / var_yaw);
      angle_penalty = std::max(impl_->minimum_angle_penalty, angle_penalty);
    }

    size_t num_fx = fine_x_offsets.size();
    size_t num_fy = fine_y_offsets.size();
    double grid_ox = grid.origin_x();
    double grid_oy = grid.origin_y();
    double grid_inv_res = grid.inv_resolution();
    int grid_w = grid.width();
    int grid_h = grid.height();
    const float* grid_data = grid.data();

    // 1D事前計算: x 方向のバイリニア補間成分 (gx0, gx1, wx0, wx1)
    struct BilinearX {
      int gx0;
      int gx1;
      float wx0;
      float wx1;
      bool valid;
    };
    std::vector<BilinearX> bx_table(num_fx * n_pts);
    std::vector<uint8_t> bx_all_valid(num_fx, 1);

    for (size_t dx_idx = 0; dx_idx < num_fx; ++dx_idx) {
      double cx = best_stage1.cx + fine_x_offsets[dx_idx];
      BilinearX* bx_row = &bx_table[dx_idx * n_pts];
      bool all_v = true;
      for (size_t i = 0; i < n_pts; ++i) {
        double fx = (r_pts[i].x() + cx - grid_ox) * grid_inv_res - 0.5;
        int gx0 = static_cast<int>(std::floor(fx));
        int gx1 = gx0 + 1;
        float wx1 = static_cast<float>(fx - gx0);
        float wx0 = 1.0f - wx1;
        bool v = (gx0 >= 0 && gx1 < grid_w);
        if (!v) all_v = false;
        bx_row[i] = BilinearX{gx0, gx1, wx0, wx1, v};
      }
      bx_all_valid[dx_idx] = all_v ? 1 : 0;
    }

    // 1D事前計算: y 方向のバイリニア補間成分 (gy0_off, gy1_off, wy0, wy1)
    struct BilinearY {
      int gy0_off;
      int gy1_off;
      float wy0;
      float wy1;
      bool valid;
    };
    std::vector<BilinearY> by_table(num_fy * n_pts);
    std::vector<uint8_t> by_all_valid(num_fy, 1);

    for (size_t dy_idx = 0; dy_idx < num_fy; ++dy_idx) {
      double cy = best_stage1.cy + fine_y_offsets[dy_idx];
      BilinearY* by_row = &by_table[dy_idx * n_pts];
      bool all_v = true;
      for (size_t i = 0; i < n_pts; ++i) {
        double fy = (r_pts[i].y() + cy - grid_oy) * grid_inv_res - 0.5;
        int gy0 = static_cast<int>(std::floor(fy));
        int gy1 = gy0 + 1;
        float wy1 = static_cast<float>(fy - gy0);
        float wy0 = 1.0f - wy1;
        bool v = (gy0 >= 0 && gy1 < grid_h);
        if (!v) all_v = false;
        by_row[i] = BilinearY{gy0 * grid_w, gy1 * grid_w, wy0, wy1, v};
      }
      by_all_valid[dy_idx] = all_v ? 1 : 0;
    }

    for (size_t dx_idx = 0; dx_idx < num_fx; ++dx_idx) {
      double dx = fine_x_offsets[dx_idx];
      double cx = best_stage1.cx + dx;
      const BilinearX* bx_row = &bx_table[dx_idx * n_pts];
      bool x_valid = (bx_all_valid[dx_idx] != 0);

      for (size_t dy_idx = 0; dy_idx < num_fy; ++dy_idx) {
        double dy = fine_y_offsets[dy_idx];
        double cy = best_stage1.cy + dy;
        const BilinearY* by_row = &by_table[dy_idx * n_pts];
        bool y_valid = (by_all_valid[dy_idx] != 0);

        double dist_penalty = 1.0;
        if (impl_->enable_variance_penalty && impl_->distance_variance_penalty > 0.0) {
          double total_dx = cx - init_cx;
          double total_dy = cy - init_cy;
          double dist_sq = total_dx * total_dx + total_dy * total_dy;
          double var_d = impl_->distance_variance_penalty * impl_->distance_variance_penalty;
          dist_penalty = std::exp(-0.5 * dist_sq / var_d);
          dist_penalty = std::max(impl_->minimum_distance_penalty, dist_penalty);
        }

        double sum_score = 0.0;
        if (x_valid && y_valid) {
          for (size_t i = 0; i < n_pts; ++i) {
            const auto& bx = bx_row[i];
            const auto& by = by_row[i];
            sum_score += by.wy0 * (bx.wx0 * grid_data[by.gy0_off + bx.gx0] +
                                  bx.wx1 * grid_data[by.gy0_off + bx.gx1]) +
                         by.wy1 * (bx.wx0 * grid_data[by.gy1_off + bx.gx0] +
                                  bx.wx1 * grid_data[by.gy1_off + bx.gx1]);
          }
        } else {
          for (size_t i = 0; i < n_pts; ++i) {
            const auto& bx = bx_row[i];
            const auto& by = by_row[i];
            if (bx.valid && by.valid) {
              sum_score += by.wy0 * (bx.wx0 * grid_data[by.gy0_off + bx.gx0] +
                                    bx.wx1 * grid_data[by.gy0_off + bx.gx1]) +
                           by.wy1 * (bx.wx0 * grid_data[by.gy1_off + bx.gx0] +
                                    bx.wx1 * grid_data[by.gy1_off + bx.gx1]);
            } else {
              sum_score += grid.lookup(r_pts[i].x() + cx, r_pts[i].y() + cy);
            }
          }
        }

        double norm_score = sum_score * inv_pts * angle_penalty * dist_penalty;

        if (norm_score > thread_best_stage2[tid].score) {
          thread_best_stage2[tid].score = norm_score;
          thread_best_stage2[tid].cx = cx;
          thread_best_stage2[tid].cy = cy;
          thread_best_stage2[tid].theta = theta;
        }
      }
    }
  }

  Candidate best_stage2 = best_stage1;
  for (int t = 0; t < max_threads; ++t) {
    if (thread_best_stage2[t].score > best_stage2.score) {
      best_stage2 = thread_best_stage2[t];
    }
  }

  // --- Stage 3: 2次放物線ピーク補間 (Quadratic Peak Fit & Curvature) ---
  auto eval_score = [&](double cx, double cy, double theta) -> double {
    double c = std::cos(theta);
    double s = std::sin(theta);
    double sum = 0.0;
    for (const auto& p : centered_dst) {
      double px = c * p.x() - s * p.y() + cx;
      double py = s * p.x() + c * p.y() + cy;
      sum += grid.lookup_bilinear(px, py);
    }
    return sum * inv_pts;
  };

  double s0 = best_stage2.score;
  double sx_m = eval_score(best_stage2.cx - fine_linear_step, best_stage2.cy, best_stage2.theta);
  double sx_p = eval_score(best_stage2.cx + fine_linear_step, best_stage2.cy, best_stage2.theta);
  double sy_m = eval_score(best_stage2.cx, best_stage2.cy - fine_linear_step, best_stage2.theta);
  double sy_p = eval_score(best_stage2.cx, best_stage2.cy + fine_linear_step, best_stage2.theta);
  double st_m = eval_score(best_stage2.cx, best_stage2.cy, best_stage2.theta - fine_ang_step_rad);
  double st_p = eval_score(best_stage2.cx, best_stage2.cy, best_stage2.theta + fine_ang_step_rad);

  auto fit_peak = [](double step, double s_prev, double s_mid, double s_next) -> std::pair<double, double> {
    double denom = s_prev - 2.0 * s_mid + s_next;
    if (denom < -1e-6) {
      double delta = -0.5 * step * (s_next - s_prev) / denom;
      if (std::abs(delta) <= step) {
        double curvature = -denom / (step * step);
        return {delta, curvature};
      }
    }
    return {0.0, 10.0};
  };

  auto [sub_dx, curv_x] = fit_peak(fine_linear_step, sx_m, s0, sx_p);
  auto [sub_dy, curv_y] = fit_peak(fine_linear_step, sy_m, s0, sy_p);
  auto [sub_dt, curv_t] = fit_peak(fine_ang_step_rad, st_m, s0, st_p);

  double cx_opt = best_stage2.cx + sub_dx;
  double cy_opt = best_stage2.cy + sub_dy;
  double theta_opt = core::normalize_angle(best_stage2.theta + sub_dt);

  // 重心位置からセンサ原点位置への幾何逆変換
  double c_opt = std::cos(theta_opt);
  double s_opt = std::sin(theta_opt);
  double tx_opt = cx_opt - (c_opt * centroid.x() - s_opt * centroid.y());
  double ty_opt = cy_opt - (s_opt * centroid.x() + c_opt * centroid.y());

  // 情報行列の構築 (正定値クランプ付き)
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
  double base_scale = std::max(50.0, best_stage2.score * 500.0);
  info(0, 0) = std::clamp(curv_x * 0.1 * base_scale, 20.0, 1000.0);
  info(1, 1) = std::clamp(curv_y * 0.1 * base_scale, 20.0, 1000.0);
  info(2, 2) = std::clamp(curv_t * 0.005 * base_scale, 50.0, 3000.0);

  bool csm_converged = (best_stage2.score >= impl_->score_threshold);

  // フォールバック: スコアが閾値未満の異常時のみ ICP を呼ぶ
  if (impl_->fine_matcher && !csm_converged) {
    core::OdomData csm_seed{
        initial_guess.timestamp,
        tx_opt,
        ty_opt,
        theta_opt,
    };
    auto fine_res = impl_->fine_matcher->match(dst_pts, csm_seed);
    if (fine_res.converged) {
      return fine_res;
    }
  }

  return core::MatchResult{
      tx_opt,
      ty_opt,
      theta_opt,
      csm_converged,
      info,
      csm_converged ? best_stage2.score : 0.0,
  };
}

std::shared_ptr<ICPMatcher> MultiResCSMMatcher::fine_matcher() const {
  return impl_->fine_matcher;
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
