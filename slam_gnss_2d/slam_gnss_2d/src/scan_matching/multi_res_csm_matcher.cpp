#include "slam_gnss_2d/scan_matching/multi_res_csm_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
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

  void build(const std::vector<Eigen::Vector2d>& pts, double margin, int threads) {
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

    // 各点の中心セルを先に求め、行バンド単位で並列に描画する。
    // バンドごとに書き込むセルが排他的なので、スレッド別グリッドやマージは不要で、
    // 各セルは max 演算のみのため描画順に依存せず結果は逐次実行と一致する。
    const size_t num_pts = pts.size();
    std::vector<int> pt_gx(num_pts);
    std::vector<int> pt_gy(num_pts);
    for (size_t i = 0; i < num_pts; ++i) {
      pt_gx[i] = static_cast<int>((pts[i].x() - origin_x_) * inv_resolution_);
      pt_gy[i] = static_cast<int>((pts[i].y() - origin_y_) * inv_resolution_);
    }

    constexpr int kBandRows = 32;
    const int num_bands = (height_ + kBandRows - 1) / kBandRows;
    const int num_threads = (num_pts < 64) ? 1 : std::max(1, threads);

    #pragma omp parallel for schedule(dynamic, 1) num_threads(num_threads)
    for (int band = 0; band < num_bands; ++band) {
      const int band_min_gy = band * kBandRows;
      const int band_max_gy = std::min(height_ - 1, band_min_gy + kBandRows - 1);

      for (size_t i = 0; i < num_pts; ++i) {
        const int min_gy = std::max(band_min_gy, pt_gy[i] - kernel_cells);
        const int max_gy = std::min(band_max_gy, pt_gy[i] + kernel_cells);
        if (min_gy > max_gy) {
          continue;
        }
        const auto& p = pts[i];
        const int min_gx = std::max(0, pt_gx[i] - kernel_cells);
        const int max_gx = std::min(width_ - 1, pt_gx[i] + kernel_cells);

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

  // マッチング対象 (点群・法線・尤度グリッド)
  struct Target {
    std::vector<Eigen::Vector2d> pts;
    std::vector<Eigen::Vector2d> normals;
    LikelihoodGrid grid;

    Target(double resolution, double sigma) : grid(resolution, sigma) {}
  };

  // 毎回内容が変わるターゲット用 (グリッドのメモリを使い回す)
  Target scratch_target;
  // 再利用可能ターゲット (キー: 呼び出し側が付ける不変なID) の LRU キャッシュ
  static constexpr size_t kMaxReusableTargets = 8;
  std::deque<std::pair<int, std::shared_ptr<Target>>> reusable_targets;
  // 現在マッチング対象になっているターゲット (scratch_target または reusable_targets の要素)
  Target* active_target;

  // 1回のマッチング/グリッド構築が使う OpenMP スレッド数 (0: OpenMP の既定値)
  int num_threads;
  // ICP フォールバック (ステートを持つ fine_matcher) の排他制御
  std::mutex icp_mutex;

  // 尤度グリッドの標準偏差はグリッド解像度の kGridSigmaRatio 倍
  static constexpr double kGridSigmaRatio = 1.5;

  // ターゲットの点群・法線を設定し、尤度グリッドを構築する
  void fill_target(
      Target& target,
      const std::vector<Eigen::Vector2d>& pts,
      const std::vector<Eigen::Vector2d>& normals) {
    target.pts = pts;
    target.normals = normals;
    target.grid.build(pts, linear_search_window + 1.0, match_threads());
  }

  // key の再利用ターゲットを LRU キャッシュから取り出し、最近使用した位置に移す。無ければ nullptr
  std::shared_ptr<Target> find_target(int key) {
    for (auto it = reusable_targets.begin(); it != reusable_targets.end(); ++it) {
      if (it->first == key) {
        auto entry = *it;
        reusable_targets.erase(it);
        reusable_targets.push_back(entry);
        return entry.second;
      }
    }
    return nullptr;
  }

  int match_threads() const {
    const int max_threads = omp_get_max_threads();
    return num_threads > 0 ? std::min(num_threads, max_threads) : max_threads;
  }

  // 指定ターゲットに対する 3 段階 CSM の本体。スレッド数 threads で並列化する。
  core::MatchResult run_match(
      const Target& target,
      const std::vector<Eigen::Vector2d>& dst_pts,
      const core::OdomData& initial_guess,
      int threads);

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
      double min_angle_pen,
      int num_thr)
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
        scratch_target(grid_res, grid_res * kGridSigmaRatio),
        active_target(&scratch_target),
        num_threads(num_thr) {}
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
    double minimum_angle_penalty,
    int num_threads)
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
          minimum_angle_penalty,
          num_threads)) {
  // near-link の候補を並列にマッチングするため、OpenMP の入れ子並列 (外側+マッチング内部) を許可する
  omp_set_max_active_levels(2);
}

MultiResCSMMatcher::~MultiResCSMMatcher() = default;

void MultiResCSMMatcher::set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) {
  impl_->fill_target(impl_->scratch_target, src_pts, {});
  impl_->active_target = &impl_->scratch_target;
}

void MultiResCSMMatcher::set_target_cloud_with_normals(
    const std::vector<Eigen::Vector2d>& src_pts,
    const std::vector<Eigen::Vector2d>& src_normals) {
  impl_->fill_target(impl_->scratch_target, src_pts, src_normals);
  impl_->active_target = &impl_->scratch_target;
}

bool MultiResCSMMatcher::try_use_reusable_target(int key) {
  const auto target = impl_->find_target(key);
  if (!target) {
    return false;
  }
  impl_->active_target = target.get();
  return true;
}

void MultiResCSMMatcher::set_reusable_target_cloud_with_normals(
    int key,
    const std::vector<Eigen::Vector2d>& src_pts,
    const std::vector<Eigen::Vector2d>& src_normals) {
  auto& cache = impl_->reusable_targets;
  auto target = std::make_shared<Impl::Target>(
      impl_->grid_resolution, impl_->grid_resolution * Impl::kGridSigmaRatio);
  impl_->fill_target(*target, src_pts, src_normals);

  cache.erase(
      std::remove_if(cache.begin(), cache.end(), [key](const auto& e) { return e.first == key; }),
      cache.end());
  cache.emplace_back(key, target);
  while (cache.size() > Impl::kMaxReusableTargets) {
    cache.pop_front();
  }
  impl_->active_target = target.get();
}

void MultiResCSMMatcher::clear_reusable_targets() {
  impl_->active_target = &impl_->scratch_target;
  impl_->reusable_targets.clear();
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
  return impl_->run_match(*impl_->active_target, dst_pts, initial_guess, impl_->match_threads());
}

std::vector<core::MatchResult> MultiResCSMMatcher::match_reusable_targets(
    const core::ConstScanDataPtr& dst,
    const std::vector<ReusableMatchRequest>& requests) {
  std::vector<core::MatchResult> results;
  results.reserve(requests.size());
  for (const auto& req : requests) {
    results.push_back(core::MatchResult{
        req.initial_guess.x, req.initial_guess.y, req.initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0});
  }
  if (!dst || requests.empty()) {
    return results;
  }

  std::vector<Eigen::Vector2d> dst_pts = core::scan_to_points(dst);

  // キャッシュ済みターゲットを解決する (未登録の key は不収束のまま)
  std::vector<std::shared_ptr<Impl::Target>> targets(requests.size());
  for (size_t i = 0; i < requests.size(); ++i) {
    targets[i] = impl_->find_target(requests[i].key);
  }

  // 候補ごとのマッチングを並列に実行し、各マッチング内部も入れ子で並列化する。
  // 各マッチングは独立で、結果はスレッド数に依らず決定的。
  const int num_requests = static_cast<int>(requests.size());
  const int total_threads = impl_->match_threads();
  const bool parallel = num_requests > 1 && total_threads > 1;
  const int inner_threads = parallel
      ? std::max(1, std::min(total_threads, omp_get_max_threads() / num_requests))
      : total_threads;
  #pragma omp parallel for if(parallel) num_threads(num_requests) schedule(static, 1)
  for (int i = 0; i < num_requests; ++i) {
    if (targets[i]) {
      results[i] = impl_->run_match(*targets[i], dst_pts, requests[i].initial_guess, inner_threads);
    }
  }
  return results;
}

core::MatchResult MultiResCSMMatcher::Impl::run_match(
    const Target& target,
    const std::vector<Eigen::Vector2d>& dst_pts,
    const core::OdomData& initial_guess,
    int threads) {
  if (target.pts.empty() || !target.grid.is_valid() ||
      static_cast<int>(dst_pts.size()) < kMinCorrespondences) {
    return core::MatchResult{
        initial_guess.x, initial_guess.y, initial_guess.yaw,
        false, Eigen::Matrix3d::Zero(), 0.0};
  }

  RCLCPP_INFO_ONCE(
      rclcpp::get_logger("slam_gnss_2d.multi_res_csm"),
      "MultiResCSMMatcher OpenMP initialized (%d threads) with Scan Barycenter 2-stage CSM",
      threads);

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

  const auto& grid = target.grid;
  double inv_pts = 1.0 / static_cast<double>(dst_pts.size());

  // 各角度で重心基準の点群を回転させたもの
  const auto rotate_centered = [&](const std::vector<double>& thetas) {
    std::vector<std::vector<Eigen::Vector2d>> rotated(thetas.size());
    for (size_t a = 0; a < thetas.size(); ++a) {
      const double c = std::cos(thetas[a]);
      const double s = std::sin(thetas[a]);
      rotated[a].resize(centered_dst.size());
      for (size_t i = 0; i < centered_dst.size(); ++i) {
        rotated[a][i] = Eigen::Vector2d(
            c * centered_dst[i].x() - s * centered_dst[i].y(),
            s * centered_dst[i].x() + c * centered_dst[i].y());
      }
    }
    return rotated;
  };

  // 初期推定からの角度・距離のずれに対する分散ペナルティ (無効時は 1.0)
  const auto angle_penalty_at = [&](double theta) {
    if (!(enable_variance_penalty && angle_variance_penalty > 0.0)) {
      return 1.0;
    }
    const double dyaw = core::angle_diff(theta, initial_guess.yaw);
    const double var_yaw = angle_variance_penalty * angle_variance_penalty;
    return std::max(minimum_angle_penalty, std::exp(-0.5 * (dyaw * dyaw) / var_yaw));
  };
  const auto distance_penalty_at = [&](double dist_sq) {
    if (!(enable_variance_penalty && distance_variance_penalty > 0.0)) {
      return 1.0;
    }
    const double var_d = distance_variance_penalty * distance_variance_penalty;
    return std::max(minimum_distance_penalty, std::exp(-0.5 * dist_sq / var_d));
  };

  // --- Stage 1: Coarse CSM (広域グリッド相関探索) ---
  const int linear_steps = static_cast<int>(std::ceil(linear_search_window / linear_step));
  int yaw_steps = static_cast<int>(std::ceil(angular_search_window_rad / angular_step_rad));

  // 並進の探索窓は x, y で同じ
  std::vector<double> x_offsets;
  x_offsets.reserve(2 * linear_steps + 1);
  for (int i = -linear_steps; i <= linear_steps; ++i) {
    x_offsets.push_back(i * linear_step);
  }
  const auto& y_offsets = x_offsets;

  std::vector<double> yaw_candidates;
  yaw_candidates.reserve(2 * yaw_steps + 3);
  yaw_candidates.push_back(initial_guess.yaw);
  yaw_candidates.push_back(0.0); // 直進仮説 (dyaw = 0)
  for (int i = -yaw_steps; i <= yaw_steps; ++i) {
    double y_val = core::normalize_angle(initial_guess.yaw + i * angular_step_rad);
    yaw_candidates.push_back(y_val);
  }

  std::sort(yaw_candidates.begin(), yaw_candidates.end());
  yaw_candidates.erase(
      std::unique(yaw_candidates.begin(), yaw_candidates.end(),
                  [](double a, double b) { return std::abs(core::angle_diff(a, b)) < 1e-4; }),
      yaw_candidates.end());

  const int num_angles = static_cast<int>(yaw_candidates.size());
  const auto rotated_centered = rotate_centered(yaw_candidates);

  struct Candidate {
    double score{-1.0};
    double cx{0.0};
    double cy{0.0};
    double theta{0.0};
    // 同点時に探索順(角度→dx→dy)で決定的に選ぶためのキー
    int64_t order{std::numeric_limits<int64_t>::max()};
    bool updated{false};
  };

  const int max_threads = std::max(threads, 1);
  std::vector<Candidate> thread_best_stage1(max_threads);
  for (int t = 0; t < max_threads; ++t) {
    thread_best_stage1[t].cx = init_cx;
    thread_best_stage1[t].cy = init_cy;
    thread_best_stage1[t].theta = initial_guess.yaw;
  }

  #pragma omp parallel for schedule(dynamic, 1) num_threads(max_threads)
  for (int a = 0; a < num_angles; ++a) {
    int tid = omp_get_thread_num();
    double theta = yaw_candidates[a];
    const auto& r_pts = rotated_centered[a];
    size_t n_pts = r_pts.size();

    const double angle_penalty = angle_penalty_at(theta);

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

        const double dist_penalty = distance_penalty_at(dx * dx + dy * dy);

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

        int64_t order = (static_cast<int64_t>(a) * static_cast<int64_t>(num_x) +
                         static_cast<int64_t>(dx_idx)) * static_cast<int64_t>(num_y) +
                        static_cast<int64_t>(dy_idx);
        auto& thread_best = thread_best_stage1[tid];
        if (norm_score > thread_best.score ||
            (norm_score == thread_best.score && order < thread_best.order)) {
          thread_best.score = norm_score;
          thread_best.cx = cx;
          thread_best.cy = cy;
          thread_best.theta = theta;
          thread_best.order = order;
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
    if (thread_best_stage1[t].score > best_stage1.score ||
        (thread_best_stage1[t].score == best_stage1.score &&
         thread_best_stage1[t].order < best_stage1.order)) {
      best_stage1 = thread_best_stage1[t];
    }
  }

  // --- Stage 2: Fine CSM (解像度1cm / 角度0.2度 局所精密探索) ---
  double fine_linear_step = 0.01; // 1cm 刻み
  int fine_lin_steps = std::max(1, static_cast<int>(std::round(linear_step / fine_linear_step)));
  std::vector<double> fine_x_offsets;
  for (int i = -fine_lin_steps; i <= fine_lin_steps; ++i) {
    fine_x_offsets.push_back(i * fine_linear_step);
  }
  std::vector<double> fine_y_offsets = fine_x_offsets;

  double fine_ang_step_rad = 0.2 * M_PI / 180.0; // 0.2度 刻み
  int fine_ang_steps = std::max(1, static_cast<int>(std::round(angular_step_rad / fine_ang_step_rad)));
  std::vector<double> fine_yaw_candidates;
  for (int i = -fine_ang_steps; i <= fine_ang_steps; ++i) {
    fine_yaw_candidates.push_back(core::normalize_angle(best_stage1.theta + i * fine_ang_step_rad));
  }

  const int num_fine_angles = static_cast<int>(fine_yaw_candidates.size());
  const auto fine_rotated_centered = rotate_centered(fine_yaw_candidates);

  std::vector<Candidate> thread_best_stage2(max_threads);
  for (int t = 0; t < max_threads; ++t) {
    thread_best_stage2[t] = best_stage1;
    thread_best_stage2[t].order = std::numeric_limits<int64_t>::max();
    thread_best_stage2[t].updated = false;
  }

  #pragma omp parallel for schedule(dynamic, 1) num_threads(max_threads)
  for (int a = 0; a < num_fine_angles; ++a) {
    int tid = omp_get_thread_num();
    double theta = fine_yaw_candidates[a];
    const auto& r_pts = fine_rotated_centered[a];
    size_t n_pts = r_pts.size();

    const double angle_penalty = angle_penalty_at(theta);

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

        const double total_dx = cx - init_cx;
        const double total_dy = cy - init_cy;
        const double dist_penalty = distance_penalty_at(total_dx * total_dx + total_dy * total_dy);

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

        int64_t order = (static_cast<int64_t>(a) * static_cast<int64_t>(num_fx) +
                         static_cast<int64_t>(dx_idx)) * static_cast<int64_t>(num_fy) +
                        static_cast<int64_t>(dy_idx);
        auto& thread_best = thread_best_stage2[tid];
        // Stage1 の最良より真に高いものだけを採用し、Stage2 候補同士の同点は探索順で決める
        bool better = thread_best.updated
            ? (norm_score > thread_best.score ||
               (norm_score == thread_best.score && order < thread_best.order))
            : (norm_score > thread_best.score);
        if (better) {
          thread_best.score = norm_score;
          thread_best.cx = cx;
          thread_best.cy = cy;
          thread_best.theta = theta;
          thread_best.order = order;
          thread_best.updated = true;
        }
      }
    }
  }

  Candidate best_stage2 = best_stage1;
  bool stage2_found = false;
  for (int t = 0; t < max_threads; ++t) {
    const auto& c = thread_best_stage2[t];
    if (!c.updated) {
      continue;
    }
    if (!stage2_found || c.score > best_stage2.score ||
        (c.score == best_stage2.score && c.order < best_stage2.order)) {
      best_stage2 = c;
      stage2_found = true;
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

  bool csm_converged = (best_stage2.score >= score_threshold);

  // フォールバック: スコアが閾値未満の異常時のみ ICP を呼ぶ
  if (fine_matcher && !csm_converged) {
    std::lock_guard<std::mutex> icp_lock(icp_mutex);
    // ICP 用ターゲット (kd-tree・法線) は、このフォールバック時にだけ構築する
    if (target.normals.empty()) {
      fine_matcher->set_target_cloud(target.pts);
    } else {
      fine_matcher->set_target_cloud_with_normals(target.pts, target.normals);
    }
    core::OdomData csm_seed{
        initial_guess.timestamp,
        tx_opt,
        ty_opt,
        theta_opt,
    };
    auto fine_res = fine_matcher->match(dst_pts, csm_seed);
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

}  // namespace scan_matching
}  // namespace slam_gnss_2d
