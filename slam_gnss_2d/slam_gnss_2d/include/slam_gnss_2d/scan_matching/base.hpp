#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

// 登録済みの再利用ターゲット (key) に対するマッチング要求
struct ReusableMatchRequest {
  int key;
  core::OdomData initial_guess;
};

class ScanMatcherBase {
 public:
  virtual ~ScanMatcherBase() = default;

  virtual void set_target_cloud(const std::vector<Eigen::Vector2d>& src_pts) = 0;
  virtual void set_target_cloud_with_normals(
      const std::vector<Eigen::Vector2d>& src_pts,
      const std::vector<Eigen::Vector2d>& src_normals) {
    (void)src_normals;
    set_target_cloud(src_pts);
  }

  // 内容が不変な点群を ID (key) 付きで登録し、同じ key の再設定でターゲット構築を省くための API
  // (try_use / set_reusable / match_reusable_targets)。対応しているかは supports_reusable_targets() で分かる。
  // 対応しないマッチャは常に try_use が false を返し、呼び出し側が通常のターゲット設定にフォールバックする。
  virtual bool supports_reusable_targets() const { return false; }
  virtual bool try_use_reusable_target(int /*key*/) { return false; }
  virtual void set_reusable_target_cloud_with_normals(
      int /*key*/,
      const std::vector<Eigen::Vector2d>& src_pts,
      const std::vector<Eigen::Vector2d>& src_normals) {
    set_target_cloud_with_normals(src_pts, src_normals);
  }
  virtual void clear_reusable_targets() {}

  // 登録済みの再利用ターゲット複数に対して、同一スキャンをマッチングする (結果は requests と同順)。
  // 未登録の key は不収束の結果を返す。既定実装は逐次実行で、対応するマッチャは並列化してよい。
  virtual std::vector<core::MatchResult> match_reusable_targets(
      const core::ConstScanDataPtr& dst,
      const std::vector<ReusableMatchRequest>& requests) {
    std::vector<core::MatchResult> results;
    results.reserve(requests.size());
    for (const auto& req : requests) {
      if (try_use_reusable_target(req.key)) {
        results.push_back(match(dst, req.initial_guess));
      } else {
        results.push_back(core::MatchResult{
            req.initial_guess.x, req.initial_guess.y, req.initial_guess.yaw,
            false, Eigen::Matrix3d::Zero(), 0.0});
      }
    }
    return results;
  }

  virtual core::MatchResult match(
      const core::ConstScanDataPtr& dst,
      const core::OdomData& initial_guess) = 0;

  core::MatchResult match(
      const core::ScanDataPtr& dst,
      const core::OdomData& initial_guess) {
    return match(std::static_pointer_cast<const core::ScanData>(dst), initial_guess);
  }
};

using ScanMatcherPtr = std::shared_ptr<ScanMatcherBase>;

}  // namespace scan_matching
}  // namespace slam_gnss_2d
