#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "slam_gnss_2d/core/config.hpp"
#include "slam_gnss_2d/pose_graph/base.hpp"
#include "slam_gnss_2d/scan_matching/base.hpp"
#include "slam_gnss_2d/scan_matching/reference_provider/base.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

class ScanMatchingBuilder : public PoseGraphBuilderBase {
 public:
  ScanMatchingBuilder(
      scan_matching::ScanMatcherPtr matcher,
      scan_matching::ReferenceProviderPtr provider,
      double min_translation,
      double min_rotation,
      int max_failure_streak,
      double max_translation_drift = 0.08,
      bool enable_near_keyframe_links = true,
      int near_link_buffer_size = 10,
      double near_link_max_distance = 2.0,
      int near_link_min_index_diff = 2,
      int near_link_max_links_per_node = 3,
      double near_link_max_translation_drift = 0.4,
      double near_link_max_rotation_drift_deg = 15.0,
      double near_link_min_eigenvalue = 10.0,
      core::OdomFusionConfig odom_fusion = core::OdomFusionConfig{});

  std::optional<core::PoseNode> add_scan(
      const core::ScanDataPtr& scan,
      const core::OdomData& odom) override;

  std::vector<core::PoseNode> get_nodes() const override;
  std::vector<core::PoseEdge> get_edges() const override;
  void reset() override;

  bool loop_just_closed() override { return false; }
  void replace_nodes(const std::vector<core::PoseNode>& nodes) override;
  std::string timing_summary() const override;

  int icp_attempt_count() const { return icp_attempt_count_; }
  int icp_success_count() const { return icp_success_count_; }
  int odom_fallback_count() const { return odom_fallback_count_; }
  int near_link_success_count() const { return near_link_success_count_; }

 private:
  void add_near_keyframe_links(const core::PoseNode& current_node);
  void fuse_with_odometry(
      double odom_dx, double odom_dy, double odom_dyaw,
      double& dx, double& dy, double& dyaw,
      Eigen::Matrix3d& information) const;

  scan_matching::ScanMatcherPtr matcher_;
  scan_matching::ReferenceProviderPtr provider_;
  double min_translation_;
  double min_rotation_;
  int max_failure_streak_;
  double max_translation_drift_{0.08};

  bool enable_near_keyframe_links_{true};
  int near_link_buffer_size_{10};
  double near_link_max_distance_{2.0};
  int near_link_min_index_diff_{2};
  int near_link_max_links_per_node_{3};
  double near_link_max_translation_drift_{0.4};
  double near_link_max_rotation_drift_rad_{15.0 * M_PI / 180.0};
  double near_link_min_eigenvalue_{10.0};
  int near_link_success_count_{0};
  core::OdomFusionConfig odom_fusion_;

  std::vector<core::PoseNode> nodes_;
  std::vector<core::PoseEdge> edges_;
  std::optional<core::OdomData> last_odom_;
  int failure_streak_{0};

  struct StageTimes {
    double reference_sec{0.0};   // ローカルマップ参照点群の構築
    double set_target_sec{0.0};  // マッチャのターゲット設定 (尤度グリッド構築等)
    double match_sec{0.0};       // 逐次マッチング本体
    double near_link_sec{0.0};   // near-link マッチング全体
    double near_target_sec{0.0}; // うち候補ターゲットの構築 (キャッシュミス時)
    double near_match_sec{0.0};  // うちマッチング本体
  };
  StageTimes stage_times_;

  int icp_attempt_count_{0};
  int icp_success_count_{0};
  int odom_fallback_count_{0};
};

}  // namespace pose_graph
}  // namespace slam_gnss_2d
