#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "slam_gnss_2d/pose_graph/base.hpp"
#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"
#include "slam_gnss_2d/scan_matching/base.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

class LoopClosureBuilder : public PoseGraphBuilderBase {
 public:
  LoopClosureBuilder(
      std::shared_ptr<ScanMatchingBuilder> inner,
      scan_matching::ScanMatcherPtr loop_matcher,
      double loop_closure_search_radius,
      int loop_closure_min_node_gap,
      int loop_closure_max_failure_streak,
      double max_loop_dyaw_deg,
      double loop_closure_crossing_reject_deg,
      double loop_closure_submap_radius,
      double loop_closure_max_score);

  std::optional<core::PoseNode> add_scan(
      const core::ScanDataPtr& scan,
      const core::OdomData& odom) override;

  std::vector<core::PoseNode> get_nodes() const override;
  std::vector<core::PoseEdge> get_edges() const override;
  void reset() override;

  bool loop_just_closed() override;
  void replace_nodes(const std::vector<core::PoseNode>& nodes) override;

  std::vector<core::PoseEdge> get_loop_edges() const;

  int loop_attempt_count() const { return loop_attempt_count_; }
  int loop_success_count() const { return loop_success_count_; }
  int icp_attempt_count() const { return inner_ ? inner_->icp_attempt_count() : 0; }
  int icp_success_count() const { return inner_ ? inner_->icp_success_count() : 0; }
  int odom_fallback_count() const { return inner_ ? inner_->odom_fallback_count() : 0; }

 private:
  std::shared_ptr<ScanMatchingBuilder> inner_;
  scan_matching::ScanMatcherPtr loop_matcher_;
  double search_radius_;
  int min_node_gap_;
  int max_failure_streak_;
  double max_loop_dyaw_rad_;
  double crossing_reject_rad_;
  double submap_radius_;
  double max_score_;

  std::vector<core::PoseEdge> loop_edges_;
  int loop_failure_streak_{0};
  bool loop_just_closed_flag_{false};
  int loop_attempt_count_{0};
  int loop_success_count_{0};
  std::vector<core::PoseNode> all_nodes_cache_;

  std::vector<core::PoseNode> find_loop_candidates(const core::PoseNode& node) const;
  std::vector<Eigen::Vector2d> build_candidate_submap(const core::PoseNode& candidate) const;
  bool try_add_loop_edge(const core::PoseNode& node, const core::PoseNode& candidate);
};

}  // namespace pose_graph
}  // namespace slam_gnss_2d
