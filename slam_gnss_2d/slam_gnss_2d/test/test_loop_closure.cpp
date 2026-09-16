#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "slam_gnss_2d/pose_graph/odom_builder.hpp"
#include "slam_gnss_2d/pose_graph/scan_matching_builder.hpp"
#include "slam_gnss_2d/pose_graph/loop_closure_builder.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"
#include "slam_gnss_2d/scan_matching/reference_provider/scan_to_scan.hpp"

namespace slam_gnss_2d {
namespace pose_graph {

TEST(LoopClosureTest, ReplaceNodesUpdatesNodes) {
  auto matcher = std::make_shared<scan_matching::ICPMatcher>(
      10, 1e-3, 1.0, "huber", 0.15, 1.0);
  auto ref_provider = std::make_unique<scan_matching::ScanToScanProvider>();
  auto sm_builder = std::make_shared<ScanMatchingBuilder>(
      matcher, std::move(ref_provider), 0.2, 0.1, 5);

  auto loop_matcher = std::make_shared<scan_matching::ICPMatcher>(
      10, 1e-3, 1.0, "huber", 0.05, 100.0);

  LoopClosureBuilder lc_builder(
      sm_builder,
      loop_matcher,
      5.0,   // search_radius
      3,     // min_node_gap
      3,     // max_failure_streak
      30.0,  // max_loop_dyaw_deg
      0.0,   // crossing_reject_deg
      3.0,   // submap_radius
      0.1    // max_score
  );

  std::vector<core::PoseNode> initial_nodes;
  initial_nodes.push_back({0, 1.0, 0.0, 0.0, 0.0, nullptr});
  initial_nodes.push_back({1, 2.0, 1.0, 0.0, 0.0, nullptr});

  lc_builder.replace_nodes(initial_nodes);

  auto nodes = lc_builder.get_nodes();
  ASSERT_EQ(nodes.size(), 2u);
  EXPECT_NEAR(nodes[0].x, 0.0, 1e-6);
  EXPECT_NEAR(nodes[1].x, 1.0, 1e-6);

  // Update nodes after backend optimization
  std::vector<core::PoseNode> updated_nodes;
  updated_nodes.push_back({0, 1.0, 0.5, 0.2, 0.1, nullptr});
  updated_nodes.push_back({1, 2.0, 1.5, 0.3, 0.2, nullptr});

  lc_builder.replace_nodes(updated_nodes);

  nodes = lc_builder.get_nodes();
  ASSERT_EQ(nodes.size(), 2u);
  EXPECT_NEAR(nodes[0].x, 0.5, 1e-6);
  EXPECT_NEAR(nodes[0].y, 0.2, 1e-6);
  EXPECT_NEAR(nodes[0].yaw, 0.1, 1e-6);
  EXPECT_NEAR(nodes[1].x, 1.5, 1e-6);
  EXPECT_NEAR(nodes[1].y, 0.3, 1e-6);
  EXPECT_NEAR(nodes[1].yaw, 0.2, 1e-6);
}

}  // namespace pose_graph
}  // namespace slam_gnss_2d
