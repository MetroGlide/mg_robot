#include <gtest/gtest.h>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/map_manager/counting_renderer.hpp"

namespace slam_gnss_2d {
namespace map_manager {
namespace {

core::ScanDataPtr make_single_point_scan(double distance, double angle = 0.0) {
  auto scan = std::make_shared<core::ScanData>();
  scan->timestamp = 100.0;
  scan->angle_min = angle;
  scan->angle_increment = 0.1;
  scan->range_min = 0.1;
  scan->range_max = 10.0;
  scan->lidar_x = 0.0;
  scan->lidar_y = 0.0;
  scan->ranges.push_back(static_cast<float>(distance));
  return scan;
}

TEST(TestCountingRenderer, SingleHitBecomesOccupied) {
  // 1回だけヒットした点（hits=1, misses=0）が min_hits の誤適用で Free (0) にならず
  // 確実に Occupied (100) になることを検証
  double resolution = 0.1;
  double expansion_margin = 2.0;
  double hit_threshold = 0.3;
  int min_hits = 1;

  CountingRenderer renderer(resolution, expansion_margin, hit_threshold, min_hits);

  core::PoseNode node0;
  node0.index = 0;
  node0.timestamp = 100.0;
  node0.x = 0.0;
  node0.y = 0.0;
  node0.yaw = 0.0;
  node0.scan = make_single_point_scan(1.0, 0.0);  // 前方 1.0m にヒット

  // 初回ノード投入
  renderer.rerender_all({node0});

  auto occ_data = renderer.to_occupancy_array();
  const cv::Mat& grid = occ_data.data;

  int hit_px = static_cast<int>((1.0 - occ_data.origin_x) / resolution);
  int hit_py = static_cast<int>((0.0 - occ_data.origin_y) / resolution);

  // ヒット点は 100 (Occupied) であること
  EXPECT_EQ(grid.at<int8_t>(hit_py, hit_px), 100);

  // ロボットとヒット点の中間点は 0 (Free) であること
  int mid_px = static_cast<int>((0.5 - occ_data.origin_x) / resolution);
  int mid_py = static_cast<int>((0.0 - occ_data.origin_y) / resolution);
  EXPECT_EQ(grid.at<int8_t>(mid_py, mid_px), 0);

  // レーザーの届かない遠方は -1 (Unknown) であること
  int far_px = static_cast<int>((3.0 - occ_data.origin_x) / resolution);
  int far_py = static_cast<int>((3.0 - occ_data.origin_y) / resolution);
  if (far_px >= 0 && far_px < grid.cols && far_py >= 0 && far_py < grid.rows) {
    EXPECT_EQ(grid.at<int8_t>(far_py, far_px), -1);
  }
}

TEST(TestCountingRenderer, HitRatioThreshold) {
  // hits=1, misses=3 の場合、比率は 1/(1+3) = 0.25
  // hit_threshold=0.3 のとき 0.25 < 0.3 なので Free (0) になる
  // hit_threshold=0.2 のとき 0.25 >= 0.2 なので Occupied (100) になる
  double resolution = 0.1;
  double expansion_margin = 5.0;

  CountingRenderer renderer_strict(resolution, expansion_margin, 0.3, 1);
  CountingRenderer renderer_lenient(resolution, expansion_margin, 0.2, 1);

  // 1回目は (1.0, 0.0) で反射
  core::PoseNode node_hit;
  node_hit.index = 0;
  node_hit.timestamp = 100.0;
  node_hit.x = 0.0;
  node_hit.y = 0.0;
  node_hit.yaw = 0.0;
  node_hit.scan = make_single_point_scan(1.0, 0.0);

  // 2〜4回目は (2.0, 0.0) へ抜ける光線（(1.0, 0.0) を通過）
  core::PoseNode node_pass;
  node_pass.index = 1;
  node_pass.timestamp = 101.0;
  node_pass.x = 0.0;
  node_pass.y = 0.0;
  node_pass.yaw = 0.0;
  node_pass.scan = make_single_point_scan(2.0, 0.0);

  std::vector<core::PoseNode> nodes = {node_hit, node_pass, node_pass, node_pass};

  renderer_strict.rerender_all(nodes);
  renderer_lenient.rerender_all(nodes);

  auto occ_strict = renderer_strict.to_occupancy_array();
  auto occ_lenient = renderer_lenient.to_occupancy_array();

  int target_px = static_cast<int>((1.0 - occ_strict.origin_x) / resolution);
  int target_py = static_cast<int>((0.0 - occ_strict.origin_y) / resolution);

  // 厳格な閾値(0.3)では 0.25 < 0.3 のため Free (0)
  EXPECT_EQ(occ_strict.data.at<int8_t>(target_py, target_px), 0);

  // 緩やかな閾値(0.2)では 0.25 >= 0.2 のため Occupied (100)
  EXPECT_EQ(occ_lenient.data.at<int8_t>(target_py, target_px), 100);
}

TEST(CountingRendererTest, SubmapPatchRendersCorrectly) {
  double resolution = 0.05;
  CountingRenderer renderer(resolution, 10.0, 0.2, 1);

  core::PoseNode node;
  node.index = 0;
  node.timestamp = 100.0;
  node.x = 2.0;
  node.y = 3.0;
  node.yaw = 0.0;

  auto patch = std::make_shared<core::SubmapPatch>();
  patch->resolution = resolution;
  patch->width = 10;
  patch->height = 10;
  patch->origin_x = 0.0;
  patch->origin_y = 0.0;
  patch->hit_patch = cv::Mat::zeros(10, 10, CV_32SC1);
  patch->miss_patch = cv::Mat::zeros(10, 10, CV_32SC1);

  // パッチ内の (2, 2) に hit=5 を設定
  patch->hit_patch.at<int32_t>(2, 2) = 5;
  node.submap_patch = patch;

  renderer.rerender_all({node});

  auto occ = renderer.to_occupancy_array();
  int occupied_count = 0;
  int found_px = -1;
  int found_py = -1;
  for (int y = 0; y < occ.data.rows; ++y) {
    for (int x = 0; x < occ.data.cols; ++x) {
      if (occ.data.at<int8_t>(y, x) == 100) {
        occupied_count++;
        found_px = x;
        found_py = y;
      }
    }
  }
  EXPECT_EQ(occupied_count, 1);

  // node (2.0, 3.0) + patch offset (2*0.05, 2*0.05) = (2.10, 3.10)
  int expected_px = static_cast<int>((2.10 - occ.origin_x) / resolution);
  int expected_py = static_cast<int>((3.10 - occ.origin_y) / resolution);
  EXPECT_NEAR(found_px, expected_px, 1);
  EXPECT_NEAR(found_py, expected_py, 1);
}

}  // namespace
}  // namespace map_manager
}  // namespace slam_gnss_2d
