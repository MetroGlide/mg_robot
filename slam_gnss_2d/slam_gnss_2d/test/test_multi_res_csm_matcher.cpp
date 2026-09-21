#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/scan_matching/multi_res_csm_matcher.hpp"
#include "slam_gnss_2d/scan_matching/icp_matcher.hpp"

namespace slam_gnss_2d {
namespace scan_matching {

namespace {

core::ScanDataPtr make_corner_scan() {
  auto scan = std::make_shared<core::ScanData>();
  scan->timestamp = 1.0;
  scan->angle_min = -M_PI;
  scan->angle_increment = 2.0 * M_PI / 360.0;
  scan->range_min = 0.1;
  scan->range_max = 30.0;
  scan->lidar_x = 0.0;
  scan->lidar_y = 0.0;
  scan->lidar_yaw = 0.0;
  scan->ranges.assign(360, 10.0f);

  for (int i = 0; i < 360; ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double ca = std::cos(angle);
    double sa = std::sin(angle);
    if (ca > 0.1 && std::abs(sa / ca) <= 1.0) {
      double r = 2.0 / ca;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
    if (sa > 0.1 && std::abs(ca / sa) <= 1.0) {
      double r = 2.0 / sa;
      if (r < scan->ranges[i]) scan->ranges[i] = static_cast<float>(r);
    }
  }
  return scan;
}

std::vector<Eigen::Vector2d> make_corner_target_points() {
  std::vector<Eigen::Vector2d> target;
  for (double y = -2.0; y <= 2.0; y += 0.03) {
    target.emplace_back(2.0, y);
  }
  for (double x = -2.0; x <= 2.0; x += 0.03) {
    target.emplace_back(x, 2.0);
  }
  return target;
}

}  // namespace

TEST(MultiResCSMMatcherTest, RecoversFromLargeInitialOffset) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      100, 1e-4, 1.0, "huber", 0.15, 1.0,
      10.0, 500.0, 0.0, 1e-4, 1e-4);

  MultiResCSMMatcher matcher(
      fine_matcher,
      0.3,    // linear_search_window [m]
      12.0,   // angular_search_window_deg [deg]
      0.02,   // linear_step [m]
      1.0,    // angular_step_deg [deg]
      0.03,   // grid_resolution [m]
      0.1);   // score_threshold

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  auto scan = make_corner_scan();

  // オドメトリ初期値に大きなバイアスを与える（並進 +0.15m, -0.10m, 回転 +6.0度）
  core::OdomData initial_guess{
      1.0,
      0.15,
      -0.10,
      6.0 * M_PI / 180.0,
  };

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.03);
  EXPECT_NEAR(result.dy, 0.0, 0.03);
  EXPECT_NEAR(result.dyaw, 0.0, 0.03);
  EXPECT_GT(result.score, 0.3);
}

TEST(MultiResCSMMatcherTest, HandlesPureRotationBias) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      100, 1e-4, 1.0, "huber", 0.15, 1.0,
      10.0, 500.0, 0.0, 1e-4, 1e-4);

  MultiResCSMMatcher matcher(
      fine_matcher,
      0.2,    // linear_search_window
      15.0,   // angular_search_window_deg
      0.02,   // linear_step
      0.5,    // angular_step_deg
      0.03,   // grid_resolution
      0.1);

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  auto scan = make_corner_scan();

  // 回転のみ 8.0度 の大きなバイアス
  core::OdomData initial_guess{
      1.0,
      0.0,
      0.0,
      8.0 * M_PI / 180.0,
  };

  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.03);
  EXPECT_NEAR(result.dy, 0.0, 0.03);
  EXPECT_NEAR(result.dyaw, 0.0, 0.02);
}

TEST(MultiResCSMMatcherTest, DegeneracyHandlingEmptyScan) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      50, 1e-3, 1.0, "huber", 0.15, 1.0);

  MultiResCSMMatcher matcher(fine_matcher);

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  core::OdomData initial_guess{1.0, 0.0, 0.0, 0.0};
  auto empty_scan = std::make_shared<core::ScanData>();
  auto result = matcher.match(empty_scan, initial_guess);

  EXPECT_FALSE(result.converged);
}

TEST(MultiResCSMMatcherTest, VariancePenaltyPreservesStraightHypothesis) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      100, 1e-4, 1.0, "huber", 0.15, 1.0,
      10.0, 500.0, 0.0, 1e-4, 1e-4);

  // Variance penalty を有効化 (distance=0.5, angle=1.0)
  MultiResCSMMatcher matcher(
      fine_matcher,
      0.3,    // linear_search_window
      15.0,   // angular_search_window_deg
      0.02,   // linear_step
      0.5,    // angular_step_deg
      0.03,   // grid_resolution
      0.1,    // score_threshold
      true,   // enable_variance_penalty
      0.5,    // distance_variance_penalty
      1.0,    // angle_variance_penalty
      0.5,    // minimum_distance_penalty
      0.9);   // minimum_angle_penalty

  auto target_pts = make_corner_target_points();
  matcher.set_target_cloud(target_pts);

  auto scan = make_corner_scan();

  // オドメトリ直進 (0.0, 0.0, 0.0)
  core::OdomData initial_guess{1.0, 0.0, 0.0, 0.0};
  auto result = matcher.match(scan, initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.02);
  EXPECT_NEAR(result.dy, 0.0, 0.02);
  EXPECT_NEAR(result.dyaw, 0.0, 0.01);
}

TEST(MultiResCSMMatcherTest, ReusableTargetGivesSameResultAsPlainTarget) {
  auto make_matcher = []() {
    return MultiResCSMMatcher(nullptr, 0.3, 12.0, 0.02, 1.0, 0.03, 0.1);
  };
  auto target_pts = make_corner_target_points();
  std::vector<Eigen::Vector2d> normals(target_pts.size(), Eigen::Vector2d(1.0, 0.0));
  auto scan = make_corner_scan();
  core::OdomData initial_guess{1.0, 0.10, -0.05, 4.0 * M_PI / 180.0};

  auto plain = make_matcher();
  plain.set_target_cloud_with_normals(target_pts, normals);
  auto expected = plain.match(scan, initial_guess);

  auto reusable = make_matcher();
  EXPECT_FALSE(reusable.try_use_reusable_target(7));
  reusable.set_reusable_target_cloud_with_normals(7, target_pts, normals);
  auto first = reusable.match(scan, initial_guess);

  // 別のターゲットを挟んでも、同じ key でキャッシュから復元できる
  std::vector<Eigen::Vector2d> other_pts = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  reusable.set_target_cloud_with_normals(other_pts, std::vector<Eigen::Vector2d>(3, Eigen::Vector2d(0.0, 1.0)));
  ASSERT_TRUE(reusable.try_use_reusable_target(7));
  auto second = reusable.match(scan, initial_guess);

  for (const auto& r : {first, second}) {
    EXPECT_EQ(r.converged, expected.converged);
    EXPECT_DOUBLE_EQ(r.dx, expected.dx);
    EXPECT_DOUBLE_EQ(r.dy, expected.dy);
    EXPECT_DOUBLE_EQ(r.dyaw, expected.dyaw);
    EXPECT_DOUBLE_EQ(r.score, expected.score);
  }

  reusable.clear_reusable_targets();
  EXPECT_FALSE(reusable.try_use_reusable_target(7));
}

TEST(MultiResCSMMatcherTest, ReusableTargetCacheEvictsOldestEntries) {
  MultiResCSMMatcher matcher(nullptr, 0.3, 12.0, 0.02, 1.0, 0.03, 0.1);
  auto target_pts = make_corner_target_points();
  std::vector<Eigen::Vector2d> normals(target_pts.size(), Eigen::Vector2d(1.0, 0.0));

  for (int key = 0; key < 20; ++key) {
    matcher.set_reusable_target_cloud_with_normals(key, target_pts, normals);
  }
  EXPECT_FALSE(matcher.try_use_reusable_target(0));
  EXPECT_TRUE(matcher.try_use_reusable_target(19));
}

TEST(MultiResCSMMatcherTest, FallsBackToIcpWithLazilyBuiltTarget) {
  auto fine_matcher = std::make_shared<ICPMatcher>(
      100, 1e-4, 1.0, "huber", 0.15, 1.0,
      10.0, 500.0, 0.0, 1e-4, 1e-4);
  // score_threshold を到達不能な値にして、必ず ICP フォールバックを通す
  MultiResCSMMatcher matcher(fine_matcher, 0.3, 12.0, 0.02, 1.0, 0.03, 2.0);

  auto target_pts = make_corner_target_points();
  std::vector<Eigen::Vector2d> normals;
  for (const auto& p : target_pts) {
    normals.push_back(p.x() > 1.99 ? Eigen::Vector2d(1.0, 0.0) : Eigen::Vector2d(0.0, 1.0));
  }
  matcher.set_target_cloud_with_normals(target_pts, normals);

  core::OdomData initial_guess{1.0, 0.05, -0.03, 2.0 * M_PI / 180.0};
  auto result = matcher.match(make_corner_scan(), initial_guess);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.dx, 0.0, 0.03);
  EXPECT_NEAR(result.dy, 0.0, 0.03);
}

TEST(MultiResCSMMatcherTest, BatchMatchGivesSameResultsAsSequentialMatches) {
  auto target_pts = make_corner_target_points();
  std::vector<Eigen::Vector2d> normals(target_pts.size(), Eigen::Vector2d(1.0, 0.0));
  auto scan = make_corner_scan();

  const std::vector<core::OdomData> guesses = {
      {1.0, 0.10, -0.05, 4.0 * M_PI / 180.0},
      {1.0, -0.08, 0.06, -3.0 * M_PI / 180.0},
      {1.0, 0.02, 0.10, 0.0},
  };

  // 逐次: キー付きターゲットを 1 つずつ設定して match
  MultiResCSMMatcher sequential(nullptr, 0.3, 12.0, 0.02, 1.0, 0.03, 0.1, true, 0.5, 1.0, 0.5, 0.9, 4);
  std::vector<core::MatchResult> expected;
  for (size_t i = 0; i < guesses.size(); ++i) {
    // 各キーに少しずつ違う点群を登録する
    std::vector<Eigen::Vector2d> pts = target_pts;
    for (auto& p : pts) {
      p.x() += 0.01 * static_cast<double>(i);
    }
    sequential.set_reusable_target_cloud_with_normals(static_cast<int>(i), pts, normals);
    expected.push_back(sequential.match(scan, guesses[i]));
  }

  // バッチ: 同じターゲットをすべて登録してから並列マッチング
  MultiResCSMMatcher batch(nullptr, 0.3, 12.0, 0.02, 1.0, 0.03, 0.1, true, 0.5, 1.0, 0.5, 0.9, 12);
  std::vector<ReusableMatchRequest> requests;
  for (size_t i = 0; i < guesses.size(); ++i) {
    std::vector<Eigen::Vector2d> pts = target_pts;
    for (auto& p : pts) {
      p.x() += 0.01 * static_cast<double>(i);
    }
    batch.set_reusable_target_cloud_with_normals(static_cast<int>(i), pts, normals);
    requests.push_back({static_cast<int>(i), guesses[i]});
  }
  // 未登録の key は不収束で返る
  requests.push_back({99, guesses[0]});
  auto results = batch.match_reusable_targets(scan, requests);

  ASSERT_EQ(results.size(), guesses.size() + 1);
  for (size_t i = 0; i < guesses.size(); ++i) {
    EXPECT_EQ(results[i].converged, expected[i].converged);
    EXPECT_DOUBLE_EQ(results[i].dx, expected[i].dx);
    EXPECT_DOUBLE_EQ(results[i].dy, expected[i].dy);
    EXPECT_DOUBLE_EQ(results[i].dyaw, expected[i].dyaw);
    EXPECT_DOUBLE_EQ(results[i].score, expected[i].score);
  }
  EXPECT_FALSE(results.back().converged);
}

TEST(MultiResCSMMatcherTest, ResultDoesNotDependOnThreadCount) {
  auto target_pts = make_corner_target_points();
  std::vector<Eigen::Vector2d> normals(target_pts.size(), Eigen::Vector2d(1.0, 0.0));
  auto scan = make_corner_scan();
  core::OdomData initial_guess{1.0, 0.10, -0.05, 4.0 * M_PI / 180.0};

  core::MatchResult reference;
  bool first = true;
  for (int threads : {1, 2, 5, 12}) {
    MultiResCSMMatcher matcher(nullptr, 0.3, 12.0, 0.02, 1.0, 0.03, 0.1, true, 0.5, 1.0, 0.5, 0.9, threads);
    matcher.set_target_cloud_with_normals(target_pts, normals);
    auto result = matcher.match(scan, initial_guess);
    if (first) {
      reference = result;
      first = false;
      continue;
    }
    EXPECT_DOUBLE_EQ(result.dx, reference.dx) << "threads=" << threads;
    EXPECT_DOUBLE_EQ(result.dy, reference.dy) << "threads=" << threads;
    EXPECT_DOUBLE_EQ(result.dyaw, reference.dyaw) << "threads=" << threads;
    EXPECT_DOUBLE_EQ(result.score, reference.score) << "threads=" << threads;
  }
}

}  // namespace scan_matching
}  // namespace slam_gnss_2d
