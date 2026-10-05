#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "obstacle_detection/obstacle_detector.hpp"

namespace
{

using obstacle_detection::ObstacleDetectionParams;
using obstacle_detection::ObstacleDetector;
using obstacle_detection::Point3;
using obstacle_detection::PointCloudView;

// XYZ (float32) のみの点群バッファを作る補助クラス
class CloudBuilder
{
public:
  void add(float x, float y, float z)
  {
    data_.push_back(x);
    data_.push_back(y);
    data_.push_back(z);
  }

  // x-y 平面の矩形を step 間隔で敷き詰める（z 一定）
  void addPlane(float x0, float x1, float y0, float y1, float z, float step)
  {
    for (float x = x0; x < x1; x += step) {
      for (float y = y0; y < y1; y += step) {
        add(x, y, z);
      }
    }
  }

  // x-y の矩形断面を持つ柱（z を 0.05 刻みで積む）
  void addPillar(float x0, float x1, float y0, float y1, float z0, float z1, float step)
  {
    const int layers = static_cast<int>(std::round((z1 - z0) / 0.05f));
    for (int k = 0; k <= layers; ++k) {
      addPlane(x0, x1, y0, y1, z0 + 0.05f * k, step);
    }
  }

  PointCloudView view() const
  {
    PointCloudView v;
    v.data = reinterpret_cast<const uint8_t *>(data_.data());
    v.num_points = data_.size() / 3;
    v.point_step = 3 * sizeof(float);
    v.x_offset = 0;
    v.y_offset = sizeof(float);
    v.z_offset = 2 * sizeof(float);
    return v;
  }

private:
  std::vector<float> data_;
};

ObstacleDetectionParams defaultParams() { return ObstacleDetectionParams{}; }

const Eigen::Isometry3f kIdentity = Eigen::Isometry3f::Identity();

}  // namespace

TEST(ObstacleDetector, EmptyInputGivesEmptyOutput)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder cloud;
  detector.process(cloud.view(), kIdentity);
  EXPECT_TRUE(detector.obstacle_points().empty());
  EXPECT_TRUE(detector.clusters().empty());
}

TEST(ObstacleDetector, FlatGroundIsNotObstacle)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder cloud;
  cloud.addPlane(0.2f, 2.8f, -0.7f, 0.7f, 0.0f, 0.01f);
  detector.process(cloud.view(), kIdentity);
  EXPECT_TRUE(detector.obstacle_points().empty());
  EXPECT_TRUE(detector.clusters().empty());
}

TEST(ObstacleDetector, SlopedGroundBelowThresholdIsNotObstacle)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder cloud;
  // 勾配 約 5% の斜面。5cm セル内の高低差は約 2.5mm
  for (float x = 0.2f; x < 2.8f; x += 0.01f) {
    for (float y = -0.7f; y < 0.7f; y += 0.01f) {
      cloud.add(x, y, 0.05f * x);
    }
  }
  detector.process(cloud.view(), kIdentity);
  EXPECT_TRUE(detector.obstacle_points().empty());
}

TEST(ObstacleDetector, PillarIsDetectedWithBoundingBox)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder cloud;
  cloud.addPlane(0.2f, 2.8f, -0.7f, 0.7f, 0.0f, 0.02f);
  cloud.addPillar(1.50f, 1.60f, 0.00f, 0.10f, 0.0f, 0.5f, 0.02f);
  detector.process(cloud.view(), kIdentity);

  ASSERT_EQ(detector.clusters().size(), 1u);
  const auto & c = detector.clusters()[0];
  EXPECT_NEAR(c.min_x, 1.50f, 0.05f);
  EXPECT_NEAR(c.max_x, 1.60f, 0.05f);
  EXPECT_NEAR(c.min_y, 0.00f, 0.05f);
  EXPECT_NEAR(c.max_y, 0.10f, 0.05f);
  EXPECT_NEAR(c.min_z, 0.0f, 1e-4f);
  EXPECT_NEAR(c.max_z, 0.5f, 0.05f);
  EXPECT_EQ(static_cast<size_t>(c.num_points), detector.obstacle_points().size());
}

TEST(ObstacleDetector, LargeObstacleIsNotDroppedBySize)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder cloud;
  // 幅 1.2m・高さ 0.8m の壁。点数は数千点を超える
  cloud.addPillar(1.0f, 1.05f, -0.6f, 0.6f, 0.0f, 0.8f, 0.01f);
  detector.process(cloud.view(), kIdentity);

  EXPECT_GT(detector.obstacle_points().size(), 2000u);
  ASSERT_EQ(detector.clusters().size(), 1u);
  EXPECT_EQ(static_cast<size_t>(detector.clusters()[0].num_points), detector.obstacle_points().size());
}

TEST(ObstacleDetector, OutOfRoiAndNonFinitePointsAreIgnored)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder cloud;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  cloud.add(nan, 0.0f, 0.5f);
  cloud.add(1.0f, nan, 0.5f);
  cloud.add(1.0f, 0.0f, inf);
  cloud.addPillar(5.0f, 5.1f, 0.0f, 0.1f, 0.0f, 0.5f, 0.02f);   // x が範囲外
  cloud.addPillar(1.0f, 1.1f, 2.0f, 2.1f, 0.0f, 0.5f, 0.02f);   // y が範囲外
  cloud.addPillar(1.0f, 1.1f, 0.0f, 0.1f, 1.5f, 2.0f, 0.02f);   // z が範囲外
  detector.process(cloud.view(), kIdentity);
  EXPECT_TRUE(detector.obstacle_points().empty());
  EXPECT_EQ(detector.stats().cropped_points, 0u);
}

TEST(ObstacleDetector, SensorTransformIsApplied)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder cloud;
  // センサ座標では z 軸が前方。光学座標系 (x 右, y 下, z 前) -> base_link (x 前, y 左, z 上)
  Eigen::Matrix3f r;
  r << 0, 0, 1,
      -1, 0, 0,
       0, -1, 0;
  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
  tf.linear() = r;
  tf.translation() = Eigen::Vector3f(0.0f, 0.0f, 0.3f);  // センサは地上 0.3m

  // base_link で x=1.5, y=0.0〜0.1, z=0.0〜0.5 の柱に対応するセンサ座標の点群
  for (int k = 0; k <= 10; ++k) {
    const float z = 0.05f * k;
    for (float x = 1.5f; x < 1.6f; x += 0.02f) {
      for (float y = 0.0f; y < 0.1f; y += 0.02f) {
        const Eigen::Vector3f base(x, y, z);
        const Eigen::Vector3f sensor = tf.inverse() * base;
        cloud.add(sensor.x(), sensor.y(), sensor.z());
      }
    }
  }
  detector.process(cloud.view(), tf);

  ASSERT_EQ(detector.clusters().size(), 1u);
  EXPECT_NEAR(detector.clusters()[0].min_x, 1.5f, 0.03f);
  EXPECT_NEAR(detector.clusters()[0].max_z, 0.5f, 0.03f);
}

TEST(ObstacleDetector, IsolatedSingleCellNoiseIsRemoved)
{
  auto params = defaultParams();
  params.min_cluster_cells = 2;
  ObstacleDetector detector(params);
  CloudBuilder cloud;
  // 1 セルに収まる細い柱（0.03m 角）
  cloud.addPillar(1.00f, 1.03f, 0.00f, 0.03f, 0.0f, 0.5f, 0.01f);
  detector.process(cloud.view(), kIdentity);
  EXPECT_TRUE(detector.obstacle_points().empty());
  EXPECT_TRUE(detector.clusters().empty());
}

TEST(ObstacleDetector, NearbyObstaclesMergeOnlyWithinTolerance)
{
  auto params = defaultParams();
  params.cluster_tolerance = 0.15;
  ObstacleDetector detector(params);
  CloudBuilder cloud;
  cloud.addPillar(1.00f, 1.10f, 0.0f, 0.10f, 0.0f, 0.5f, 0.02f);
  cloud.addPillar(1.00f, 1.10f, 0.20f, 0.30f, 0.0f, 0.5f, 0.02f);  // 隙間 約 0.1m -> 併合
  cloud.addPillar(2.00f, 2.10f, 0.0f, 0.10f, 0.0f, 0.5f, 0.02f);  // 遠い -> 別クラスタ
  detector.process(cloud.view(), kIdentity);
  EXPECT_EQ(detector.clusters().size(), 2u);
}

TEST(ObstacleDetector, ResultsAreResetBetweenFrames)
{
  ObstacleDetector detector(defaultParams());
  CloudBuilder with_pillar;
  with_pillar.addPillar(1.5f, 1.6f, 0.0f, 0.1f, 0.0f, 0.5f, 0.02f);
  detector.process(with_pillar.view(), kIdentity);
  ASSERT_EQ(detector.clusters().size(), 1u);

  CloudBuilder ground_only;
  ground_only.addPlane(0.2f, 2.8f, -0.7f, 0.7f, 0.0f, 0.02f);
  detector.process(ground_only.view(), kIdentity);
  EXPECT_TRUE(detector.obstacle_points().empty());
  EXPECT_TRUE(detector.clusters().empty());

  detector.process(with_pillar.view(), kIdentity);
  EXPECT_EQ(detector.clusters().size(), 1u);
}

TEST(ObstacleDetector, StrideReducesProcessedPoints)
{
  auto params = defaultParams();
  params.stride = 4;
  ObstacleDetector detector(params);
  CloudBuilder cloud;
  cloud.addPlane(0.2f, 2.8f, -0.7f, 0.7f, 0.0f, 0.02f);
  detector.process(cloud.view(), kIdentity);
  EXPECT_NEAR(
    static_cast<double>(detector.stats().cropped_points),
    static_cast<double>(detector.stats().input_points) / 4.0,
    static_cast<double>(detector.stats().input_points) * 0.01);
}

TEST(ObstacleDetector, OutlierTrimIgnoresSingleFlyingPixel)
{
  CloudBuilder cloud;
  // 平地に 1 点だけ高い点（フライングピクセル）
  cloud.addPlane(1.00f, 1.05f, 0.0f, 0.05f, 0.0f, 0.01f);
  cloud.add(1.02f, 0.02f, 0.6f);

  auto base_params = defaultParams();
  base_params.min_cluster_cells = 1;
  ObstacleDetector no_trim(base_params);
  no_trim.process(cloud.view(), kIdentity);
  EXPECT_FALSE(no_trim.obstacle_points().empty()) << "trim なしでは飛び画素で誤検出する";

  auto params = base_params;
  params.z_outlier_trim = 1;
  ObstacleDetector trim(params);
  trim.process(cloud.view(), kIdentity);
  EXPECT_TRUE(trim.obstacle_points().empty());
}

TEST(ObstacleDetector, InvalidParamsThrow)
{
  auto bad = defaultParams();
  bad.grid_size = 0.0;
  EXPECT_THROW(ObstacleDetector{bad}, std::invalid_argument);

  bad = defaultParams();
  bad.cropbox_x_min = 4.0;
  EXPECT_THROW(ObstacleDetector{bad}, std::invalid_argument);

  bad = defaultParams();
  bad.stride = 0;
  EXPECT_THROW(ObstacleDetector{bad}, std::invalid_argument);

  bad = defaultParams();
  bad.z_outlier_trim = 2;
  EXPECT_THROW(ObstacleDetector{bad}, std::invalid_argument);

  bad = defaultParams();
  bad.grid_size = 1e-5;  // セル数が上限を超える
  EXPECT_THROW(ObstacleDetector{bad}, std::invalid_argument);
}
