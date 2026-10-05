#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace obstacle_detection
{

struct Point3
{
  float x;
  float y;
  float z;
};

// 既定値は params/obstacle_detection.yaml と揃える（yaml が実運用の値）
struct ObstacleDetectionParams
{
  // 入力点の間引き間隔（1 = 全点）
  int stride = 1;

  // ROI (base_link)
  double cropbox_x_min = 0.0;
  double cropbox_x_max = 3.0;
  double cropbox_y_min = -0.75;
  double cropbox_y_max = 0.75;
  double cropbox_z_min = -0.1;
  double cropbox_z_max = 1.0;

  // 2.5D ΔZ 評価
  double grid_size = 0.05;
  double delta_z_threshold = 0.15;
  int min_points_per_cell = 2;
  // セルごとに z の上下から無視する外れ値点数 (0 または 1)
  int z_outlier_trim = 0;

  // グリッド連結成分によるクラスタリング
  double cluster_tolerance = 0.15;
  int min_cluster_cells = 2;
};

// 生バッファ上の XYZ (float32) 点群への参照
struct PointCloudView
{
  const uint8_t * data = nullptr;
  size_t num_points = 0;
  size_t point_step = 0;
  size_t x_offset = 0;
  size_t y_offset = 0;
  size_t z_offset = 0;
};

struct ObstacleCluster
{
  int num_cells = 0;
  int num_points = 0;
  float min_x = 0.0f;
  float max_x = 0.0f;
  float min_y = 0.0f;
  float max_y = 0.0f;
  float min_z = 0.0f;
  float max_z = 0.0f;
};

// 直近フレームの統計（処理時間は [ms]）
struct FrameStats
{
  size_t input_points = 0;
  size_t cropped_points = 0;
  size_t candidate_cells = 0;
  size_t output_points = 0;
  size_t num_clusters = 0;
  double accumulate_ms = 0.0;
  double judge_ms = 0.0;
  double cluster_ms = 0.0;
  double extract_ms = 0.0;
};

class ObstacleDetector
{
public:
  // 不正なパラメータは std::invalid_argument を送出する
  explicit ObstacleDetector(const ObstacleDetectionParams & params);

  static void validate(const ObstacleDetectionParams & params);

  // sensor 座標の点群を base 座標へ変換しながら障害物を検出する。
  // 結果は obstacle_points() / clusters() / stats() で参照する（次回の process まで有効）
  void process(const PointCloudView & input, const Eigen::Isometry3f & sensor_to_base);

  const std::vector<Point3> & obstacle_points() const { return obstacle_points_; }
  const std::vector<ObstacleCluster> & clusters() const { return clusters_; }
  const FrameStats & stats() const { return stats_; }

private:
  struct GridCell
  {
    int count = 0;
    float z_min1 = 0.0f;
    float z_min2 = 0.0f;
    float z_max1 = 0.0f;
    float z_max2 = 0.0f;
    float x_min = 0.0f;
    float x_max = 0.0f;
    float y_min = 0.0f;
    float y_max = 0.0f;
  };

  struct CroppedPoint
  {
    Point3 p;
    int cell;
  };

  // ラベル: 障害物セル候補で未探索 / 障害物ではない or 棄却 / 0 以上はクラスタ番号
  static constexpr int kUnvisited = -1;
  static constexpr int kNone = -2;

  void accumulate(const PointCloudView & input, const Eigen::Isometry3f & sensor_to_base);
  void judgeCells();
  void clusterCells();
  void extractPoints();

  ObstacleDetectionParams params_;
  int grid_width_;
  int grid_height_;
  int link_radius_;
  int min_points_needed_;

  // フレーム間で確保済みバッファを使い回す
  std::vector<GridCell> grid_;
  std::vector<int> labels_;
  std::vector<int> touched_cells_;
  std::vector<int> candidate_cells_;
  std::vector<int> stack_;
  std::vector<int> component_;
  std::vector<CroppedPoint> cropped_;

  std::vector<Point3> obstacle_points_;
  std::vector<ObstacleCluster> clusters_;
  FrameStats stats_;
};

}  // namespace obstacle_detection
