# mg_drivers

LiDAR / DepthCam / GPS / IMU / モータドライバ群と、Realsense 点群から立体障害物を検出する
`obstacle_detection_3d_node` を含むパッケージ。本 README は `obstacle_detection_3d_node` を中心に記載する。

## obstacle_detection_3d_node

Realsense D435i の点群（`points`、既定 remap 先 `/rs_d435i/depth/color/points`）を base_link に変換し、
2.5D グリッド上の ΔZ（セル内の高さ差）で立体障害物を検出する。検出対象はコーンに限らず、壁・人・箱など一般の障害物。

パイプライン:

```
点群 → base_link へ変換（TFは待たず最新値を使用） → ROI（cropbox）内の点をグリッドに集計
  → セルの ΔZ が閾値を超えたセルを障害物候補に → 2D グリッド上の連結成分でクラスタリング
  → 小さいクラスタを除去 → ~/points_obstacle（XYZ）, ~/cluster_markers を publish
```

出力は検出ゼロのフレームでも必ず publish する（Nav2 側のコストマップがクリアされるように）。
クラスタの点数に上限はない（大きな障害物も欠落しない）。

### パラメータ

[`params/obstacle_detection.yaml`](./params/obstacle_detection.yaml) を編集し、**ノードを再起動**して調整する。
ランタイムの `ros2 param set` には対応していない。起動時に採用値が `INFO` ログに出力され、
不正な値（`grid_size<=0`、min>max など）は理由付きのエラーでノードが終了する。

主なパラメータ（yaml 内のコメントに単位・効果を記載）:

| パラメータ | 効果 |
| --- | --- |
| `stride` | 入力点の間引き。速度に効く |
| `cropbox_*` | ROI [m]。狭めるほど速い |
| `grid_size` | グリッド解像度 [m]。粗いほど速い |
| `delta_z_threshold` | 障害物判定の高さ差閾値 [m] |
| `min_points_per_cell` | セル判定に必要な最小点数 |
| `z_outlier_trim` | セルごとに z の外れ値を 1 点だけ無視するか (0/1) |
| `cluster_tolerance` | 障害物セルを連結する距離 [m] |
| `min_cluster_cells` | クラスタとして残す最小セル数（孤立ノイズ除去） |
| `publish_markers` / `publish_stats` / `stats_period` | Marker・統計ログの出力可否と間隔 |

### 処理速度の確認（統計ログ）

`publish_stats: true`（既定）のとき、`stats_period` 秒ごとに次のログが出る。

```
stats [5.9 fps, 6 frames] time avg/max [ms]: tf 0.02/0.02 accumulate 0.76/0.89 judge 0.00/0.00
  cluster 0.00/0.00 extract 0.03/0.05 publish 0.04/0.05 total 0.90/1.02 latency 12.3/18.0
  | points avg: input 250000 cropped 4000 output 500 | candidate_cells 15 clusters 2.0
```

- `tf`〜`extract` は検出処理内の各段の時間、`publish` は PointCloud2/Marker の構築・publish 時間、`total` はコールバック全体。
- `latency` は `now - header.stamp` のため、**rosbag 再生時は無意味な値になる**（bag のタイムスタンプと壁時計がずれるため）。
- 速度を比較したいときは `total` の平均/最大を見る。

## rosbag でのリプレイ確認

bag に点群 (`PointCloud2`) が含まれない場合、深度画像から点群を復元しながら確認する。
既存の `rosbag-replay` サービス（bag の配信のみ）とは別に、
点群復元・障害物検出・RViz をまとめて起動する `obstacle-detection-replay` サービスを用意している。

### 準備（`.env`）

```bash
ROSBAG_FILE=${ROSBAG_PATH}/TC2026/20260913/record_all_20260913_070735
ROSBAG_TOPICS=/camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw /camera/camera/color/camera_info /tf /tf_static
```

- `/tf_static` は再生開始直後に 1 回しか配信されないため、必ず含める。
  `ros2 bag play --start-offset` で始めると `/tf_static` を取りこぼし、TF 解決に失敗するので使わない。
- 等速再生ではメッセージが届かないことがある（未調査）。安定しない場合は `OPTS=-r 0.5` などで速度を落とす。

### 起動（2 端末）

```bash
# 端末1: bag 配信
make rosbag-replay

# 端末2: 点群復元 + 障害物検出 + RViz
make obstacle-detection-replay
```

`make obstacle-detection-replay` は `ros2 launch mg_drivers obstacle_detection_replay.launch.py` を実行する。
主な引数（`OPTS` 経由、例 `make obstacle-detection-replay OPTS="rviz:=false use_color:=true"`）:

| 引数 | 既定 | 説明 |
| --- | --- | --- |
| `use_color` | `false` | カラー付き点群を復元するか。RGB は RViz の Image 表示で確認できるため既定は無効 |
| `param_file` | `params/obstacle_detection.yaml` | 障害物検出のパラメータYAML。調整用の別ファイルを指定して比較できる |
| `rviz` | `true` | RViz を起動するか |
| `rviz_config` | `rviz/obstacle_detection_replay.rviz` | RViz の設定ファイル |

RViz（[`rviz/obstacle_detection_replay.rviz`](./rviz/obstacle_detection_replay.rviz)、Fixed Frame: `base_link`）には次を表示する:

- RGB画像: `/camera/camera/color/image_raw`
- Depth画像: `/camera/camera/depth/image_rect_raw`
- 復元点群: `/rs_d435i/depth/color/points`（グレー）
- 検出点: `/obstacle_detection_3d_node/points_obstacle`（赤）
- 検出オブジェクト: `/obstacle_detection_3d_node/cluster_markers`（bbox とクラスタ番号/点数）

パラメータを変えて比較する場合は、端末2を Ctrl-C で止め、yaml を編集（または `param_file` を切り替え）してから
`make obstacle-detection-replay` を再実行する。端末1の bag 配信は入れ直すか、`ros2 bag play` を最初から再生する。

### ベースライン比較（数値）

**自動化された比較ツールは未整備。** `tools/scripts/` の `eval_slam.py` / `run_slam_variant.sh` は SLAM 専用で対象外。
整備済みの手段は上記の統計ログのみ。同一 bag・同一入力に対し `param_file` を切り替えて
`obstacle-detection-replay` を実行し、ログの `stats` 行（`total` の平均/最大、`output` 点数、`clusters` 数）を
手動で比較する。過去の PCL 実装との比較は改修時にコンテナ内でアドホックに行ったもので、再現可能な手順としては整備していない。

## テスト

```bash
# C++ (検出ロジックの gtest。13 ケース)
docker exec <develop コンテナ> bash -c \
  "cd /root/ros2_ws && colcon build --packages-select mg_drivers --cmake-args -DBUILD_TESTING=ON && \
   ./build/mg_drivers/test_obstacle_detector"

# Python (点群復元の点群配列生成。mg_utils 側)
make test pkg=mg_utils
```
