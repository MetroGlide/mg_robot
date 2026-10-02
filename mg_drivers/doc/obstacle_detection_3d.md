# 3D 障害物検出 (obstacle_detection_3d_node)

RealSense D435i の点群 (`points`) を `base_link` に変換し、2.5D グリッド上の ΔZ (セル内の高さ差) で立体障害物を検出する。
検出対象はコーンに限らず、壁・人・箱など一般の障害物。

## 入出力

| 種別 | トピック | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| 入力 | `points` | `sensor_msgs/PointCloud2` | launch で `/rs_d435i/depth/color/points` に remap している (`obstacle_detection_3d.launch.py`) |
| 出力 | `~/points_obstacle` | `sensor_msgs/PointCloud2` | 検出した障害物の点 (XYZ) |
| 出力 | `~/cluster_markers` | `visualization_msgs/MarkerArray` | クラスタの bbox と、番号・点数 (`publish_markers: true` のとき) |

- 出力は、検出ゼロのフレームでも必ず publish する (Nav2 のコストマップがクリアされるように)。
- クラスタの点数に上限はない (大きな障害物も欠落しない)。
- `use_sensor_data_qos` (既定 `false`) で、入出力の QoS を `SensorDataQoS` に切り替える。`false` のときは、depth 10 の reliable。

## パイプライン

```
点群 → base_link へ変換 (TF は待たず最新値を使用) → ROI (cropbox) 内の点をグリッドに集計
  → セルの ΔZ が閾値を超えたセルを障害物候補に → 2D グリッド上の連結成分でクラスタリング
  → 小さいクラスタを除去 → ~/points_obstacle、~/cluster_markers を publish
```

## パラメータ

`params/obstacle_detection.yaml` を編集し、**ノードを再起動**して調整する。実行中の `ros2 param set` には対応していない。
起動時に、採用した値が `INFO` ログに出る。不正な値 (`grid_size<=0`、min>max など) は、理由付きのエラーでノードが終了する。

主なパラメータ (単位と効果は、yaml のコメントにある)。

| パラメータ | 効果 |
| :--- | :--- |
| `stride` | 入力点の間引き。速度に効く |
| `cropbox_*` | ROI [m]。狭めるほど速い |
| `grid_size` | グリッドの解像度 [m]。粗いほど速い |
| `delta_z_threshold` | 障害物と判定する高さ差の閾値 [m] |
| `min_points_per_cell` | セルを判定するのに必要な最小点数 |
| `z_outlier_trim` | セルごとに z の外れ値を 1 点だけ無視するか (0/1) |
| `cluster_tolerance` | 障害物セルを連結する距離 [m] |
| `min_cluster_cells` | クラスタとして残す最小セル数 (孤立ノイズの除去) |
| `publish_markers` / `publish_stats` / `stats_period` | Marker と統計ログの出力の可否と間隔 |
| `use_sensor_data_qos` | QoS の切り替え (上記) |

## 処理速度の確認 (統計ログ)

`publish_stats: true` (既定) のとき、`stats_period` 秒ごとに次のログが出る。

```
stats [5.9 fps, 6 frames] time avg/max [ms]: tf 0.02/0.02 accumulate 0.76/0.89 judge 0.00/0.00
  cluster 0.00/0.00 extract 0.03/0.05 publish 0.04/0.05 total 0.90/1.02 latency 12.3/18.0
  | points avg: input 250000 cropped 4000 output 500 | candidate_cells 15 clusters 2.0
```

- `tf`〜`extract` は検出処理内の各段の時間、`publish` は PointCloud2 / Marker の構築・publish の時間、`total` はコールバック全体。
- `latency` は `now - header.stamp` なので、**rosbag の再生時は無意味な値になる** (bag のタイムスタンプと壁時計がずれるため)。
- 速度を比べるときは、`total` の平均と最大を見る。

## 起動

実機の launch では、`mg_drivers/bringup_postprocess.launch.py` が、`use_realsense` が `true` のときに起動する。単体では次のとおり。

```bash
ros2 launch mg_drivers obstacle_detection_3d.launch.py
```

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `use_sim_time` | `false` | シミュレーション時刻を使うか |
| `use_sensor_data_qos` | `false` | QoS の切り替え |
| `param_file` | `params/obstacle_detection.yaml` | パラメータの YAML |

## rosbag でのリプレイ確認

bag に点群 (`PointCloud2`) が含まれないとき、深度画像から点群を復元しながら確認する。
`rosbag-replay` サービス (bag の配信のみ) とは別に、点群の復元・障害物検出・RViz2 をまとめて起動する `obstacle-detection-replay` サービスがある。

### 準備 (`.env`)

```bash
ROSBAG_FILE=${ROSBAG_PATH}/TC2026/20260913/record_all_20260913_070735
ROSBAG_TOPICS=/camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw /camera/camera/color/camera_info /tf /tf_static
```

- `/tf_static` は、再生の開始直後に 1 回しか配信されないので、必ず含める。`ros2 bag play --start-offset` で始めると `/tf_static` を取りこぼして TF の解決に失敗するので、使わない。
- 等速の再生では、メッセージが届かないことがある (原因は未調査)。安定しないときは、`OPTS="-r 0.5"` などで速度を落とす。

### 起動 (2 端末)

```bash
# 端末 1: bag の配信
make rosbag-replay

# 端末 2: 点群の復元 + 障害物検出 + RViz2
make obstacle-detection-replay
```

`make obstacle-detection-replay` は、`ros2 launch mg_drivers obstacle_detection_replay.launch.py` を実行する。
引数は `OPTS` で渡す (例: `make obstacle-detection-replay OPTS="rviz:=false use_color:=true"`)。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `use_color` | `false` | カラー付きの点群を復元するか。RGB は RViz2 の Image 表示で確認できるので、既定は無効 |
| `param_file` | `params/obstacle_detection.yaml` | 障害物検出のパラメータ YAML。調整用の別ファイルを指定して比べられる |
| `rviz` | `true` | RViz2 を起動するか |
| `rviz_config` | `rviz/obstacle_detection_replay.rviz` | RViz2 の設定ファイル |

RViz2 (`rviz/obstacle_detection_replay.rviz`、Fixed Frame は `base_link`) には、次を表示する。

- RGB 画像: `/camera/camera/color/image_raw`
- Depth 画像: `/camera/camera/depth/image_rect_raw`
- 復元した点群: `/rs_d435i/depth/color/points` (グレー)
- 検出した点: `/obstacle_detection_3d_node/points_obstacle` (赤)
- 検出したオブジェクト: `/obstacle_detection_3d_node/cluster_markers` (bbox と、クラスタの番号・点数)

パラメータを変えて比べるときは、端末 2 を Ctrl-C で止め、yaml を編集 (または `param_file` を切り替え) してから、`make obstacle-detection-replay` を再実行する。端末 1 の bag の配信は、入れ直すか、`ros2 bag play` を最初から再生する。

### ベースラインとの比較 (数値)

**自動化された比較ツールは未整備。** `tools/scripts/` の `eval_slam.py` と `run_slam_variant.sh` は SLAM 専用で、対象外。
整備済みの手段は、上の統計ログだけ。同じ bag・同じ入力に対して `param_file` を切り替えて `obstacle-detection-replay` を実行し、ログの `stats` 行 (`total` の平均と最大、`output` の点数、`clusters` の数) を手で比べる。
過去の PCL 実装との比較は、改修のときにコンテナ内でアドホックに行ったもので、再現できる手順としては整備していない。
