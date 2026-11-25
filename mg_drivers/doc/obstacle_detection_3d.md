# 3D 障害物検出 (obstacle_detection_3d_node)

RealSense D435i の点群 (`points`) を処理の座標系 (`target_frame`) に変換し、2.5D グリッド上の ΔZ (セル内の高さ差) で立体障害物を検出する。
検出対象はコーンに限らず、壁・人・箱など一般の障害物と、上 LiDAR の面より低い低床障害物。

## 入出力

| 種別 | トピック | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| 入力 | `points` | `sensor_msgs/PointCloud2` | launch で `/camera/camera/depth/color/points` に remap している (`obstacle_detection_3d.launch.py`)。実機・シミュレータ・再生の復元点群で同じ名前 |
| 出力 | `~/points_obstacle` | `sensor_msgs/PointCloud2` | 検出した障害物の点 (XYZ) |
| 出力 | `~/cluster_markers` | `visualization_msgs/MarkerArray` | クラスタの bbox と、番号・点数 (`publish_markers: true` のとき) |

- 出力は、検出ゼロのフレームでも必ず publish する (Nav2 のコストマップがクリアされるように)。
- クラスタの点数に上限はない (大きな障害物も欠落しない)。
- `use_sensor_data_qos` で、入出力の QoS を `SensorDataQoS` に切り替える。`false` のときは、reliable の QoS。

## パイプライン

```
点群 → target_frame へ変換 (TF は待たず最新値を使用) → ROI (cropbox) 内の点をグリッドに集計
  → セルの ΔZ が閾値を超えたセルを障害物候補に → 2D グリッド上の連結成分でクラスタリング
  → 小さいクラスタを除去 → ~/points_obstacle、~/cluster_markers を publish
```

## パラメータ

`params/obstacle_detection.yaml` が既定。起動時に、採用した値が `INFO` ログに出る。不正な値 (`grid_size<=0`、min>max など) は、理由付きのエラーでノードが終了する。

実機での調整のため、検出のパラメータと `target_frame` は、実行中に `ros2 param set` で変えられる (次のフレームから効く。yaml は変わらないので、決めた値は yaml に書き戻す)。値はまとめて検証し、不正なら理由を返して何も変えない (ログに `rejected parameter change`)。受け付けると `params updated` のログに全体の値が出る。`use_sensor_data_qos`・`publish_markers`・`publish_stats`・`stats_period` は起動時だけ。

```bash
ros2 param set /obstacle_detection_3d_node delta_z_threshold 0.15
```

主なパラメータ (単位と効果は、yaml のコメントにある)。

| パラメータ | 効果 |
| :--- | :--- |
| `target_frame` | 処理と出力の座標系。`cropbox_*` はこの座標系の値で、地面が z=0 にある前提 (地面の高さにある `base_footprint` を使う) |
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

値どうしの関係:

- **検出できる最小の高さ**は、`cropbox_z_min` + `delta_z_threshold`。ROI の下限より下の点 (地面) は集計しないので、物体の ROI 内の部分の高さが閾値を超える必要がある。低床障害物を拾うときは、この和を対象の高さより十分小さくする。下げすぎると、地面の凹凸・深度のノイズ・物体の輪郭の飛び画素で誤検出する (`z_outlier_trim` で飛び画素を抑えられる)。対象と同程度の縁石や段差も、障害物として検出する
- `cropbox_z_max` は、STVL の消去の錐台の上端 (手前の面) 以下にする。錐台より上でマークした点は消去されず、時間切れまで残る (立ち去った人の上半身の跡で、走行の再開が遅れる)。これより高い物は、地面から立ち上がる部分で検出する。地面から浮いていて、この高さと上 LiDAR の面の間だけにある物は、どちらにも見えない
- `cropbox_x_*`・`cropbox_y_*` は、コストマップの STVL (`obstacle_stvl_layer`) の消去の錐台を含むように取る。錐台が ROI からはみ出すと、検出していない場所の障害物を消してしまう ([costmap_and_collision.md](../../mg_navigation/doc/costmap_and_collision.md#コストマップ))

## 処理速度の確認 (統計ログ)

`publish_stats: true` のとき、`stats_period` 秒ごとに次のログが出る。

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

| 引数 | 内容 |
| :--- | :--- |
| `use_sim_time` | シミュレーション時刻を使うか |
| `use_sensor_data_qos` | QoS の切り替え |
| `param_file` | パラメータの YAML |

## 実機での確認と調整

低床障害物を検出するための値 (ROI の下限と ΔZ の閾値) と、コストマップの STVL の消去の錐台は、シミュレータと記録の再生でしか確かめていない。実機では、次の順に確かめて調整する。決めた値は yaml に書き戻す。

### 機能のオンオフ

| 対象 | 切り替え方 | 効く時期 |
| :--- | :--- | :--- |
| カメラの層 (STVL) をコストマップから外す | `ros2 param set /local_costmap/local_costmap obstacle_stvl_layer.enabled false` (global は `/global_costmap/global_costmap`) | すぐ |
| 検出の感度・範囲 | `ros2 param set /obstacle_detection_3d_node <名前> <値>` (上の「パラメータ」) | 次のフレーム |
| 錐台の視野・奥行き、観測の高さの範囲 | `ros2 param set /local_costmap/local_costmap obstacle_stvl_layer.obstacle_points.<名前> <値>` (`vertical_fov_angle`、`horizontal_fov_angle`、`min_z`、`max_z`、`min_obstacle_height`、`max_obstacle_height`。global の観測源の名前は `pointcloud`) | すぐ |
| 錐台の消去の速さ (`decay_acceleration`)、時間切れ (`voxel_decay`) | `mg_navigation/params/nav2_params.yaml` を編集して、ナビゲーションを起動し直す (実行中には変えられない) | 起動し直し |
| カメラと検出をまとめて止める | ナビゲーションの launch 引数 `use_realsense:=false` | 起動し直し |
| カメラ・検出の途絶で走行を一時停止する監視 (`driver_watchdog` の `obstacle_camera`) をやめる | launch 引数 `watch_obstacle_camera:=false` ([driver_watchdog](./driver_watchdog.md)) | 起動し直し |

### 確かめる順と、見るもの

RViz (`mg_navigation/rviz/rviz.rviz`) で、点群 (`/camera/camera/depth/color/points`)、検出 (`/obstacle_detection_3d_node/points_obstacle` と `cluster_markers`)、STVL のボクセル (`/local_costmap/voxel_grid`。global は `publish_voxel_map` が false なので出ない) を重ねて見る。

1. **カメラの取り付け角度**: 平らな地面だけの前方で、点群の地面が、`base_footprint` の高さ 0 に、距離によらず乗っているかを見る。遠くほど上 (または下) にずれるなら、URDF のカメラの角度 (`mg_description` の `realsense_d435i` の `rpy`) が実物と違う。ずれたままだと、遠くの地面が ROI の下限を越えて、判定に入りやすくなる
2. **点群の実際の視野と錐台**: 前方の左右と上下の端に物を置き、点群が届く範囲より錐台 (ボクセルが消える範囲) が狭いかを見る。物を除いたときに、点群の外でボクセルが消えるなら、錐台が広すぎる。視野の角度を下げる。`test/test_nav2_params_consistency.py` の点群の視野の定数も、測った値に直す
3. **低床障害物の検出と停止**: 対象の高さの箱を進路に置き、何 m 手前から検出され、どこで止まり、避けるかを見る。手前で止めて待っても (RPP の待ち)、走り出して箱に当たらないかを見る (箱がカメラの視野の下に入っても、`voxel_decay` の間は覚えている)。見落とすなら ΔZ の閾値か ROI の下限を下げる
4. **立ち去る人**: 正面に人が立ち、立ち去ったあと、何秒で走り出すかを見る。遅いなら、ボクセルが錐台の外に残っていないか (ROI の上限と錐台の上端の関係) と、`decay_acceleration` を見る
5. **走路の脇と路面**: 縁石・草・側溝の格子・タイルの目地などで、検出が出るかと、走行 (減速・停止・迂回) にどう効くかを見る。止めたくない物で止まるなら、ΔZ の閾値・`min_cluster_cells`・`min_points_per_cell`・`z_outlier_trim` を上げる。どの値で、何が検出から外れるかを記録する
6. **横・後ろの跡**: リカバリー (後退・首振り) の前後で、視野の外に残ったボクセル (人や壁の跡) が、再開を遅らせていないかを見る。遅らせるなら local の `voxel_decay` を短くする (短くしすぎると、3 の箱を待つ間に忘れる)

調べる走行は `tools/scripts/record.sh -m all` で記録する (深度画像と検出結果が入る)。あとで「rosbag でのリプレイ確認」の手順で、パラメータを変えて比べられる。

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

| 引数 | 内容 |
| :--- | :--- |
| `use_color` | カラー付きの点群を復元するか。RGB は RViz2 の Image 表示で確認できる |
| `param_file` | 障害物検出のパラメータ YAML。調整用の別ファイルを指定して比べられる |
| `rviz` | RViz2 を起動するか |
| `rviz_config` | RViz2 の設定ファイル |

RViz2 (`rviz/obstacle_detection_replay.rviz`、Fixed Frame は `base_footprint`) には、次を表示する。

- RGB 画像: `/camera/camera/color/image_raw`
- Depth 画像: `/camera/camera/depth/image_rect_raw`
- 復元した点群: `/camera/camera/depth/color/points` (グレー)
- 検出した点: `/obstacle_detection_3d_node/points_obstacle` (赤)
- 検出したオブジェクト: `/obstacle_detection_3d_node/cluster_markers` (bbox と、クラスタの番号・点数)

パラメータを変えて比べるときは、端末 2 を Ctrl-C で止め、yaml を編集 (または `param_file` を切り替え) してから、`make obstacle-detection-replay` を再実行する。端末 1 の bag の配信は、入れ直すか、`ros2 bag play` を最初から再生する。

### ベースラインとの比較 (数値)

**自動化された比較ツールは未整備。** `tools/scripts/` の `eval_slam.py` と `run_slam_variant.sh` は SLAM 専用で、対象外。
整備済みの手段は、上の統計ログだけ。同じ bag・同じ入力に対して `param_file` を切り替えて `obstacle-detection-replay` を実行し、ログの `stats` 行 (`total` の平均と最大、`output` の点数、`clusters` の数) を手で比べる。
過去の PCL 実装との比較は、改修のときにコンテナ内でアドホックに行ったもので、再現できる手順としては整備していない。
