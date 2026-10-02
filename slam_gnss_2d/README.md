# slam_gnss_2d

2D LiDAR・オドメトリ・GNSS (RTK) を統合する、C++ の 2D 占有格子地図の生成 (SLAM)。プロジェクトで使う SLAM の現役の実装。
ポーズグラフを GTSAM / iSAM2 で最適化し、GNSS を絶対位置の拘束として使う。地図は UTM の東・北に揃って作られる。
走行時には、`gnss_transform.yaml` を使って GNSS を地図の座標に変換するブリッジ (`slam_gnss_nav_bridge_node`) が、自己位置推定に GNSS を渡す。

- 設計と構成: [doc/design.md](./doc/design.md)
- GNSS の拘束: [doc/gnss_algorithm.md](./doc/gnss_algorithm.md)

## パッケージ

| ディレクトリ | 内容 |
| :--- | :--- |
| `slam_gnss_2d/` | SLAM のエンジンとノード (C++)。`src/`、`include/slam_gnss_2d/`、`launch/`、`params/`、`rviz/`、`test/` |
| `slam_gnss_2d_msgs/` | メッセージ・サービスの定義 (`PoseGraphDiff`、`GetPoseGraph`、`SaveSlamMap`) |

`slam_gnss_2d/` 直下には、単体で開発するための `Dockerfile`、`compose.yaml`、`Makefile` もある ([doc/development.md](./doc/development.md#単体の環境で開発する))。

## ノード

| 実行ファイル | 内容 |
| :--- | :--- |
| `slam_node` | オンラインの SLAM (ノード名 `slam_gnss_2d_node`) |
| `slam_offline_node` | rosbag2 を 1 スキャンずつ高速に処理するオフラインの SLAM |
| `reoptimize_node` | 保存した `pose_graph.json` を読み込み、再最適化と再描画をする |
| `pose_graph_preview_node` | 保存済みのポーズグラフと地図を、RViz2 などで見るために配信する |
| `slam_gnss_nav_bridge_node` | 走行時に、生の GNSS を地図の座標に変換して `/odom/gps` に配信する ([doc/nav_bridge.md](./doc/nav_bridge.md)) |
| `anchor_publisher_node` | `gnss_transform.yaml` のアンカーを `/slam_gnss_2d/anchor` に配信する |
| `save_slam_map_cli` | 地図・`gnss_transform.yaml`・`pose_graph.json` を一括で保存するコマンド |

## launch

| ファイル | 内容 | 主な引数 |
| :--- | :--- | :--- |
| `bringup_slam_gnss_2d.launch.py` | `slam_node` と RViz2。センサのドライバは含まない | `simulation` (`$SIMULATION`)、`rviz` (`$USE_RVIZ`)、`rviz_param` (`slam_gnss_2d.rviz`)、`params_file` |
| `offline_slam_gnss_2d.launch.py` | `slam_offline_node` と RViz2 | `bag_path`、`start_time`、`end_time`、`skip_intermediate_rendering`、`rviz`、`params_file` |
| `reoptimize.launch.py` | `reoptimize_node` | `input_dir` (必須)、`bag_path`、`save_dir`、`params_file` |
| `preview.launch.py` | `pose_graph_preview_node` | `pose_graph_file` (必須) |

ルートの make と、launch の対応は次のとおり ([doc/commands.md](../doc/commands.md))。

| コマンド | launch |
| :--- | :--- |
| `make slam-gnss-2d` | `slam_gnss_2d` の `bringup_slam_gnss_2d.launch.py` |
| `make offline-slam-gnss-2d BAG=<bag>` | `mg_bringup` の `offline_slam_gnss_2d.launch.py` (`slam_gnss_2d` のものに、description と前処理を足したもの) |
| `make reoptimize` | `reoptimize.launch.py` |

センサのドライバと SLAM を一緒に起動する launch は、`mg_bringup` の `bringup_slam_gnss_2d.launch.py` ([mg_bringup](../mg_bringup/README.md))。

## トピックとサービス

`slam_node` が出すもの。

| 名前 | 型 | 内容 |
| :--- | :--- | :--- |
| `map` | `nav_msgs/OccupancyGrid` (transient_local) | 占有格子の地図 |
| `slam_gnss_2d/path` | `nav_msgs/Path` | 推定した軌跡 |
| `slam_gnss_2d/path_before_optimize` | `nav_msgs/Path` | 最適化前の軌跡 |
| `slam_gnss_2d/pose_graph` | `visualization_msgs/MarkerArray` | ポーズグラフの可視化 |
| `slam_gnss_2d/pose_graph_diff` | `slam_gnss_2d_msgs/PoseGraphDiff` | ポーズグラフの差分 (Web UI が使う) |
| `slam_gnss_2d/anchor` | `sensor_msgs/NavSatFix` (transient_local) | アンカーの緯度経度 |
| TF `map→odom` | | 推定した地図の姿勢 |
| サービス `slam_gnss_2d/save_slam_map` | `slam_gnss_2d_msgs/SaveSlamMap` | 地図などの保存 |
| サービス `slam_gnss_2d/get_pose_graph` | `slam_gnss_2d_msgs/GetPoseGraph` | ポーズグラフの全体の取得 |

入力は `topics.scan` (`/scan_top_lidar`)、`topics.odom` (`/odom`)、`gnss.topics` (`/navpvt` または `/gps/fix`)。

## 使い方

### オンラインの SLAM

```bash
ros2 launch slam_gnss_2d bringup_slam_gnss_2d.launch.py rviz:=true
ros2 launch slam_gnss_2d bringup_slam_gnss_2d.launch.py simulation:=true rviz:=true   # シミュレーション
```

GNSS を使うので、走行を始めてから、`gnss.anchor.init_distance_m` (30 m) 走るまで、地図の向きが決まらない。

### オフラインの再処理 (rosbag2)

リアルタイムの再生を待たずに、bag のセンサデータをステップ駆動で高速に処理する。

```bash
ros2 launch slam_gnss_2d offline_slam_gnss_2d.launch.py bag_path:=/root/ros2_data/rosbag/sample_bag rviz:=true
# bag の開始 10 秒後から 60 秒目までを処理
ros2 launch slam_gnss_2d offline_slam_gnss_2d.launch.py bag_path:=<bag> start_time:=10.0 end_time:=60.0
```

- `start_time` / `end_time`: bag の開始からの経過秒 [s]。`0.0` で、先頭 / 末尾。
- `skip_intermediate_rendering` (既定 `true`): 途中の地図の描画を止めて、終了時に描く (高速)。RViz2 で処理中の地図を見るときは `false`。

終了時に、処理の段階ごとの所要時間が、`[timing]` のログに出る。

### ポーズグラフの再最適化

保存したポーズグラフを読み込み、ループの追加の探索や一括最適化 (Levenberg-Marquardt) をやり直す。

```bash
ros2 launch slam_gnss_2d reoptimize.launch.py input_dir:=/root/ros2_data/slam_maps/latest bag_path:=<bag> rviz:=true
```

`rviz` 引数はない。結果は `save_dir` (省略時は `input_dir` と同じ) に保存される。

### 地図とパラメータの保存

実行中、またはオフライン処理の完了後に、保存する。

```bash
# カレントディレクトリに、4 つのファイルをまとめて保存
ros2 run slam_gnss_2d save_slam_map_cli
ros2 run slam_gnss_2d save_slam_map_cli -d /root/ros2_data/slam_maps/experiment_01
```

| オプション | 内容 |
| :--- | :--- |
| `-d`, `--dir <PATH>` | 保存先 (省略時はカレントディレクトリ) |
| `-m`, `--map-topic <TOPIC>` | 地図のトピック (既定 `/map`) |
| `-s`, `--service <NAME>` | 保存のサービス (既定 `/slam_gnss_2d/save_slam_map`) |
| `--no-pgm` | `map.pgm` / `map.yaml` の出力を省く |
| `-t`, `--timeout <SEC>` | タイムアウト (既定 5.0) |

サービスを直接呼ぶこともできる。この場合、`gnss_transform.yaml` と `pose_graph.json` だけが保存される (`map.pgm` と `map.yaml` を含めるには、`save_slam_map_cli` を使う)。

```bash
ros2 service call /slam_gnss_2d/save_slam_map slam_gnss_2d_msgs/srv/SaveSlamMap "{map_dir: '/root/ros2_data/slam_maps/latest'}"
```

### 出力ファイル

| ファイル | 内容 | 使うもの |
| :--- | :--- | :--- |
| `map.pgm` | 占有格子の画像 (白: 空き、黒: 障害物、灰: 未探索) | Nav2 の `map_server` |
| `map.yaml` | 地図の解像度・原点・閾値 | Nav2 の `map_server` |
| `gnss_transform.yaml` | 地図の原点と、地球の座標 (WGS84 / UTM) の対応 ([doc/gnss_transform.md](./doc/gnss_transform.md)) | `slam_gnss_nav_bridge_node` |
| `pose_graph.json` | 全キーフレームの位置・姿勢、エッジの情報行列 | `reoptimize_node`、Web UI |

## パラメータ

`slam_gnss_2d/params/slam_gnss_2d.yaml` (SLAM)、`slam_gnss_2d/params/nav_bridge.yaml` (ナビゲーションのブリッジ)。全項目と、既定値の理由は [doc/parameters.md](./doc/parameters.md)。

主な既定の構成:

- スキャンマッチング: `multi_res_csm` (2 段階の相関探索)。参照は直近 30 キーフレームのローカルマップ
- キーフレーム: 0.5 m / 0.5 rad ごと
- ループクロージャ: **無効** (`loop_closure.enabled: false`)
- GNSS: 有効。NavPVT 入力。アンテナのレバーアーム (0.26, -0.13) m を補正する
- 最適化: iSAM2

## テスト

C++ の gtest が 16 本 ([doc/development.md](./doc/development.md#テスト))。ルートの `make test` には含まれない。

## 依存

`rclcpp`、`ublox_msgs`、`rosbag2_cpp`、GTSAM、GeographicLib、nanoflann、OpenMP、yaml-cpp、nlohmann-json、OpenCV (Docker の `runtime-base` と、単体の `Dockerfile` で入れる)。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/design.md](./doc/design.md) | 構成、データ型、処理の流れ、スキャンマッチング、ループクロージャ、設計の決定 |
| [doc/gnss_algorithm.md](./doc/gnss_algorithm.md) | アンカー、初期方位、prior、動的な再アンカー |
| [doc/gnss_transform.md](./doc/gnss_transform.md) | `gnss_transform.yaml` と座標変換 |
| [doc/nav_bridge.md](./doc/nav_bridge.md) | ナビゲーション用の GNSS ブリッジと `/odom/gps` |
| [doc/parameters.md](./doc/parameters.md) | 全パラメータ |
| [doc/development.md](./doc/development.md) | 設計のルール、環境、テスト |
