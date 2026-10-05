# mg_slam

slam_toolbox による 2D SLAM の launch と設定、地図のプレビュー、rosbag の収録。

> **GNSS 拘束付きの SLAM は、[slam_gnss_2d](../slam_gnss_2d/README.md) (C++) が現役の実装。** このパッケージの `scripts/slam_gnss_2d/` (Python) は旧実装で、`map_preview` のためだけに使われている ([旧 Python 版](#旧-python-版))。

## launch

| ファイル | 内容 | compose サービス |
| :--- | :--- | :--- |
| `bringup_slam_toolbox.launch.py` | slam_toolbox (オンラインの非同期) と `map_saver`、軌跡の配信、RViz2、rosbag の収録 | `slam` (`mg_bringup` の `bringup_slam_toolbox.launch.py` 経由) |
| `map_preview.launch.py` | 保存済みの SLAM の出力 (地図・アンカー・ポーズグラフ) を配信して、見る | `map-preview` |
| `record_bag.launch.py` | rosbag の収録 (`mg_utils` の同名の launch とは別のファイル) | なし |
| `bringup_slam_gnss_2d.launch.py`、`offline_slam_gnss_2d.launch.py`、`reoptimize_slam.launch.py` | 旧 Python 版の SLAM | なし (使わない) |

### bringup_slam_toolbox.launch.py

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `slam_params_file` | `params/slam_toolbox.yaml` | slam_toolbox のパラメータ |
| `simulation` | `$SIMULATION` | `use_sim_time` |
| `log_level` | `info` | ログのレベル |
| `rviz` | `$USE_RVIZ` | RViz2 の起動 |
| `rviz_param` | `rviz.rviz` | RViz2 の設定ファイル (`rviz/` の名前) |
| `record_bag` | `false` | rosbag の収録 (`mg_utils` の `record_bag.launch.py`。`ROSBAG_PATH` が必須) |

起動するもの: slam_toolbox (`slam_toolbox` パッケージの launch を include)、`map_saver_server` (Nav2、ライフサイクルで管理)、`actual_path_publisher.py` (`actual_path`: TF から作った実際の軌跡)、RViz2 (任意)。
センサのドライバは、`mg_bringup` の `bringup_slam_toolbox.launch.py` が、先に起動する ([mg_bringup](../mg_bringup/README.md))。

## 使い方

### slam_toolbox で地図を作る

```bash
make slam                       # ドライバ + slam_toolbox。RViz2 は起動しない
make rviz2-slam                 # (別端末) RViz2
```

slam_toolbox の RViz2 プラグインで、地図の保存や、ループクロージャの操作ができる ([doc/slam_toolbox.md](./doc/slam_toolbox.md))。
`record_bag:=true` を付けると、`ROSBAG_PATH` の下に、`<YYYYMMDD>_bag/<接頭辞><番号>` の bag を収録する。

```bash
make slam OPTS="record_bag:=true"
```

収録するトピックは `params/record_topic_list.txt` (30 トピック)。GNSS を使うときは、`/gps/fix` または `/navpvt` が必要。
手動の収録は、`tools/scripts/record.sh` ([tools の README](../tools/README.md))。

### 保存済みの地図のプレビュー

make ターゲットはない。compose のサービス `map-preview` を使う。

```bash
SLAM_MAP_DIR=/root/ros2_data/map/<地図名> docker compose up map-preview
```

`SLAM_MAP_DIR` に、`map.yaml`・`gnss_transform.yaml`・`pose_graph.json` がある SLAM の出力を指定すると、`map_server`、`anchor_publisher`、`pose_graph_preview` が起動し、RViz2 や Web UI で見られる。
Web UI の地図の管理 (`mg_system_manager` の `/maps`) が、このサービスを起動する。

## slam_toolbox の設定

`params/slam_toolbox.yaml` の要点。

| 項目 | 値 |
| :--- | :--- |
| ソルバ | Ceres (`SPARSE_NORMAL_CHOLESKY`、`SCHUR_JACOBI`、`LEVENBERG_MARQUARDT`) |
| モード | `mapping` (`enable_interactive_mode: true`) |
| フレーム | `odom` / `map` / `base_footprint`、スキャンは `/scan_top_lidar` |
| 地図 | 解像度 0.05 m、`max_laser_range` 30 m、`map_update_interval` 5 s |
| キーフレーム | `minimum_travel_distance` 0.5 m、`minimum_travel_heading` 0.5 rad |
| ループクロージャ | 有効 (`loop_search_maximum_distance` 3.0 m、`loop_match_minimum_chain_size` 10) |

内部のアルゴリズムと、パラメータの意味は、[doc/slam_toolbox.md](./doc/slam_toolbox.md)。

## 構成

```
launch/    bringup_slam_toolbox / map_preview / record_bag、旧 Python 版の launch
params/    slam_toolbox.yaml、record_topic_list.txt、slam_gnss_2d.yaml (旧 Python 版)
rviz/      rviz.rviz (slam_toolbox 用)、slam_gnss_2d.rviz (旧 Python 版)
scripts/   actual_path_publisher.py、slam_gnss_2d/ (旧 Python 版)
doc/       slam_toolbox.md
```

## 旧 Python 版

`scripts/slam_gnss_2d/` は、GNSS 拘束付き SLAM の旧実装 (Python)。現在の実装 ([slam_gnss_2d](../slam_gnss_2d/README.md)、C++) に置き換わっていて、次の用途だけで使われている。

- `map_preview.launch.py` の `anchor_publisher_node.py` と `pose_graph_preview_node.py` (`mg_msgs/PoseGraphDiff` と `GetPoseGraph` を使う)

その他の、旧 Python 版の `slam_node.py`・`slam_offline_node.py`・`reoptimize_node.py` と、その launch は、compose と make から使われない。
次の点に、注意する。

- ノード名とトピック・サービス名が、C++ 版と同じ (`slam_gnss_2d_node`、`/slam_gnss_2d/...`)。同時に起動すると、衝突する。
- 地図の保存のサービスの型が違う。旧 Python 版は `std_srvs/Trigger`、C++ 版は `slam_gnss_2d_msgs/SaveSlamMap`。
- Python のモジュール `slam_gnss_2d` を、このパッケージがインストールする。ROS のパッケージ `slam_gnss_2d` と、名前が同じ。
- パラメータ (`params/slam_gnss_2d.yaml`) の値は、C++ 版の `slam_gnss_2d/params/slam_gnss_2d.yaml` とは、別に管理されている。C++ 版の値と同じとは限らない。

旧 Python 版の設計の説明は、C++ 版に合わせた [slam_gnss_2d/doc/](../slam_gnss_2d/doc/design.md) に移した。

## 依存

`package.xml`: `mg_utils`、`navigation2`、`nav2_bringup`、`slam_toolbox`、`tf2_ros`。旧 Python 版は、`requirements.txt` (`gtsam`、`numpy<2.0`、`scipy`、`opencv-python`、`pyproj`) を使う。
テストは `make test pkg=mg_slam` (旧 Python 版のもの)。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/slam_toolbox.md](./doc/slam_toolbox.md) | slam_toolbox の内部の処理 (Karto、スキャンマッチング、ポーズグラフ、ループクロージャ)、RViz プラグイン、運用のツール |
