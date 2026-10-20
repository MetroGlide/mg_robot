# 共通ライブラリ (tools/common/)

`tools/scripts/` の CLI が使う共通モジュール。一覧は [tools の README](../README.md)。

## `tools.common.bag` (rosbag 操作)
- **`detect_storage_id(bag_path)`**:
  - rosbag パスから MCAP (`.mcap`) または SQLite3 (`.db3`) を自動判別。
- **`load_metadata(bag_path)`**:
  - `metadata.yaml` が存在すれば安全に読み込み、辞書として返却。
- **`open_reader(bag_path, storage_id=None, topics=None)`**:
  - `rosbag2_py.SequentialReader` を初期化・オープン（トピックフィルタリング対応）。
- **`open_writer(bag_path, storage_id='mcap')`**:
  - `rosbag2_py.SequentialWriter` を初期化・オープン。
- **`MessageDeserializer`**:
  - トピック名または型名から ROS2 メッセージクラスを動的解決・キャッシュし、デシリアライズを実行。
- **`message_to_dict(msg)`**:
  - ROS2 メッセージを辞書に再帰変換（NumPy 配列やタプルも JSON シリアライズ可能化）。

## `tools.common.geo` (幾何・地理座標変換)
- **`lat_lon_to_utm(lat, lon, zone=54)`**:
  - WGS84 緯度経度から UTM 座標 (Easting, Northing) [m] へ変換（`pyproj.Proj` インスタンスキャッシュ付き）。
- **`quaternion_to_yaw(qx, qy, qz, qw)`**:
  - クォータニオンから 2D 平面上の Yaw 角 [rad] を算出。
- **`haversine_distance(lat1, lon1, lat2, lon2)`**:
  - 2地点の緯度経度から球面三角法による大圏距離 [m] を算出。

## `tools.common.map` (地図・座標系変換)
- **`load_map_info(map_yaml_path)`**:
  - Nav2 `map.yaml` を読み込み、画像配列 (NumPy)、origin `[x, y, yaw]`、resolution `[m/px]`、画像パスを返却。
- **`estimate_rigid_transform(utm_xy, map_xy)`**:
  - 対応点群から Procrustes 解析 (SVD) により剛体変換 `(x, y, yaw)` [UTM -> map] を推定。
- **`load_transform_from_yaml(yaml_path, label=None)`**:
  - `static_transforms.yaml` から指定 label の `(x, y, yaw)` を取得。
- **`pixel_to_map_coordinates(u, v, resolution, origin_x, origin_y, image_height)`**:
  - 画像ピクセル座標 (u, v) [左上原点] を地図実世界座標 (x, y) [m, 左下原点] に変換。

## `tools.common.cli` (CLI 出力パス解決)
- **`add_output_args(parser, default_filename)`**:
  - `-o/--output`, `--output-to-bag-dir`, `--output-dir` を argparse パーサーに一括追加。
- **`resolve_output_path(bag_path, default_filename, output=None, output_to_bag_dir=False, output_dir=None)`**:
  - 優先度順に出力ファイルの絶対パスを決定し、親ディレクトリを自動作成。

## `tools.common.pose_graph` (slam_gnss_2d 出力の読込)
rclpy に依存しないため、テストや rosbag を読まない解析からも使えます。
- **`load_slam_output(slam_dir)`**: `pose_graph.json` / `gnss_transform.yaml` を読み込み、ノード列 `[t, x, y, yaw]` を返却。
- **`interpolate_nodes(nodes, times)`**: ノード列を時刻で補間 (隣接ノードの間隔が大きい所は無効)。
- **`shift_nodes_to_anchor(nodes, from_transform, to_transform)`**: 2 つの SLAM 結果のアンカー (UTM) の差だけ座標を移す。別走行を、地図を作った走行の座標系で評価するときに使う。
- **`antenna_positions` / `fit_rigid` / `apply_rigid`**: GNSS アンテナ位置の算出と SE(2) 剛体整合。

## `tools.common.loc_metrics` (自己位置推定の評価指標)
numpy のみに依存する指標の計算 (`compose`, `pose_errors`, `correction_jumps`, `nees`, `best_lag`, `recovery_metrics`, `count_episodes` ほか)。

## `tools.common.faults` (故障注入の定義)
評価ツールと故障注入ノードが共有する故障定義 (`kidnap` / `gnss_bias` / `gnss_drop` / `odom_scale` / `scan_drop` / `amcl_drift`) の読込。時刻は再生開始からの経過秒。

## `tools.common.fault_apply` (故障注入の計算)
故障の作用の判定と、スキャンの欠測・GNSS のバイアスなどの計算。numpy のみに依存し、`fault_injector.py` が使う。

## `tools.common.offline_faults` (オフライン評価の故障への変換)
故障定義を、自己位置の監督のオフライン評価 (`localization_supervisor.core.faults`) の定義に直す。情報源の名前は `mg_localization_supervisor/config/supervision.yaml` に合わせる。`eval_localization_offline.py` が使う。
