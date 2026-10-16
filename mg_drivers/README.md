# mg_drivers

センサとモータのドライバ、ホイールオドメトリの補正、点群の後処理、3D 障害物検出。
実機では、`mg_bringup` の `bringup_common` から `bringup.launch.py` が起動される ([mg_bringup](../mg_bringup/README.md))。

## ノード

### C++

| 実行ファイル | 内容 | 主な入出力 |
| :--- | :--- | :--- |
| `wheel_odometry_node` | 車輪のエンコーダ (シリアル) からオドメトリを出す。切断されたら再接続する ([切断からの復旧](#切断からの復旧)) | 出力: `odom` (`nav_msgs/Odometry`)、`~/connected` (`Bool`)、診断 `connection`、イベント `driver.*` |
| `motor_driver_node` | 速度指令をモータドライバ (シリアル) に送る。切断されたら再接続する ([切断からの復旧](#切断からの復旧)) | 入力: `cmd_vel` (`Twist`)、`~/emergency_stop` (`Bool`)、出力: `~/connected` (`Bool`)、診断 `connection`・`emergency_stop`、イベント `driver.*` |
| `depth_postprocess_node` | 点群をボクセルグリッドで間引き、統計的な外れ値を除く。出力を使うノードがないため、既定では起動しない (`use_depth_postprocess`) | 入力: `points`、出力: `points_filtered` (`PointCloud2`) |
| `obstacle_detection_3d_node` | 点群から立体障害物を検出する ([doc](./doc/obstacle_detection_3d.md)) | 入力: `points`、出力: `~/points_obstacle`、`~/cluster_markers` |
| `pcl_downsampling_node` | 点群の間引き。どの launch からも起動されない | 入力: `points/raw`、出力: `points/downsampled` |

### Python (`scripts/`)

| 実行ファイル | 内容 | 主な入出力 |
| :--- | :--- | :--- |
| `wheel_odom_corrector_node.py` | ホイールオドメトリのスケール・バイアスを補正し、速度の共分散を設定する ([doc](./doc/wheel_odom_corrector.md)) | 入力: `odom/raw`、出力: `odom` |
| `driver_watchdog_node.py` | LiDAR・オドメトリ・モータドライバの途絶を監視し、シーケンサに一時停止を出す。止まった上流ドライバ (rplidar) は終了させて、respawn に起動し直させる ([doc](./doc/driver_watchdog.md)) | 入力: `scan_top_lidar`、`odom`、`/motor_driver_node/connected`、出力: 診断 `driver_watchdog_node: <id>`、イベント `watchdog.*`、`/waypoint_sequencer_node/pause_request` |
| `odometry_tf_broadcaster_node.py` | `odom` から TF `odom→base_footprint` を配信する | 入力: `odom` |
| `pose_with_cov_publish_controller_node.py` | `PoseWithCovarianceStamped` の中継を入/切する | 入力: `pose_with_cov_origin`、出力: `pose_with_cov`、サービス: `~/change_publish_state` |
| `generic_publish_controller_node.py` | 任意の型のトピックの中継を入/切する (`msg_module`・`msg_class`・`publish`・`queue_size` で指定)。AMCL の出力のゲート (`amcl_publish_controller_node`) に使う ([mg_navigation](../mg_navigation/README.md)) | 入力: `input_topic`、出力: `output_topic`、サービス: `~/change_publish_state` |
| `depth_to_pointcloud_node.py` | 深度画像 (とカラー画像) から点群を復元する。rosbag の再生確認用 | 出力: `points` |
| `gnss_diagnostics_node.py` | GNSS の測位の品質 (RTK が Fixed か) を診断として出す。ublox の標準の診断は RTK の Fixed と Float を区別しないため、`navpvt` から判定する (判定は `scripts/gnss_quality.py`)。UBX プロトコルで GNSS を使うときだけ起動する | 入力: `/navpvt`、出力: 診断 `gnss_diagnostics_node: quality` (集約で `/Robot/Drivers/GNSS`)、イベント `gnss.fix_changed` |
| `pc_resource_publisher_node.py` | PC の CPU 使用率とメモリ使用率を配信し、しきい値で診断のレベルを決める | 出力: `cpu_usage`、`memory_usage` (`Float32`)、診断 `pc_resource_publisher_node: cpu` と `memory` (集約で `/Robot/System/Resources`) |
| `odom_covariance_override_node.py` | オドメトリの共分散を上書きする。どの launch からも起動されない | 入力: `odom`、出力: `odom/covariance` |
| `odom_offset_republisher.py` | rosbag の再生で、オドメトリの原点をずらして配信し直す (下記) | |

### `odom_offset_republisher.py`

マッピングの走行ログを収録したとき、オドメトリが 0 に戻っていなかった場合に、ずれを引いて配信し直す。

```bash
ros2 bag play --clock <bag> --remap /odom:=/odom_raw
ros2 run mg_drivers odom_offset_republisher.py
```

このノードは `odom_raw` を購読する。補正ノードの入力 `odom/raw` とは別のトピック。

## launch

`bringup.launch.py` が、次の 4 つを束ねる。

| ファイル | 内容 |
| :--- | :--- |
| `bringup_sensors.launch.py` | (実機のみ) `wheel_odometry_node`、RPLiDAR 2 台、GNSS、RealSense |
| `bringup_postprocess.launch.py` | 補正ノード、TF の配信、前方 LiDAR の中継、GNSS の NMEA 変換、点群の後処理、障害物検出 |
| `bringup_common.launch.py` | PC のリソースの配信 (`use_resource_pub`) |
| `bringup_hardware.launch.py` | (実機のみ) `motor_driver_node` (`wheel_pitch`、`max_speed` は launch に書かれている) |

個別に使う launch は次のとおり。

| ファイル | 内容 |
| :--- | :--- |
| `bringup_realsense.launch.py` | RealSense の起動 (`use_rs_d415`、`use_rs_d435i` で機種を選ぶ) |
| `bringup_kissicp.launch.py` | KISS-ICP のオドメトリ。`/cloud_top_lidar` を入力に、`/kissicp/odom` を出す (`pc_topic`、`odom_topic`、`visualize`、`deskew`、`max_range`、`min_range` など) |
| `laser_filters.launch.py` | LiDAR のスキャンの前処理 (`top_laser_filter_yaml`)。`/scan_top_lidar` から、近距離のノイズと自車の点を落とし、`/scan_filtered/scan_top_lidar` に出す。`bringup_postprocess` から起動する ([doc/laser_filters.md](./doc/laser_filters.md)) |
| `depth_to_pointcloud.launch.py` | 深度画像からの点群の復元 (`use_color`、トピック名、`depth_scale`、`stride`) |
| `obstacle_detection_3d.launch.py` | 障害物検出 (`use_sim_time`、`use_sensor_data_qos`、`param_file`) |
| `obstacle_detection_replay.launch.py` | rosbag の再生確認用。点群の復元 + 障害物検出 + RViz2 ([doc](./doc/obstacle_detection_3d.md)) |

### bringup.launch.py の引数

| 引数 | 内容 |
| :--- | :--- |
| `simulation` | `true` なら、センサのドライバを起動しない (`bringup_sensors` を除く) |
| `drive` | `bringup_hardware` を起動するか ([注意](#注意)) |
| `use_odom` / `use_odom_tf` | ホイールオドメトリ / TF の配信 |
| `use_lidar` / `use_gps` | LiDAR / GNSS |
| `use_realsense` | 障害物検出と点群の後処理 (`bringup_postprocess`) |
| `use_depth_postprocess` | 点群の後処理 (`depth_postprocess_node`) も起動する。`use_realsense` が `true` のときだけ効く |
| `use_rs_imu` | RealSense の IMU (ジャイロと加速度) |
| `use_rs_align_depth` | RealSense で、深度をカラーに位置合わせした画像 (`aligned_depth_to_color`) を出す。使うノードがないため既定では出さない |
| `use_odom_corrector` | 補正ノードを起動し、ドライバの出力を `odom/raw` にする |
| `odom_corrector_params_file` | `params/` のファイル名または絶対パス |
| `respawn_drivers` | ドライバのプロセスが終了したら起動し直す ([切断からの復旧](#切断からの復旧)) |
| `use_driver_watchdog` | ドライバの監視ノードを起動する (実機のみ。[doc](./doc/driver_watchdog.md)) |

### bringup_sensors.launch.py の引数

| 引数 | 内容 |
| :--- | :--- |
| `odom_port` | ホイールオドメトリのシリアルポート |
| `top_rplidar_port` | LiDAR のポート |
| `gps_port` | GNSS のポート |
| `use_ubx_protocol` | `true`: u-blox の UBX (`ublox_gps_node`)。`false`: NMEA (`nmea_navsat_driver`) |
| `use_rs_d435i` / `use_rs_d435` | RealSense の機種 |
| `use_rs_imu` / `use_rs_align_depth` | 上と同じ |
| `respawn_drivers` | 上と同じ |

`bringup_hardware.launch.py` の引数は、`device_name` (モータドライバのポート) と `respawn_drivers`。

## 主なトピック

| トピック | 出すノード | 内容 |
| :--- | :--- | :--- |
| `/odom` (補正あり: `/odom/raw` → `/odom`) | `wheel_odometry_node`、補正ノード | ホイールオドメトリ |
| `/scan_top_lidar` | `top_rplidar_node` | 上 LiDAR (RPLiDAR S2) |
| `/gps/fix`、`/navpvt` ほか | `ublox_gps_node` | GNSS |
| `/camera/camera/depth/color/points` → `/camera/depth/points_postprocessed` | RealSense → `depth_postprocess_node` | 点群と、後処理した点群 (`use_depth_postprocess` のとき) |
| `/cmd_vel` | (入力) `motor_driver_node` | 速度指令 |

## パラメータ

`params/` のファイル。

| ファイル | 内容 |
| :--- | :--- |
| `wheel_odom_corrector.yaml` | ホイールオドメトリの補正 ([doc](./doc/wheel_odom_corrector.md)) |
| `driver_watchdog.yaml` | ドライバの監視 ([doc](./doc/driver_watchdog.md)) |
| `obstacle_detection.yaml` | 3D 障害物検出 ([doc](./doc/obstacle_detection_3d.md)) |
| `ekf_global.yaml` | ナビゲーションの EKF (`ekf_global_node`)。位置はホイールオドメトリの速度、AMCL、GNSS。構成は [mg_navigation](../mg_navigation/README.md) |
| `ekf_slam.yaml`、`navsat_transform.yaml` | slam_toolbox の SLAM で、`use_ekf:=true` のときに使う EKF と `navsat_transform` |
| `ublox_ubx_gps.yaml` | u-blox の設定 |
| `rs_d435i.yaml`、`rs_d415.yaml` | RealSense の設定 |
| `top_laser_filter.yaml` | LiDAR の前処理のフィルタ ([doc/laser_filters.md](./doc/laser_filters.md)) |

ホイールオドメトリ (`wheel_odometry_node`) と、モータドライバのパラメータは、launch に直接書かれている (`bringup_sensors.launch.py`、`bringup_hardware.launch.py`)。

## PC の資源の診断

`pc_resource_publisher_node` は、直近 `average_samples` 回の平均が、しきい値以上になると診断のレベルを上げる。Web UI は、この診断のレベルで CPU・メモリの色を決める (しきい値は UI に持たない)。

| パラメータ | 内容 |
| :--- | :--- |
| `cpu_warn_percent` / `cpu_error_percent` | CPU 使用率の `WARN` / `ERROR` |
| `memory_warn_percent` / `memory_error_percent` | メモリ使用率の `WARN` / `ERROR` |
| `average_samples` | 判定に使う平均の回数 (配信の周期 × 回数が、平均する時間) |

## 切断からの復旧

シリアル接続のドライバは自分で再接続し、自分で直せないものはプロセスごと起動し直す (`respawn_drivers`)。仕組み、診断、イベントは [doc/driver_recovery.md](./doc/driver_recovery.md)。

## デバイスと依存

- udev ルール: `config/usb-serial-devices.rules`。センサを `/dev/ttyRobot-*` の名前で見せる (ベンダー ID などの値は空欄)。
- 外部リポジトリ (`mg_drivers.rosinstall`): `nmea_navsat_driver` (MetroGlide の fork)、`rplidar_ros`、`pointcloud_to_laserscan`、`kiss-icp` (fork)。
- `requirements.txt`: `psutil`、`pyserial`、`pyproj`。
- `package.xml`: `rclcpp`、`rclpy`、`laser_filters`、`pcl_ros`、`realsense2_camera`、`robot_localization`、`ublox` など。

## 注意

- `bringup.launch.py` の `bringup_hardware` の条件式は、Python の `and` で書かれていて、`drive` が効かない。実機 (`simulation:=false`) では、`drive` によらず `motor_driver_node` が起動する。
- `obstacle_detection_3d.launch.py` は、入力 `points` を `/rs_d435i/depth/color/points` に remap している。実機の RealSense が点群を出すトピックは `/camera/camera/depth/color/points` (`depth_postprocess_node` の入力と同じ)。シミュレータのブリッジと、`depth_to_pointcloud` の復元点群は、`/rs_d435i/depth/color/points`。
- `scripts/generate_static_transforms.py` は、インストールされない。`tools/scripts/generate_static_transforms.py` と内容が異なる別のファイル。

## テスト

```bash
# Python (補正の計算)
make test pkg=mg_drivers

# C++ (障害物検出のロジックの gtest)
docker exec <develop コンテナ> bash -c \
  "cd /root/ros2_ws && colcon build --packages-select mg_drivers --cmake-args -DBUILD_TESTING=ON && \
   ./build/mg_drivers/test_obstacle_detector"

# 点群の生成 (mg_utils)
make test pkg=mg_utils
```

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/wheel_odom_corrector.md](./doc/wheel_odom_corrector.md) | ホイールオドメトリの補正のモデル、速度が 0 になる不具合、影響、元に戻す方法 |
| [doc/driver_recovery.md](./doc/driver_recovery.md) | シリアルドライバの切断からの復旧、診断、イベント、respawn |
| [doc/laser_filters.md](./doc/laser_filters.md) | LiDAR の前処理の構成と、他の設定との関係 |
| [doc/obstacle_detection_3d.md](./doc/obstacle_detection_3d.md) | 3D 障害物検出のパイプライン、パラメータ、統計ログ、rosbag での確認 |
