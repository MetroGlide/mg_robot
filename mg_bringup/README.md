# mg_bringup

SLAM とナビゲーションを起動する、トップレベルの launch 群。ドライバ・URDF・自己位置推定・Nav2・SLAM の launch を束ねるだけで、ノードやパラメータは持たない。
compose のサービス (`make slam`、`make navigation` など) は、ここの launch を起動する ([doc/commands.md](../doc/commands.md))。

## launch

| ファイル | 起動するもの | compose サービス |
| :--- | :--- | :--- |
| `launch/common/bringup_common.launch.py` | `mg_drivers/bringup` と `mg_description/bringup` (センサ・前処理・URDF) | - (下の launch が include) |
| `launch/navigation/bringup_navigation.launch.py` | 上の common、EKF (`ekf_global_node`)、`slam_gnss_nav_bridge`、`mg_navigation/bringup` (自己位置推定・Nav2・ウェイポイントシーケンサ) | `navigation` |
| `launch/slam/bringup_slam_toolbox.launch.py` | common、任意で EKF と `navsat_transform`、`mg_slam/bringup_slam_toolbox` | `slam` |
| `launch/slam/bringup_slam_gnss_2d.launch.py` | common と `slam_gnss_2d/bringup_slam_gnss_2d` | なし |
| `launch/slam/offline_slam_gnss_2d.launch.py` | `mg_description`、`mg_drivers/bringup_postprocess`、`slam_offline_node` (rosbag の再処理) | `offline-slam-gnss-2d` |

`slam-gnss-2d` のサービスは、この package ではなく `slam_gnss_2d` の launch を直接起動する (SLAM ノードのみ)。

階層は [doc/system_architecture.md](../doc/system_architecture.md#launch-の階層) を参照。

## 起動引数

引数の既定値に環境変数 (`SIMULATION`、`USE_RVIZ`、`MAP_PATH`、`WAYPOINT_PATH`) を使うものがある ([doc/environment.md](../doc/environment.md))。
`OPTS` で渡す: `make navigation OPTS="use_ekf:=false global_planner:=navfn"`。

### bringup_common

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `simulation` | `$SIMULATION` | `true` でセンサのドライバを起動しない |
| `drive` | `false` | モータドライバを起動するか ([注意](#注意)) |
| `use_odom` / `use_odom_tf` | `true` | ホイールオドメトリ / `odom→base_footprint` の TF |
| `use_lidar` / `use_gps` / `use_realsense` | `true` | 各センサ |
| `use_odom_corrector` | `false` | ホイールオドメトリの補正ノード |
| `odom_corrector_params_file` | `wheel_odom_corrector.yaml` | `mg_drivers/params` のファイル名または絶対パス |

### bringup_navigation

`bringup_common` の引数 (`use_odom_corrector` の既定値は `true`) に加えて次の引数がある。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `map_path` | `$MAP_PATH/Region1_jikoichi.yaml` | 測位用の地図 |
| `planning_map_path` | `$MAP_PATH/Region1_kinshi.yaml` | 計画用の地図 |
| `waypoints_load_path` | `$WAYPOINT_PATH` | ウェイポイントのファイル |
| `rviz` | `$USE_RVIZ` | RViz2 を起動するか |
| `record_bag` | `false` | rosbag を収録するか |
| `global_planner` | `smac_lattice` | グローバルプランナ (`smac_lattice` / `navfn`) |
| `use_ekf` | `True` | EKF を起動するか |
| `ekf_params_file` | `ekf_global.yaml` | `mg_drivers/params` のファイル名または絶対パス |
| `ekf_odom_topic` | `ekf_global_odom` | EKF の出力トピック |
| `use_navigation` | `true` | `false` で自己位置推定だけを起動する (rosbag の再生評価用) |
| `use_gnss_amcl_initializer` | `true` | GNSS から AMCL の初期姿勢を与えるノード |
| `localization_monitor` | `watchdog` | 自己位置の監視ノード (`none` / `watchdog` / `supervisor`) |
| `supervisor_params_file` | `mg_navigation/params/localization_supervisor.yaml` | 監督ノードのパラメータ |
| `nav2_params_file` | `mg_navigation/params/nav2_params.yaml` | Nav2 のパラメータ |
| `use_slam_gnss_bridge` | `true` | GNSS ブリッジ (`/odom/gps`) を起動するか |
| `bridge_params_file` | `slam_gnss_2d/params/nav_bridge.yaml` | GNSS ブリッジのパラメータ |
| `gnss_transform_file` | `$MAP_PATH/gnss_transform.yaml` | GNSS を地図の座標に変換するファイル |

`bringup_common` に渡す `drive` は `false` で固定。

### bringup_slam_toolbox

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `simulation`、`drive`、`use_odom`、`use_lidar`、`use_gps` | `bringup_common` と同じ | |
| `rviz` | `$USE_RVIZ` | RViz2 を起動するか |
| `record_bag` | `false` | rosbag を収録するか |
| `use_ekf` | `False` | EKF と `navsat_transform` を起動するか。`true` のとき、ドライバのオドメトリの TF は出さない |
| `ekf_params_file` | `ekf_slam.yaml` | `mg_drivers/params` のファイル名 |
| `navsat_params_file` | `navsat_transform.yaml` | 同上 |
| `ekf_odom_topic` | `odometry/filtered` | EKF の出力トピック |

### bringup_slam_gnss_2d

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `simulation`、`drive`、`use_odom`、`use_lidar`、`use_gps` | `bringup_common` と同じ | |
| `rviz` | `$USE_RVIZ` | RViz2 を起動するか |
| `rviz_param` | `slam_gnss_2d.rviz` | RViz2 の設定ファイル名 |

### offline_slam_gnss_2d

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `bag_path` | `$ROSBAG_FILE` | 処理する rosbag |
| `start_time` / `end_time` | `0.0` | 処理する区間 [秒]。bag の開始からの経過時間。`0.0` は先頭 / 末尾 |
| `skip_intermediate_rendering` | `true` | 途中の地図描画を止めて、終了時に描く |
| `rviz` | `$USE_RVIZ` (未設定なら `false`) | RViz2 を起動するか |
| `rviz_param` | `slam_gnss_2d.rviz` | RViz2 の設定ファイル名 |
| `params_file` | `slam_gnss_2d/params/slam_gnss_2d.yaml` | SLAM のパラメータ |

## 注意

- `drive` を `true` にしてもしなくても、実機 (`simulation:=false`) ではモータドライバが起動する。`mg_drivers/launch/bringup.launch.py` の条件式が、Python の `and` で書かれているため。
- `navigation` サービスの `map_path` と `planning_map_path` の既定値は、特定のファイル名 (`Region1_*.yaml`) に固定されている。別の地図を使うときは引数で渡す ([doc/environment.md](../doc/environment.md#地図の指定))。

## 依存

`package.xml` の依存は `mg_utils` だけ。launch が実際に include する次のパッケージは、`package.xml` に書かれていない: `mg_drivers`、`mg_description`、`mg_navigation`、`mg_waypoint_navigation`、`mg_slam`、`slam_gnss_2d`、`robot_localization`。
