# システム構成

MG-01 のノードとデータの流れ、座標系 (TF)、センサの取り付け位置をまとめる。
各ノードの詳細は、リンク先のパッケージの README を参照。

## 全体のデータフロー

```
 センサ                     ドライバ/前処理                自己位置推定                   ナビゲーション                 操作・監視
┌────────────────┐       ┌───────────────────┐
│ モータドライバ  │◄─────│ motor_driver_node  │◄─ /cmd_vel ◄───────────────────────────────┐
│ (ホイール)      │─────►│ wheel_odometry_node│─ /odom/raw ─► wheel_odom_corrector ─ /odom ─┼─► EKF (ekf_global_node)
├────────────────┤       ├───────────────────┤                                            │      ▲ ▲ ▲
│ RPLiDAR S2 (上) │─────►│ rplidar_node       │─ /scan_top_lidar ───────────► AMCL ─ /amcl_pose ┘ │ │
├────────────────┤       ├───────────────────┤                                                   │ │
│ u-blox GNSS     │─────►│ ublox_gps_node     │─ /navpvt ─► slam_gnss_nav_bridge ─ /odom/gps ─────┘ │
├────────────────┤       ├───────────────────┤                                                     │
│ RealSense D435i │─────►│ realsense2_camera  │─ 点群 ─► depth_postprocess ─► obstacle_detection_3d  │
└────────────────┘       └───────────────────┘                                                     │
                                                                                                    ▼
 EKF ─► map→odom TF ─► Nav2 (planner / controller / bt_navigator) ─ /cmd_vel_nav ─► collision_monitor
                                      ▲                                              ─ /cmd_vel_collision ─► velocity_smoother ─ /cmd_vel
                                      │ NavigateToPose / FollowWaypoints 等
                       waypoint_sequencer (mg_waypoint_navigation) ◄── Web UI (foxglove_bridge, system_manager)
```

- 自己位置推定の詳細 (AMCL・GNSS・EKF の融合、AMCL の入/切の調停、監督ノード) は [mg_navigation の README](../mg_navigation/README.md) を参照。
- `/cmd_vel` は、Nav2 の controller が出した `cmd_vel_nav` を `collision_monitor`、`velocity_smoother` の順に通して作る (`mg_navigation/launch/navigation_launch.py`)。
- 地図作成 (SLAM) は走行とは別のスタックで行う。センサは同じドライバを使い、`slam_gnss_2d` が地図と `gnss_transform.yaml` を作る ([slam_gnss_2d](../slam_gnss_2d/README.md))。

## launch の階層

`make` で起動するサービスと、起動する launch の対応は [commands.md](./commands.md) を参照。

```
mg_bringup/navigation/bringup_navigation.launch.py   (make navigation)
├─ mg_bringup/common/bringup_common.launch.py
│   ├─ mg_drivers/bringup.launch.py                  センサ・前処理・補正ノード (実機のみ sensors)
│   │   ├─ bringup_sensors      (実機のみ) wheel_odometry / rplidar / GNSS / RealSense
│   │   ├─ bringup_postprocess  odom 補正・TF・scan の公開制御・点群の後処理・障害物検出
│   │   ├─ bringup_common       PC のリソース配信
│   │   └─ bringup_hardware     motor_driver_node (実機のみ)
│   └─ mg_description/bringup.launch.py              robot_state_publisher
├─ ekf_global_node                                   robot_localization
├─ slam_gnss_nav_bridge                              slam_gnss_2d (GNSS → /odom/gps)
└─ mg_navigation/bringup.launch.py
    ├─ localization_launch.py                        map_server / AMCL / 初期化 / 監視 / 調停
    ├─ navigation_launch.py                          Nav2 本体
    └─ mg_waypoint_navigation/waypoint_sequencer.launch.py
```

SLAM 側の launch は次のとおり。

| サービス | launch | 備考 |
| :--- | :--- | :--- |
| `slam` | `mg_bringup/slam/bringup_slam_toolbox.launch.py` | slam_toolbox ([mg_slam](../mg_slam/README.md)) |
| `slam-gnss-2d` | `slam_gnss_2d/bringup_slam_gnss_2d.launch.py` | GNSS 拘束付き 2D SLAM (現役)。SLAM ノードと RViz2 だけで、センサのドライバは含まない |
| (サービスなし) | `mg_bringup/slam/bringup_slam_gnss_2d.launch.py` | ドライバ (`bringup_common`) と上の SLAM を束ねる |
| `offline-slam-gnss-2d` | `mg_bringup/slam/offline_slam_gnss_2d.launch.py` | rosbag からの再処理 |
| `reoptimize-slam` | `slam_gnss_2d/reoptimize.launch.py` | ポーズグラフの再最適化 |

## 座標系 (TF)

URDF (`mg_description/urdf/`) から抽出した固定リンクの構成。`map→odom` は EKF、`odom→base_footprint` は `odometry_tf_broadcaster_node` が配信する。

```
map ─(EKF)─► odom ─(odometry_tf_broadcaster)─► base_footprint
base_footprint ─► base_link (z=0.15)
base_link ─► left_wheel_link  (y=+0.25, 半径 0.15)
          ─► right_wheel_link (y=-0.25, 半径 0.15)
          ─► caster_link      (x=-0.45, z=-0.1)
          ─► gps_link         (0.26, -0.13, 0.33)
          ─► top_frame_link   (0.23, 0, 0.45)
                 ─► top_lrf_link (上 LiDAR。0, 0, 0.03。yaw=π)
                 ─► livox_lidar_link (0, 0, 0.06)
                 ─► camera_link_base (D435i。0.02, 0, -0.23。pitch=0.27)
```

- 車輪間隔は 0.5 m。
- Livox は URDF のリンクがあるだけで、ドライバは起動していない。

## センサとデバイス

| センサ | ドライバ | 主なトピック | デバイス (udev 名) |
| :--- | :--- | :--- | :--- |
| ホイール (モータドライバ) | 自作 `motor_driver_node` / `wheel_odometry_node` | `/cmd_vel` (入力)、`/odom/raw` または `/odom` | `/dev/ttyRobot-motordriver` / `/dev/ttyRobot-odom` |
| 上 LiDAR RPLiDAR S2 | `rplidar_ros` | `/scan_top_lidar` | `/dev/ttyRobot-toplidar` |
| GNSS u-blox | `ublox_gps_node` (UBX) または `nmea_navsat_driver` (NMEA) | `/gps/fix`、`/navpvt` | `/dev/ttyRobot-gps` |
| RealSense D435i | `realsense2_camera` | `/camera/camera/depth/color/points` ほか | USB |

- udev ルールは `mg_drivers/config/usb-serial-devices.rules`。
- 起動引数 (ポート、使うセンサの切り替え) は [mg_drivers の README](../mg_drivers/README.md) を参照。

## 外部に出る口 (ポート)

| ポート | サービス | 用途 |
| :--- | :--- | :--- |
| 8765 | `foxglove-bridge` | Web UI と ROS の通信 (WebSocket) |
| 8080 | `web-ui` | Web UI の静的配信 |
| 5173 | `web-ui-dev` | Vite 開発サーバ (HMR) |
| 8001 | `system-manager` | コンテナ・地図・rosbag・シナリオテストの管理 API |

## 関連ドキュメント

- [commands.md](./commands.md): make ターゲットと compose サービスの対応
- [docker.md](./docker.md): Docker イメージの構成
- [operation.md](./operation.md): 地図作成から走行・解析までの運用手順
