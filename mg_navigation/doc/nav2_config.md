# Nav2 の構成と設定

`mg_navigation/launch/navigation_launch.py` が、Nav2 のノードを起動する。設定は `params/nav2_params.yaml` (planner だけ `params/planner_*.yaml`)。
Nav2 は Humble。`collision_monitor` と `costmap_2d` などは、リポジトリ内の修正版を使う ([nav2_pkg](../../nav2_pkg/README.md))。

## 起動するノード

| ノード | パッケージ | 役割 |
| :--- | :--- | :--- |
| `planning_map_server` | `nav2_map_server` | 計画用の地図を `/planning_map` に配信する (測位用の `map_server` とは別) |
| `controller_server` | `nav2_controller` | 経路の追従。`FollowPath` は Regulated Pure Pursuit |
| `smoother_server` | `nav2_smoother` | 経路の平滑化 |
| `planner_server` | `nav2_planner` | グローバルプランナ (プラグイン名 `GridBased`) |
| `behavior_server` | `nav2_behaviors` | リカバリー動作 (`spin`、`backup`、`drive_on_heading`、`wait`) |
| `bt_navigator` | `nav2_bt_navigator` | Behavior Tree による `navigate_to_pose` |
| `velocity_smoother` | `nav2_velocity_smoother` | 速度指令の加減速の制限 |
| `collision_monitor` | `nav2_collision_monitor` | 速度指令に対する衝突の防止 (停止・減速) |
| `collision_detector` | `nav2_collision_monitor` | 衝突の検知 (指令には介入せず、状態だけを出す) |
| `costmap_filter_info_server` | `nav2_map_server` | コストマップフィルタの情報。起動はするが、ライフサイクルの管理対象ではない |

`lifecycle_manager_navigation` が、`costmap_filter_info_server` 以外のノードを管理する。`local_costmap` は `controller_server`、`global_costmap` は `planner_server` が持つ。
`use_composition` の既定は `False` (ノードごとに別プロセス)。

## 速度指令の経路

```
controller_server ─ cmd_vel_nav ─► collision_monitor ─ cmd_vel_collision ─► velocity_smoother ─ cmd_vel ─► motor_driver_node
```

- `behavior_server` (後退・旋回) は、`collision_monitor` を通らず、`cmd_vel` を直接出す。後退・旋回中の衝突の回避は、`behavior_server` 自身の、ローカルコストマップによる衝突の確認 (`simulate_ahead_time`) が担う。Humble の `BehaviorServer` が読むのは `costmap_topic` と `footprint_topic` だけで、グローバルコストマップでの衝突の確認はできない。
- `velocity_smoother` は `OPEN_LOOP` で、`max_accel` / `max_decel` は x が 2.5 m/s²、旋回が 3.2 rad/s²。`max_velocity` は、`FollowPath` の `desired_linear_vel` (1.0 m/s) と合わないため、設定していない。

## コストマップ

どちらも、フットプリントは `[[0.4, 0.3], [0.4, -0.3], [-0.2, -0.3], [-0.2, 0.3]]` (原点は車体の後端寄り) で、`footprint_padding` は 0.1 m、解像度は 0.05 m。

| | local_costmap | global_costmap |
| :--- | :--- | :--- |
| 座標系 | `odom` (ローリングウィンドウ 8 m × 8 m) | `map` |
| 更新 | 5 Hz | 1 Hz |
| 地図 | なし | `/planning_map` (`static_layer`) |
| レイヤー | `top_obstacle_layer`、`obstacle_stvl_layer`、`inflation_layer` | `static_layer`、`top_obstacle_layer`、`obstacle_stvl_layer`、`inflation_layer` |
| 膨張 | `inflation_radius` 0.8 m、`cost_scaling_factor` 3.0 | `inflation_radius` 1.0 m、`cost_scaling_factor` 3.0 |

- `top_obstacle_layer`: 上 LiDAR (`/scan_top_lidar`) の障害物の記録と消去。
- `obstacle_stvl_layer` (`spatio_temporal_voxel_layer`): 3D 障害物検出の出力 (`/obstacle_detection_3d_node/points_obstacle`、[mg_drivers](../../mg_drivers/doc/obstacle_detection_3d.md)) を記録する。`voxel_decay` は local が 5 秒、global が 25 秒。
- 深度カメラの `depth_voxel_layer`、`denoise_layer` は、定義だけが残り、`plugins` からは外れている。
- `speed_filter` (`filters`) は、定義だけがあり、`filters` はコメントアウトされている。
- **`footprint_padding: 0.1` の理由**: 膨張レイヤーの内接半径は、フットプリントの原点から最も近い辺までの距離で決まる。このフットプリントは原点が後端寄りで、そのままでは半幅 (0.3 m) より小さい 0.2 m になる。幅 0.5 m 未満の隙間 (例: 間隔 0.8 m のコーン) が致死扱いにならず、プランナが通れると誤認して経路を引き、RPP が詰まって失敗する。0.1 m の余白で、内接半径を半幅の 0.3 m にして、通れない隙間を塞ぐ。
- Smac から NavFn に戻すときは、`inflation_radius` と `cost_scaling_factor` (従来値は 2.5 と 1.0) と、この余白も確認する (`planner_navfn.yaml` のコメント)。

## グローバルプランナ

`global_planner` 引数 (既定 `smac_lattice`) で、`params/planner_<名前>.yaml` を選ぶ。

| 名前 | ファイル | 特徴 |
| :--- | :--- | :--- |
| `smac_lattice` (既定) | `planner_smac_lattice.yaml` | フットプリント全体で衝突を判定し、差動二輪の動き (前進・その場旋回) の経路を作る。`max_planning_time` は 2 秒。Gazebo での計測では、CPU は NavFn とほぼ同じ (計画が重くなるのは、経路がない・狭い場所のとき) |
| `navfn` | `planner_navfn.yaml` | 点ロボットとして計画する (従来)。計算負荷が小さい |

実機で計画の負荷が高いときは、`global_planner:=navfn` で戻せる。`navfn` を使うときは、`nav2_params.yaml` の `footprint_padding` と `inflation_layer` の値を、上の理由で確認する。

## コントローラ (`controller_server`)

- 周期は 10 Hz。
- `FollowPath` は Regulated Pure Pursuit。`desired_linear_vel` 1.0 m/s、`lookahead_dist` 0.6 m、衝突の検出 (`use_collision_detection`) あり、曲率とコストによる速度の調整あり。
- `general_goal_checker`: `xy_goal_tolerance` 0.25 m、`yaw_goal_tolerance` 0.25 rad、`stateful: True`。ウェイポイントの停止点の到達の判定は、この値による ([mg_waypoint_navigation](../../mg_waypoint_navigation/doc/architecture.md))。
- `progress_checker`: 5 秒 (`movement_time_allowance`) で 0.5 m (`required_movement_radius`) 進まなければ、詰まりとみなして `FollowPath` を失敗させる。衝突の検知のあとの待ち時間としても働く。
- `failure_tolerance` は 5.0 秒。有効な速度が出せない間 (前方に障害物があるなど) は、速度 0 を出して待ち、この時間を超えると `FollowPath` が失敗して、BT のリカバリー (首振り・待機・後退) に入る。**-1 (無期限) にしない**。`progress_checker` の失敗まで無視され、詰まっても失敗せず、リカバリーにも入れず、永久に止まる (Gazebo で確認)。
- `speed_limit_topic` は `/speed_limit`。

## Behavior Tree

`bt_navigator` の既定の BT は Nav2 の `navigate_w_replanning_and_recovery.xml`。ウェイポイントのシーケンサは、ゴールごとに `mg_waypoint_navigation/behavior_trees/` の BT を指定する ([mg_waypoint_navigation](../../mg_waypoint_navigation/README.md#behavior-tree))。
`plugin_lib_names` に `nav2_goal_updated_controller_bt_node` と、このパッケージの BT プラグイン (`mg_is_path_clear_condition_bt_node`、`mg_commit_path_action_bt_node`。[README](../README.md#bt-プラグイン)) が必要。

## 衝突の防止 (`collision_monitor`)

入力は上 LiDAR (`scan_top_lidar`)。次の 3 つを、この順に評価する (`stop` が決まったら、以降を飛ばす)。

| ポリゴン | 動作 | 形 |
| :--- | :--- | :--- |
| `PolygonStop` | 停止 (速度 0) | 前方の箱 (x 0.3〜0.6 m、y ±0.3 m)。`min_points` 4 |
| `PolygonSlowdown` | 減速 (`slowdown_ratio` 0.3) | 前方 1.4 m、後方 0.3 m、両脇 0.8 m。側方から進路に入る対象 (横切る歩行者) に早く反応するため、広く取っている |
| `PolygonApproach` | 接近 (`time_before_collision` 2.0 秒) | `local_costmap/published_footprint` を、現在の速度 (旋回を含む) で前方にシミュレートし、2 秒以内に接触するなら減速する |

- `collision_detector` は、前後の箱 (`PolygonFront`、`PolygonRear`) の状態を `collision_detector_state` に出す (動作は `none`)。観測は上 LiDAR だけ。
- 状態は `collision_monitor_state` に出る (観測用)。
- **実機の注意**: `PolygonApproach` は、`footprint_padding` (0.1 m) を含む。LiDAR が車体の一部を検出する配置だと、常に接触と判定して止まる。実機では、scan に車体の点が入っていないか確認する。
- `PolygonStop` は、`enabled` を動的パラメータ (`ros2 param set`) で切り替えられる。
- **データの途絶での停止 (`<ソース>.stop_on_timeout`)**: 既定は `False` (`nav2_params.yaml` の `scan_top`)。`True` にすると、そのソースのデータが `source_timeout` (3 秒) 以上途絶えた、またはまだ届いていないときに、ポリゴンの `enabled` によらず速度 0 にする。`collision_monitor_state` の `polygon_name` は `source_timeout:<ソース名>`。`ros2 param set /collision_monitor scan_top.stop_on_timeout true` で切り替えられる。無効のままでは、途絶えたソースは黙って無視され、LiDAR が止まっても走り続ける。シーケンサの走行は、`driver_watchdog_node` が先に一時停止する ([mg_drivers](../../mg_drivers/doc/driver_watchdog.md))。これは、手動ゴールなどシーケンサを通らない走行の備え。

## AMCL

`amcl` のパラメータも、`nav2_params.yaml` にある。`scan_topic` は `/scan_top_lidar`、粒子は 500〜2000、運動モデルは `DifferentialMotionModel`、尤度場モデル。
`tf_broadcast: false` (TF は EKF が出す)。`set_initial_pose: true` で、初期姿勢は原点 (GNSS による初期化で上書きされる)。自己位置推定の構成は [localization.md](./localization.md)。

## 変更するとき

- 設定の値は、Nav2 の標準値を踏襲しているものがある。`collision_monitor`、`velocity_smoother`、`behavior_server` の値は、コメントのとおり、Gazebo と実機での確認を経て確定する。
- `scenarios` (シミュレーションでの回帰テスト) で、変更の影響を確認する ([mg_scenario_test](../../mg_scenario_test/README.md))。
