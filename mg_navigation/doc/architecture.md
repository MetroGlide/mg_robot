# Nav2 の構成

`mg_navigation/launch/navigation_launch.py` が、Nav2 のノードを起動する。設定は `params/nav2_params.yaml` (planner だけ `params/planner_*.yaml`)。値は yaml が正で、この文書には意味と関係だけを書く ([書き方の規約](../../doc/documentation_guide.md))。
Nav2 は Humble。`collision_monitor` と `costmap_2d` などは、リポジトリ内の修正版を使う ([nav2_pkg](../../nav2_pkg/README.md))。
コストマップ、プランナ、`collision_monitor` は [costmap_and_collision.md](./costmap_and_collision.md)、RPP は [rpp_tuning.md](./rpp_tuning.md)、自己位置推定は [localization.md](./localization.md)。

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
| `costmap_filter_info_server` | `nav2_map_server` | コストマップフィルタの情報。起動はするが、ライフサイクルの管理対象ではない |

`lifecycle_manager_navigation` が、`costmap_filter_info_server` 以外のノードを管理する。`local_costmap` は `controller_server`、`global_costmap` は `planner_server` が持つ。
`use_composition` は launch 引数で、切り替えるとノードごとに別プロセスか、同一プロセスかが変わる。

## 速度指令の経路

```
controller_server ─ cmd_vel_nav ─► collision_monitor ─ cmd_vel_collision ─► velocity_smoother ─ cmd_vel ─► motor_driver_node
```

- `behavior_server` (後退・旋回) は、`collision_monitor` を通らず、`cmd_vel` を直接出す。後退・旋回中の衝突の回避は、`behavior_server` 自身の、ローカルコストマップによる衝突の確認 (`simulate_ahead_time`) が担う。Humble の `BehaviorServer` が読むのは `costmap_topic` と `footprint_topic` だけで、グローバルコストマップでの衝突の確認はできない。
- **速度の上限と加減速は `velocity_smoother` が最後に決める** (`OPEN_LOOP`。`scale_velocities` で並進と旋回の比を保つ)。`max_velocity` を明示しないと `velocity_smoother` の既定値が実際の上限になるので、必ず明示する。
- 守る関係 (テストが検査する):
  - `max_velocity` の並進は、`FollowPath` の `desired_linear_vel` と同じ (アンカー `&v_max`)。
  - 旋回の加速度は、`behavior_server` の `rotational_acc_lim`、RPP の `max_angular_accel`、smoother の `max_accel` と `max_decel` の旋回で同じ (アンカー `&w_acc`)。
  - 加速は穏やかに、減速は大きめにする (`max_decel` の絶対値が大きい)。
- 実機では `motor_driver_node` の `motor_driver.max_speed` ([mg_drivers](../../mg_drivers/README.md)) も上限になる。`max_velocity` 以上にする。
- `OPEN_LOOP` なので、実機のモータが `max_decel` どおりに減速できるかは、別に確認する。`collision_monitor` の停止の余裕はこの値から決めている。

## コントローラ (`controller_server`)

- `FollowPath` は Regulated Pure Pursuit。先読み距離は速度に連動し、衝突の検出 (`use_collision_detection`) と、曲率・コスト・ゴール接近による速度の調整がある。各パラメータの意味と調整の考え方、守る値の関係は [rpp_tuning.md](./rpp_tuning.md)。
- RPP の衝突の確認の時間 (`max_allowed_time_to_collision_up_to_carrot`) は、`collision_monitor` の `PolygonApproach` の `time_before_collision` より長くする。障害物を主に避けるのは RPP とコストマップで、`collision_monitor` は通常の走行では発動しない後ろ盾にする。
- `general_goal_checker`: `xy_goal_tolerance` と `yaw_goal_tolerance` が、到達の判定の許容。`stateful: True`。ウェイポイントの停止点の到達の判定は、この値による ([mg_waypoint_navigation](../../mg_waypoint_navigation/doc/architecture.md))。
- `progress_checker`: `movement_time_allowance` の間に `required_movement_radius` 進まなければ、詰まりとみなして `FollowPath` を失敗させる。数え始めは止まった時点で、障害物が去っても、再開して `required_movement_radius` 進むまでは止まらない。このため `movement_time_allowance` は、`failure_tolerance` に、再開して進みきる時間 (`collision_monitor` の減速を含む) を足した値より長くする。同じ値だと、待っている途中に障害物が去っても進みきる前に失敗し、不要なリカバリーに入る。障害物の前で待つ時間は `failure_tolerance` で決まる。
- `failure_tolerance`: 有効な速度が出せない間 (前方に障害物があるなど) は、速度 0 を出して待ち、この時間を超えると `FollowPath` が失敗して、BT のリカバリー (引き直し・後退・首振り・待機) に入る。**-1 (無期限) にしない**。`progress_checker` の失敗まで無視され、詰まっても失敗せず、リカバリーにも入れず、永久に止まる (Gazebo で確認)。
- `speed_limit_topic` は `/speed_limit`。

## Behavior Tree

`bt_navigator` の既定の BT は Nav2 の `navigate_w_replanning_and_recovery.xml`。ウェイポイントのシーケンサは、ゴールごとに `mg_waypoint_navigation/behavior_trees/` の BT を指定する ([mg_waypoint_navigation](../../mg_waypoint_navigation/README.md#behavior-tree))。
`plugin_lib_names` に `nav2_goal_updated_controller_bt_node` と、このパッケージの BT プラグイン (`mg_is_path_clear_condition_bt_node`、`mg_commit_path_action_bt_node`。[README](../README.md#bt-プラグイン)) が必要。

## AMCL

`amcl` のパラメータも、`nav2_params.yaml` にある。入力は生の `/scan_top_lidar` (前処理済みの scan ではない)、運動モデルは差動二輪、観測は尤度場モデル。
`tf_broadcast: false` (TF は EKF が出す)。初期姿勢は GNSS による初期化で上書きされる。自己位置推定の構成は [localization.md](./localization.md)。

## 変更するとき

- 設定の値は、Nav2 の標準値を踏襲しているものがある。`collision_monitor`、`velocity_smoother`、`behavior_server` の値は、コメントのとおり、Gazebo と実機での確認を経て確定する。
- `scenarios` (シミュレーションでの回帰テスト) で、変更の影響を確認する ([mg_scenario_test](../../mg_scenario_test/README.md))。
- **`nav2_params.yaml` の YAML のアンカー (`&名前` / `*名前`) は、スカラー (数値・文字列) だけに使う。** 起動時に `RewrittenYaml` が YAML を読み直して書き出すとき、リストや辞書のアンカーは `&id001` の形で残り、rcl のパラメータのパーサが読めず、全 Nav2 サーバーが起動時に落ちる。アンカーを使っているのは、巡航速度 (`&v_max`)、旋回の加速度 (`&w_acc`)、`&robot_footprint`、`&footprint_padding`、RPP とコストマップの膨張で共有する `&inflation_cost_scaling_factor`。
- 符号の反転や、別のファイルの値との関係 (smoother の上限と巡航速度、旋回の加速度、LiDAR の前処理の箱と footprint など) は、アンカーで表せないので `test/test_nav2_params_consistency.py` が検査する。値を変えたら `make test pkg=mg_navigation` を実行する。
