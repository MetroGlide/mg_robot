# mg_navigation

Nav2 の起動と設定、自己位置推定の launch と、AMCL の監視 (watchdog)。
AMCL の初期化・入/切の調停・監督ノードは [mg_localization_supervisor](../mg_localization_supervisor/README.md) にあり、`localization_launch.py` から起動する。
`mg_bringup` の `bringup_navigation.launch.py` から `bringup.launch.py` が起動される ([mg_bringup](../mg_bringup/README.md))。

- Nav2 の構成 (ノード、速度指令の経路、コントローラ): [doc/architecture.md](./doc/architecture.md)
- コストマップ、プランナ、衝突の防止 (`collision_monitor`): [doc/costmap_and_collision.md](./doc/costmap_and_collision.md)
- 自己位置推定 (AMCL・GNSS・EKF の融合、監視と復旧): [doc/localization.md](./doc/localization.md)
- RPP (経路追従) のパラメータの意味と調整の考え方: [doc/rpp_tuning.md](./doc/rpp_tuning.md)

## ノード

### Python (`scripts/`、実装は `mg_navigation/`)

| 実行ファイル | 内容 |
| :--- | :--- |
| `amcl_watchdog_node.py` | AMCL の共分散を監視し、大きい状態が続いたら再初期化を要求する (従来の監視) |

### C++

| 実行ファイル | 内容 |
| :--- | :--- |
| `costmap_for_bag_play` | rosbag の再生で、コストマップだけを動かす (`bag_play.launch.py` から起動) |
| `waypoint_navigator_node` | 旧ウェイポイント追従。どの launch からも起動されない ([注意](#注意)) |

### BT プラグイン

`bt_navigator` が読み込む (`params/nav2_params.yaml` の `plugin_lib_names`)。`mg_waypoint_navigation` の BT で使う。使い方と BT 全体の動きは [mg_waypoint_navigation](../mg_waypoint_navigation/README.md#behavior-tree)。

| BT ノード | ライブラリ | 内容 |
| :--- | :--- | :--- |
| `IsPathClear` (Condition) | `mg_is_path_clear_condition_bt_node` | リカバリーの間に、RPP が衝突判定なしで走り出せる状態になったかを判定する。塞がっていた経路が空いたら SUCCESS |
| `IsPathBlocked` (Condition) | `mg_is_path_blocked_condition_bt_node` | 走行中に、経路の近くが塞がっているかを判定する。塞がっていると確定したときだけ SUCCESS。定期的な再計画を止めるゲートに使う |
| `CommitPath` (Action) | `mg_commit_path_action_bt_node` | 計画の結果 (`candidate`) が今のゴールへの経路なら `path` に反映する。計画が失敗しても、今のゴールへの経路があれば保つ。なければ `path` を空にして FAILURE |
| `FindFreeStart` (Action) | `mg_find_free_start_action_bt_node` | ロボットが計画用地図の障害物 (進入禁止の領域) にめり込んでいるとき、その領域の外の最も近い位置を `free_start` に出す。めり込んでいない、出られない、判定できないときは FAILURE |
| `IsLocalizationHealthy` (Condition) | `mg_is_localization_healthy_condition_bt_node` | 自己位置の監督の状態 (`/localization/status`) が `NORMAL` で新しければ SUCCESS。疑わしいときの走行側の対処を止めるために使う |

判定の仕様、ポート、ログは [doc/bt_plugins.md](./doc/bt_plugins.md)。

## launch

| ファイル | 内容 |
| :--- | :--- |
| `bringup.launch.py` | 下の 3 つを束ねる。RViz2 と rosbag の収録も起動できる |
| `localization_launch.py` | `map_server`、`amcl`、`amcl_publish_controller_node`、`amcl_watchdog_node` (遅れて起動)。調停・初期化・監督ノードは `mg_localization_supervisor` の `supervisor.launch.py` を include して起動する |
| `navigation_launch.py` | Nav2 のノード ([doc/architecture.md](./doc/architecture.md)) |
| (他パッケージの launch) | `mg_waypoint_navigation/waypoint_sequencer.launch.py` (ウェイポイントシーケンサ) |
| `bag_play.launch.py` | rosbag の再生中に、コストマップを動かして確認する (`costmap_for_bag_play`。引数: `simulation`、`rviz`、`map`、`planning_map`) |
| `record_bag.launch.py` | rosbag の収録。`mg_utils` の同名の launch と別のファイル |
| `waypoint_editor_node.launch.py` | ウェイポイントの編集。`mg_waypoint_navigation/waypoint_editor.launch.py` と重複 ([注意](#注意)) |

### bringup.launch.py の引数

`mg_bringup` の `bringup_navigation.launch.py` から、同じ名前で伝わる。

| 引数 | 内容 |
| :--- | :--- |
| `simulation` | `use_sim_time` |
| `localization_map` | 測位用の地図 (`mg_bringup` から渡される) |
| `planning_map` | 計画用の地図 (同上) |
| `params_file` | Nav2 (AMCL を含む) のパラメータ (`mg_bringup` では `nav2_params_file`) |
| `global_planner` | プランナの選択: `smac_lattice` / `navfn` |
| `use_navigation` | `false` で、`map_server` と AMCL などの自己位置推定だけを起動する (rosbag の再生評価用) |
| `use_waypoints_follower` | ウェイポイントシーケンサの起動 (`use_navigation` も `true` のとき) |
| `waypoints_load_path` | ウェイポイントのファイル |
| `use_gnss_amcl_initializer` | GNSS から AMCL の初期姿勢を与えるノード |
| `localization_monitor` | 自己位置の監視ノード: `none` / `watchdog` / `supervisor` (既定。検知して自動で復旧する) |
| `localization_supervisor` | `true` で、監視が `watchdog` のまま、監督ノードを並行して通知だけで動かす |
| `supervisor_config` | 監督の設定ファイル (既定は `mg_localization_supervisor` の `config/supervision.yaml`) |
| `localization_profile` | 監督の設定に重ねるプロファイル (空は基の設定。監視が `watchdog` のときは `notify_only`) |
| `rviz` | RViz2 の起動 |
| `record_bag` | rosbag の収録 |
| `namespace`、`use_namespace` | 名前空間 |
| `autostart` | ライフサイクルノードの自動起動 |
| `use_composition` | コンポーネントとして起動 |
| `use_respawn` | 落ちたノードの再起動 |
| `log_level` | ログのレベル |

## パラメータ

`params/` のファイル。

| ファイル | 内容 |
| :--- | :--- |
| `nav2_params.yaml` | Nav2 全体と AMCL |
| `planner_smac_lattice.yaml`、`planner_navfn.yaml` | グローバルプランナ (`global_planner` で選ぶ) |
| `costmap_bag_play.yaml` | `bag_play.launch.py` のコストマップ |
| `record_topic_list.txt` | `record_bag:=true` のときに収録するトピック |

## 使い方

```bash
make navigation                               # 自己位置推定 + Nav2 + ウェイポイントシーケンサ
make navigation OPTS="localization_supervisor:=true global_planner:=navfn"
make rviz2-navigation                         # (別端末) RViz2
```

引数は `OPTS` で渡す。実機とシミュレーションの切り替えは `.env` の `SIMULATION` ([doc/environment.md](../doc/environment.md))。

## 評価

パラメータの変更や復旧の仕組みは、rosbag と SLAM の出力を使って、実機なしに再生で評価できる ([tools の README](../tools/README.md) の `run_localization_variant.sh` と `eval_localization.py`)。

## 注意

- ウェイポイントの機能は `mg_waypoint_navigation` に移っている。このパッケージに残っている `waypoint_navigator_node` (`src/waypoint_navigator.cpp`)、`mg_navigation/waypoint.py`、`launch/waypoint_editor_node.launch.py` は、旧実装の遺物。
- `amcl_watchdog` は、パッケージではなく、`mg_navigation/amcl_watchdog/` のモジュール。以前あった `scripts/amcl_watchdog/README.md` は、[doc/localization.md](./doc/localization.md#amcl_watchdog_node) に統合した。

## テスト

```bash
make test pkg=mg_navigation   # Nav2 のパラメータの整合、watchdog の状態の対応づけ
```

BT プラグインの判定 (gtest) は、develop コンテナで colcon test を使う。

```bash
colcon build --packages-select mg_navigation
colcon test --packages-select mg_navigation --ctest-args -R test_ && colcon test-result --verbose
```

## 依存

`mg_localization_supervisor`、`localization_supervisor`、`localization_supervisor_msgs`、`mg_utils`、`mg_waypoint_navigation`、`navigation2`、`nav2_bringup`。BT プラグインは `behaviortree_cpp_v3`、`nav2_behavior_tree`、`nav2_costmap_2d`、`nav2_util`。コストマップとコリジョンモニタは、リポジトリ内の修正版 ([nav2_pkg](../nav2_pkg/README.md))。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/localization.md](./doc/localization.md) | 自己位置推定の構成、watchdog による監視 |
| [doc/architecture.md](./doc/architecture.md) | Nav2 のノード、速度指令の経路、コントローラ、BT、アンカーの規則 |
| [doc/costmap_and_collision.md](./doc/costmap_and_collision.md) | コストマップ、グローバルプランナ、衝突の防止 (`collision_monitor`) |
| [doc/bt_plugins.md](./doc/bt_plugins.md) | `IsPathClear`・`IsPathBlocked`・`CommitPath`・`FindFreeStart`・`IsLocalizationHealthy` の判定とポート |
| [doc/rpp_tuning.md](./doc/rpp_tuning.md) | RPP のパラメータの意味、値の関係、調整の進め方、症状と対処 |
