# mg_navigation

Nav2 の起動と設定、自己位置推定の補助ノード (AMCL の初期化・監視・入/切の調停)。
`mg_bringup` の `bringup_navigation.launch.py` から `bringup.launch.py` が起動される ([mg_bringup](../mg_bringup/README.md))。

- Nav2 の構成と設定: [doc/nav2_config.md](./doc/nav2_config.md)
- 自己位置推定 (AMCL・GNSS・EKF の融合、監視と復旧): [doc/localization.md](./doc/localization.md)

## ノード

### Python (`scripts/`、実装は `mg_navigation/`)

| 実行ファイル | 内容 |
| :--- | :--- |
| `gnss_amcl_initializer_node.py` | GNSS の位置から、AMCL (`/initialpose`) と EKF (`/set_pose`) の初期姿勢を出す。サービス `~/request_reinit` でやり直せる |
| `amcl_watchdog_node.py` | AMCL の共分散を監視し、大きい状態が続いたら再初期化を要求する (従来の監視) |
| `localization_supervisor_node.py` | AMCL のずれを検知して通知する。有効にすると、EKF から切り離して復旧する |
| `amcl_gate_arbiter_node.py` | AMCL の EKF への入/切を、要求元ごとに調停する |

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
| `CommitPath` (Action) | `mg_commit_path_action_bt_node` | 計画の結果 (`candidate`) が今のゴールへの経路なら `path` に反映する。計画が失敗しても、今のゴールへの経路があれば保つ。なければ `path` を空にして FAILURE |

#### IsPathClear

RPP (Humble 1.1.20 の `RegulatedPurePursuitController`) の衝突判定と同等以上に厳しく判定する。次のどこかで、フットプリントのコストが致死 (`LETHAL_OBSTACLE`) 以上ならブロックとする (Nav2 の `FootprintCollisionChecker` を使う。未知のセルの扱いも RPP と同じ)。

1. 現在の姿勢
2. 最小・最大の先読み距離のキャロットへ向かう動き。向きのずれが `rotate_to_heading_min_angle` を超えればその場の回転の掃引、そうでなければピュアパーシュートの円弧
3. 経路の、ロボットの最近傍点から `max_lookahead_dist + margin` までの範囲

- 設定値は持たない。RPP のパラメータ (`<controller_id>.lookahead_dist` など) を `controller_server` から、フットプリント・`footprint_padding`・`track_unknown_space`・`robot_base_frame` を `local_costmap` から取得する。tick の間隔が 1 秒以上空いたら新しいリカバリーとみなし、取得し直す (パラメータの変更は次のリカバリーから効く)。取得できない間は FAILURE (再開しない)
- コストマップは `local_costmap/costmap_raw` (2 Hz) を購読する。判定は、新しいコストマップか経路が届いたとき、または 0.1 秒ごと (姿勢の変化を反映する) に行い、tick では結果を返すだけ
- SUCCESS は、そのリカバリーの間に一度ブロックを観測し、その後、空いた状態が `clear_duration` (1 秒) 以上、かつコストマップ 2 枚以上続いたとき。最初から空いている場合 (スリップなど障害物以外が原因の失敗) は SUCCESS にしない。ブロックはリカバリーに入る直前のコストマップでも記録するが、空いたことはリカバリーが始まってから届いたコストマップでだけ記録する (コストマップのクリア直後の空の地図で誤判定しない)。コストマップが 2 秒以上届かなければ、空いたとはみなさない
- `local_costmap` の `publish_frequency` を下げると、「2 枚以上・1 秒以上」に時間がかかり、再開が遅れる
- RPP の `use_collision_detection` が false なら、常に FAILURE
- ログ (bt_navigator): リカバリーごとに最初の判定の結果 (`first check of this recovery: blocked=...`。コストマップの古さと経路の終点も出す)、塞がっていることを観測したとき (`path is blocked`)、空いて SUCCESS を返したとき (`path became clear, resuming`) に INFO を出す。経路がない、ロボットの姿勢や経路を変換できない、コストマップが古い、で判定できないときは WARN (5 秒に 1 回)。リカバリーで早く再開しなかった理由は、最初の判定が `blocked=0` (障害物以外が原因と判断) か、判定できなかったかで見分ける
- Nav2 を更新するときは、RPP の判定 (`isCollisionImminent` など) の変更に合わせて見直す

| ポート | 既定値 | 内容 |
| :--- | :--- | :--- |
| `path` | なし | 判定する経路 |
| `margin` | `0.3` | 経路に沿って `max_lookahead_dist` の先まで見る距離 [m] |
| `clear_duration` | `1.0` | 空いた状態が続く必要のある時間 [s] |
| `controller_node` / `controller_id` | `controller_server` / `FollowPath` | RPP のパラメータの取得先 |
| `costmap_node` / `costmap_topic` | `local_costmap/local_costmap` / `local_costmap/costmap_raw` | コストマップのパラメータの取得先と購読するトピック |
| `transform_tolerance` | `0.2` | TF の許容時間 [s] |

#### CommitPath

| ポート | 既定値 | 内容 |
| :--- | :--- | :--- |
| `candidate` | なし | 計画の結果 (`ComputePathToPose` の出力。失敗すると空になる) |
| `goal` | なし | 今のゴール |
| `path` | なし | 追従する経路 (入出力) |
| `goal_tolerance` | `0.6` | 経路の終点とゴールの距離がこれ以内なら、そのゴールへの経路とみなす [m] |

## launch

| ファイル | 内容 |
| :--- | :--- |
| `bringup.launch.py` | 下の 3 つを束ねる。RViz2 と rosbag の収録も起動できる |
| `localization_launch.py` | `map_server`、`amcl`、`amcl_publish_controller_node`、`amcl_gate_arbiter`、初期化・監視ノード (遅れて起動) |
| `navigation_launch.py` | Nav2 のノード ([doc/nav2_config.md](./doc/nav2_config.md)) |
| (他パッケージの launch) | `mg_waypoint_navigation/waypoint_sequencer.launch.py` (ウェイポイントシーケンサ) |
| `bag_play.launch.py` | rosbag の再生中に、コストマップを動かして確認する (`costmap_for_bag_play`。引数: `simulation`、`rviz`、`map`、`planning_map`) |
| `record_bag.launch.py` | rosbag の収録。`mg_utils` の同名の launch と別のファイル |
| `waypoint_editor_node.launch.py` | ウェイポイントの編集。`mg_waypoint_navigation/waypoint_editor.launch.py` と重複 ([注意](#注意)) |

### bringup.launch.py の引数

`mg_bringup` の `bringup_navigation.launch.py` から、同じ名前で伝わる。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `simulation` | `$SIMULATION` | `use_sim_time` |
| `localization_map` | `map/map.yaml` | 測位用の地図 (`mg_bringup` から渡される) |
| `planning_map` | `map/map.yaml` | 計画用の地図 (同上) |
| `params_file` | `params/nav2_params.yaml` | Nav2 (AMCL を含む) のパラメータ (`mg_bringup` では `nav2_params_file`) |
| `global_planner` | `smac_lattice` | `smac_lattice` / `navfn` |
| `use_navigation` | `true` | `false` で、`map_server` と AMCL などの自己位置推定だけを起動する (rosbag の再生評価用) |
| `use_waypoints_follower` | `true` | ウェイポイントシーケンサの起動 (`use_navigation` も `true` のとき) |
| `waypoints_load_path` | `$WAYPOINT_PATH` | ウェイポイントのファイル |
| `use_gnss_amcl_initializer` | `true` | GNSS から AMCL の初期姿勢を与えるノード |
| `localization_monitor` | `watchdog` | 自己位置の監視ノード: `none` / `watchdog` / `supervisor` |
| `supervisor_params_file` | `params/localization_supervisor.yaml` | 監督ノードのパラメータ |
| `rviz` | `$USE_RVIZ` | RViz2 の起動 |
| `record_bag` | `false` | rosbag の収録 |
| `namespace`、`use_namespace` | 空、`false` | 名前空間 |
| `autostart` | `true` | ライフサイクルノードの自動起動 |
| `use_composition` | `False` | コンポーネントとして起動 |
| `use_respawn` | `false` | 落ちたノードの再起動 |
| `log_level` | `info` | ログのレベル |

## パラメータ

`params/` のファイル。

| ファイル | 内容 |
| :--- | :--- |
| `nav2_params.yaml` | Nav2 全体と AMCL |
| `planner_smac_lattice.yaml`、`planner_navfn.yaml` | グローバルプランナ (`global_planner` で選ぶ) |
| `localization_supervisor.yaml` | 監督ノード |
| `costmap_bag_play.yaml` | `bag_play.launch.py` のコストマップ |
| `record_topic_list.txt` | `record_bag:=true` のときに収録するトピック |

## 使い方

```bash
make navigation                               # 自己位置推定 + Nav2 + ウェイポイントシーケンサ
make navigation OPTS="localization_monitor:=supervisor global_planner:=navfn"
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
make test pkg=mg_navigation   # 監督ノードの判定・状態遷移、調停ノード
```

BT プラグインの判定 (gtest) は、develop コンテナで colcon test を使う。

```bash
colcon build --packages-select mg_navigation
colcon test --packages-select mg_navigation --ctest-args -R test_ && colcon test-result --verbose
```

## 依存

`mg_msgs`、`mg_utils`、`mg_waypoint_navigation`、`navigation2`、`nav2_bringup`、`python3-numpy`、`python3-scipy`。BT プラグインは `behaviortree_cpp_v3`、`nav2_behavior_tree`、`nav2_costmap_2d`、`nav2_util`。コストマップとコリジョンモニタは、リポジトリ内の修正版 ([nav2_pkg](../nav2_pkg/README.md))。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/localization.md](./doc/localization.md) | 自己位置推定の構成、AMCL の調停・初期化・監視、監督ノードの検知と復旧 |
| [doc/nav2_config.md](./doc/nav2_config.md) | Nav2 のノード、速度指令の経路、コストマップ、プランナ、コントローラ、衝突の防止 |
