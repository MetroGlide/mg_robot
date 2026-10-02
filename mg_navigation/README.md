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

## 依存

`mg_msgs`、`mg_utils`、`mg_waypoint_navigation`、`navigation2`、`nav2_bringup`、`python3-numpy`、`python3-scipy`。コストマップとコリジョンモニタは、リポジトリ内の修正版 ([nav2_pkg](../nav2_pkg/README.md))。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/localization.md](./doc/localization.md) | 自己位置推定の構成、AMCL の調停・初期化・監視、監督ノードの検知と復旧 |
| [doc/nav2_config.md](./doc/nav2_config.md) | Nav2 のノード、速度指令の経路、コストマップ、プランナ、コントローラ、衝突の防止 |
