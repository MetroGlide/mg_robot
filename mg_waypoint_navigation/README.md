# mg_waypoint_navigation

ウェイポイントの一覧に沿って、Nav2 でロボットを走らせるシーケンサ (`waypoint_sequencer_node`) と、ウェイポイントの編集ノード (`waypoint_editor_node`)。
状態遷移 (FSM) で、各ウェイポイントへの走行と、到達時のアクション (待機、地図の切り替え、AMCL の入/切など) を進める。

## ノード

| 実行ファイル | 内容 |
| :--- | :--- |
| `waypoint_sequencer_node.py` | シーケンサ。ウェイポイントを読み込み、Nav2 の `navigate_to_pose` に順にゴールを送る |
| `waypoint_editor_node.py` | RViz2 の対話マーカーで、ウェイポイントを置いて YAML に保存する |
| `migrate_waypoints.py` | v1 形式のウェイポイントを v2 形式に変換する |

## launch

| ファイル | 起動するもの | compose サービス |
| :--- | :--- | :--- |
| `launch/waypoint_sequencer.launch.py` | `waypoint_sequencer_node` | なし (`mg_navigation/bringup.launch.py` が include する) |
| `launch/waypoint_editor.launch.py` | `map_server` (地図ごと)、`waypoint_editor_node`、RViz2 | `waypoint-editor` |

### waypoint_sequencer.launch.py

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `simulation` | `false` | `use_sim_time` |
| `load_path` | `$WAYPOINT_PATH` | ウェイポイントの YAML |
| `initial_localization_map` | 空 | 起動時に `map_server` が読んでいる測位用の地図 (`~/loaded_maps` の初期値) |
| `initial_planning_map` | 空 | 起動時に `planning_map_server` が読んでいる計画用の地図 |
| `publish_waypoint_status` | `true` | `~/status` を配信するか |

### waypoint_editor.launch.py

環境変数 `MAP_PATH` が必須で、そのディレクトリの `map_list.txt` (1 行に 1 つの地図の YAML。`#` で始まる行はコメント) に書かれた地図を、すべて読み込んで表示する。

| 引数 | 既定値 | 内容 |
| :--- | :--- | :--- |
| `simulation` | `false` | `use_sim_time` |
| `load_path` | `/root/ros2_data/map/waypoint.yaml` | 編集を始めるときに読み込む YAML |
| `save_path` | `$WAYPOINT_PATH` | 保存先。サービス `save_waypoints` は、ファイルが既にあると失敗する。`force_save_waypoints` は上書きする |

RViz2 の `/goal_pose` (2D Goal Pose) でウェイポイントを追加し、マーカーを動かして編集する。
GUI の [waypoint-tool](https://github.com/Chu-son/waypoint-tool) でも作れる ([doc/waypoint_format.md](./doc/waypoint_format.md#waypoint-tool-での作成))。

## ROS インターフェース

`waypoint_sequencer_node` のもの。型・条件・パラメータの詳細は [doc/architecture.md](./doc/architecture.md#ros-インターフェース)。

| 種別 | 名前 | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| Service | `~/start` | `mg_msgs/StartSequence` | 走行を開始する |
| Service | `~/stop` | `std_srvs/Trigger` | 止める (一時停止の要求も解除する) |
| Service | `~/reload_waypoints` | `std_srvs/Trigger` | ウェイポイントを読み込み直す |
| Service | `~/navigate_to_pose` | `mg_msgs/SendGoal` | BT を指定して、手動でゴールを 1 つ送る |
| Service | `~/load_map` | `mg_msgs/LoadMaps` | 測位用・計画用の地図を読み込ませる |
| Subscriber | `~/set_next_waypoint_index` | `std_msgs/Int16` | 次に向かうウェイポイントを指定する |
| Subscriber | `~/pause_request` | `mg_msgs/PauseRequest` | 一時停止の要求 |
| Publisher | `~/status` | `mg_msgs/SequencerStatus` | 状態 (10 Hz) |
| Publisher | `~/waypoints` | `mg_msgs/WaypointList` | 読み込んだウェイポイント |
| Publisher | `~/waypoints_markers` | `visualization_msgs/MarkerArray` | RViz2 用のマーカー |
| Publisher | `~/loaded_maps` | `mg_msgs/LoadedMaps` | 読み込み済みの地図 |
| Publisher | `~/navigation_mode` | `std_msgs/String` | 次のゴールで使うモードと BT |
| Publisher | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `waypoint_sequencer/progress`。走行中に位置が `stall_warn_sec` (既定 30 秒) 進まなければ WARN (1 Hz) |

操作は Web UI から行う ([mg_ui](../mg_ui/README.md))。

## 状態

`IDLE`、`ON_STARTING`、`NAVIGATING`、`ON_ARRIVING`、`GOAL_REACHED`、`SUSPENDED` (一時停止)、`ERROR`。遷移の図は [doc/architecture.md](./doc/architecture.md)。

## ウェイポイントの形式

YAML (v2.0)。各ウェイポイントの位置と、到達時のアクション (`service`、`publish`、`load_map`、`amcl_reset`、`wait`、`wait_trigger`、`set_navigation_mode` の 7 種) を書く。AMCL の入/切などは、`service` で指定する。
形式と、v1 からの変換は [doc/waypoint_format.md](./doc/waypoint_format.md)。`waypoint_tool/` に、waypoint-tool 用のスキーマとテンプレート (ウェイポイントと `gnss_transform.yaml` の出力) がある。

## Behavior Tree

| ファイル | 内容 |
| :--- | :--- |
| `behavior_trees/mg_navigate_to_pose.xml` | 通常。前方の障害物では止まって待ち、居座れば首振り・コストマップのクリア・短い後退で回復する |
| `behavior_trees/mg_navigate_to_pose_queue_wait.xml` | `queue_wait`。回避動作をせず、待つだけ (列に詰める動作用) |

`mg_navigation/params/nav2_params.yaml` の `plugin_lib_names` に、次が必要。

- `nav2_goal_updated_controller_bt_node`: 通過点の走行で、ゴールの上書き直後に経路を計画し直すため
- `mg_is_path_clear_condition_bt_node`、`mg_commit_path_action_bt_node`: `mg_navigation` の BT プラグイン (`IsPathClear`、`CommitPath`。[mg_navigation](../mg_navigation/README.md#bt-プラグイン))

### 障害物に出会ったときの動き (両 BT 共通の考え方)

1. **待つ**: 前方に障害物があると、RPP (`FollowPath`) の衝突判定で止まり、`controller_server` の `failure_tolerance` (5 秒) まで待つ。この間も 1 Hz で経路を計画し直し、迂回路が引ければそれに乗って走り出す。消えればそのまま走り出す
2. **計画が失敗しても止めない**: 定期的な計画は `{candidate_path}` に書き、`CommitPath` が今のゴールへの経路のときだけ `{path}` に反映する。計画が失敗しても直前の経路で追従を続け、止まるかどうかは RPP の判定に任せる。ゴールが変わって新しいゴールへの経路がないときだけ、経路を捨ててリカバリーに入る (古いゴールで到着と報告しないため)
3. **回避行動** (normal のみ): 5 秒待っても空かなければ `FollowPath` が失敗し、リカバリーで次の段階を 1 つ行ってから走行に戻る。段階は順に、首振り (+0.5rad) → コストマップのクリアと 3 秒の待機 → 逆向きの首振り (-1.0rad) → 3 秒の待機 → 0.3m の後退 (0.15m/s)。首振りと後退の後は経路を計画し直す。後退は危険なので最後の手段にしている
4. **空いたらすぐ戻る**: リカバリーの間、`IsPathClear` が「RPP が衝突判定なしで走り出せるか」を RPP と同等以上の基準で調べる。塞がっていた経路が空いた状態 (コストマップ 2 枚以上、1 秒以上) になったら、回避行動の途中でも中断して走行に戻る (除去から 1.5 秒程度)。最初から空いている失敗 (スリップなど障害物以外が原因) では中断せず、回避行動を行う

注意:

- リカバリーの試行回数は 99999 (実質無制限)。障害物が居座ると、上の段階を「RPP の待ち 5 秒 + 段階 1 つ」の周期で際限なくくり返す (1 周約 40 秒)。詰まりは `/diagnostics` の `waypoint_sequencer/progress` で知らせる
- `IsPathClear` で回避行動を中断すると、次のリカバリーは段階 1 (首振り) から始まる (Nav2 の `RoundRobin` が halt で位置を戻すため)
- 首振りが再計画に効くのは、向きを考慮する Smac Lattice (`global_planner:=smac_lattice`、既定) の場合。NavFn では向きを変えても計画は変わらない
- `CommitPath` は、経路の終点がゴールから `goal_tolerance` (0.6m。`plan_goal_match_tolerance` と同じ) 以内かで「今のゴールへの経路か」を判定する。0.6m より近いゴールが続くと区別できない
- `collision_monitor` のポリゴン (今はすべて無効) を有効にすると、RPP が止めないのに `collision_monitor` が止める状況ができる。このとき `IsPathClear` (RPP 基準) は空いていると判定するので、「空いたら戻る」が働かない場合がある。有効にするときは見直す

## 構成

```
mg_waypoint_navigation/        Python ライブラリ (waypoint.py = データモデル、waypoint_sequencer/ = FSM・Nav2 クライアント・アクション)
scripts/                       ノード (sequencer、editor、migrate)
launch/  behavior_trees/  rviz/
waypoint_tool/                 waypoint-tool 用のスキーマとテンプレート
test/                          pytest
doc/                           architecture.md、waypoint_format.md
```

## テスト

```bash
make test pkg=mg_waypoint_navigation
```

FSM の遷移、Nav2 クライアント (`navigator`。詰まりの検知を含む)、ウェイポイントの読み込み、地図の読み込み、コストマップの切り替えを調べる。
BT の動きは、シナリオテスト (`dynamic_stop_and_resume`、`dynamic_persistent_recovery`、`dynamic_recovery_resume` など。[mg_scenario_test](../mg_scenario_test/README.md)) で確かめる。

## 依存

`mg_msgs`、`nav2_msgs`、`tf2_ros`、`std_srvs`、`visualization_msgs`。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/architecture.md](./doc/architecture.md) | 構成、FSM、衝突対応、ROS インターフェース、到達判定、スレッドモデル |
| [doc/waypoint_format.md](./doc/waypoint_format.md) | ウェイポイントの YAML 形式、アクション、waypoint-tool での作成、v1 からの変換 |
