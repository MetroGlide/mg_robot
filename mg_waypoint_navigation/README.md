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

操作は Web UI から行う ([mg_ui](../mg_ui/README.md))。

## 状態

`IDLE`、`ON_STARTING`、`NAVIGATING`、`ON_ARRIVING`、`GOAL_REACHED`、`SUSPENDED` (一時停止)、`ERROR`。遷移の図は [doc/architecture.md](./doc/architecture.md)。

## ウェイポイントの形式

YAML (v2.0)。各ウェイポイントの位置と、到達時のアクション (`service`、`publish`、`load_map`、`amcl_reset`、`wait`、`wait_trigger`、`set_navigation_mode` の 7 種) を書く。AMCL の入/切などは、`service` で指定する。
形式と、v1 からの変換は [doc/waypoint_format.md](./doc/waypoint_format.md)。`waypoint_tool/` に、waypoint-tool 用のスキーマとテンプレート (ウェイポイントと `gnss_transform.yaml` の出力) がある。

## Behavior Tree

| ファイル | 内容 |
| :--- | :--- |
| `behavior_trees/mg_navigate_to_pose.xml` | 通常。経路の追従や計画に失敗したとき、待機・後退・コストマップのクリアで回復する |
| `behavior_trees/mg_navigate_to_pose_queue_wait.xml` | `queue_wait`。回避動作をせず、待つだけ (列に詰める動作用) |

`mg_navigation/params/nav2_params.yaml` の `plugin_lib_names` に、`nav2_goal_updated_controller_bt_node` が必要 (通過点の走行で、ゴールの上書き直後に経路を計画し直すため)。

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

FSM の遷移、Nav2 クライアント (`navigator`)、ウェイポイントの読み込み、地図の読み込み、コストマップの切り替えを調べる。

## 依存

`mg_msgs`、`nav2_msgs`、`tf2_ros`、`std_srvs`、`visualization_msgs`。

## ドキュメント

| ファイル | 内容 |
| :--- | :--- |
| [doc/architecture.md](./doc/architecture.md) | 構成、FSM、衝突対応、ROS インターフェース、到達判定、スレッドモデル |
| [doc/waypoint_format.md](./doc/waypoint_format.md) | ウェイポイントの YAML 形式、アクション、waypoint-tool での作成、v1 からの変換 |
