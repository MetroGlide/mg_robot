# mg_msgs

MG-01 の独自メッセージ・サービスの定義。ノードは持たない。

## メッセージ (`msg/`)

| 型 | 内容 | 配信元 | 購読・利用先 |
| :--- | :--- | :--- | :--- |
| `LoadedMaps` | `waypoint_sequencer` が `map_server` に読み込ませた地図 (測位用・計画用の YAML パス) | `mg_waypoint_navigation` の `map_loader` | Web UI |
| `PauseRequest` | ウェイポイントシーケンサへの停止要求 (要求元・有効か・理由) | Web UI、`mg_scenario_test` | `waypoint_sequencer_node` |
| `SequencerStatus` | シーケンサの状態 (FSM の状態、現在のウェイポイント、一時停止中か、残りの距離、実行中のアクション、トリガー待ちか、`~/trigger` を受け付けられるか) | `waypoint_sequencer_node` (トピック `~/status`) | Web UI、`mg_diagnostics`、`mg_scenario_test`、`mmi_switch_node` |
| `NavigationActivity` | 走行の「いまの活動」の要約 (障害物での停止、リカバリー、トリガー待ち、RViz などシーケンサを通らないゴールの走行など)。表示用で、制御の入力にはしない。状態ランプの意味 (`lamp_response`・`lamp_motion`) を含む | `waypoint_sequencer_node` (トピック `~/activity`、transient_local) | Web UI、`mg_scenario_test`、`mmi_switch_node` |
| `ActivityLamp` / `ActivityLampTable` | 活動の code ごとの状態ランプの意味の全体 | `waypoint_sequencer_node` (トピック `~/activity_lamps`、transient_local) | Web UI (ランプの見方) |
| `WaypointInfo` | ウェイポイント 1 点の情報 (位置、到達許容 (未使用)、通過点か、通過の半径、到達時のアクションの種類) | `waypoint_sequencer_node` | `WaypointList` の要素 |
| `WaypointList` | 読み込んだウェイポイントの一覧 | `waypoint_sequencer_node` (トピック `~/waypoints`、transient_local) | Web UI、`mg_scenario_test`、`waypoint_editor_node` |
| `PoseGraphDiff` | SLAM のポーズグラフの差分 (新しいノード、エッジ、GNSS の prior、ループ) | `mg_slam` の旧 Python 版 | `mg_slam` のプレビュー、Web UI |

## サービス (`srv/`)

| 型 | 内容 | サーバ | クライアント |
| :--- | :--- | :--- | :--- |
| `StartSequence` | ウェイポイントの走行を開始する (`countdown_ms` 後) | `waypoint_sequencer_node` (`~/start`) | Web UI、`mg_scenario_test` |
| `TriggerSequence` | 物理ボタンなどからの走行開始の要求。IDLE のときだけ受け付け、断ったら理由を返す (要求元を渡す) | `waypoint_sequencer_node` (`~/trigger`) | `mmi_switch_node` |
| `SendGoal` | BT を指定して、手動でゴールを 1 つ送る (走行中でないときだけ受け付ける) | `waypoint_sequencer_node` (`~/navigate_to_pose`) | Web UI |
| `LoadMaps` | 測位用・計画用の地図を `map_server` に読み込ませる (空文字の側は変更しない) | `waypoint_sequencer_node` (`~/load_map`) | Web UI |
| `GetPoseGraph` | ポーズグラフ全体を取得する | `mg_slam` の旧 Python 版 | `mg_slam` のプレビュー |
| `ResetSimRobotPose` | シミュレータのロボットの位置を戻す | なし (どこからも使われていない) | なし |

自己位置監視の状態 (`LocalizationStatus`) とゲートの調停の状態 (`GateState`) は、[localization_supervisor_msgs](../localization_supervisor/localization_supervisor_msgs/README.md) にある。

## slam_gnss_2d_msgs との関係

現役の SLAM (`slam_gnss_2d`) は、`PoseGraphDiff` と `GetPoseGraph` を、自分のメッセージパッケージ `slam_gnss_2d_msgs` から使う。同名の型が `mg_msgs` にもあるのは、旧 Python 版のため。
Web UI は両方のスキーマを登録している (`mg_ui/mg_web_ui/frontend/src/ros/schemas.ts`)。`SaveSlamMap` は `slam_gnss_2d_msgs` にだけある。

## 使い方

型の追加・変更は、`CMakeLists.txt` の `rosidl_generate_interfaces` にファイルを追加する。変更後は、型を使うパッケージを含めてビルドし直す。
Web UI から使う型は、`schemas.ts` にスキーマ名を登録する ([mg_ui の README](../mg_ui/README.md))。
