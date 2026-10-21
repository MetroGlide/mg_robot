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

| 引数 | 内容 |
| :--- | :--- |
| `simulation` | `use_sim_time` |
| `load_path` | ウェイポイントの YAML |
| `initial_localization_map` | 起動時に `map_server` が読んでいる測位用の地図 (`~/loaded_maps` の初期値) |
| `initial_planning_map` | 起動時に `planning_map_server` が読んでいる計画用の地図 |
| `publish_waypoint_status` | `~/status` を配信するか |
| `publish_activity` | `~/activity` (走行の「いまの活動」) を配信するか。`false` なら、その入力の購読もしない |
| `activity_nav_status_topic` | RViz の 2D Goal Pose などシーケンサを通らないゴールの走行を、`~/activity` の `external_goal` として出すための入力。空にすると、その検出だけ止まる |

### waypoint_editor.launch.py

環境変数 `MAP_PATH` が必須で、そのディレクトリの `map_list.txt` (1 行に 1 つの地図の YAML。`#` で始まる行はコメント) に書かれた地図を、すべて読み込んで表示する。

| 引数 | 内容 |
| :--- | :--- |
| `simulation` | `use_sim_time` |
| `load_path` | 編集を始めるときに読み込む YAML |
| `save_path` | 保存先。サービス `save_waypoints` は、ファイルが既にあると失敗する。`force_save_waypoints` は上書きする |

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
| Publisher | `~/status` | `mg_msgs/SequencerStatus` | 状態 |
| Publisher | `~/activity` | `mg_msgs/NavigationActivity` | 走行の「いまの活動」の要約 (transient_local)。表示用。詳細は [doc/architecture.md](./doc/architecture.md#いまの活動-activity) |
| Publisher | `~/waypoints` | `mg_msgs/WaypointList` | 読み込んだウェイポイント |
| Publisher | `~/waypoints_markers` | `visualization_msgs/MarkerArray` | RViz2 用のマーカー |
| Publisher | `~/loaded_maps` | `mg_msgs/LoadedMaps` | 読み込み済みの地図 |
| Publisher | `~/navigation_mode` | `std_msgs/String` | 次のゴールで使うモードと BT |
| Publisher | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `waypoint_sequencer_node: progress` (`diagnostic_updater`)。走行中に位置が `stall_warn_sec` 進まなければ WARN。集約で `/Robot/Navigation/Sequencer` |

操作は Web UI から行う ([mg_ui](../mg_ui/README.md))。

## 状態

`IDLE`、`ON_STARTING`、`NAVIGATING`、`ON_ARRIVING`、`GOAL_REACHED`、`SUSPENDED` (一時停止)、`ERROR`。遷移の図は [doc/architecture.md](./doc/architecture.md)。

## ウェイポイントの形式

YAML (v2.0)。各ウェイポイントの位置と、到達時のアクション (`service`、`publish`、`load_map`、`amcl_reset`、`wait`、`wait_trigger`、`set_navigation_mode` の 7 種) を書く。AMCL の入/切などは、`service` で指定する。
形式と、v1 からの変換は [doc/waypoint_format.md](./doc/waypoint_format.md)。`waypoint_tool/` に、waypoint-tool 用のスキーマとテンプレート (ウェイポイントと `gnss_transform.yaml` の出力) がある。

## Behavior Tree

| ファイル | 内容 |
| :--- | :--- |
| `behavior_trees/mg_navigate_to_pose.xml` | 通常。前方の障害物では止まって待ち、居座れば引き直し・コストマップのクリア・短い後退・首振りを順に試して回復する |
| `behavior_trees/mg_navigate_to_pose_queue_wait.xml` | `queue_wait`。回避動作をせず、待つだけ (列に詰める動作用) |

`mg_navigation/params/nav2_params.yaml` の `plugin_lib_names` に、次が必要。

- `nav2_goal_updated_controller_bt_node`: 通過点の走行で、ゴールの上書き直後に経路を計画し直すため
- `mg_is_path_clear_condition_bt_node`、`mg_is_path_blocked_condition_bt_node`、`mg_commit_path_action_bt_node`: `mg_navigation` の BT プラグイン (`IsPathClear`、`IsPathBlocked`、`CommitPath`。[mg_navigation](../mg_navigation/README.md#bt-プラグイン))

### 障害物に出会ったときの動き (両 BT 共通の考え方)

1. **待つ**: 前方に障害物があると、RPP (`FollowPath`) の衝突判定で止まり、`controller_server` の `failure_tolerance` まで待つ。経路の近くが塞がっている間は、定期的な再計画を止める (`IsPathBlocked`)。急に現れた障害物は、迂回路が引けても乗らずに待ち、消えればそのまま走り出す。居座ればリカバリーで迂回する。遠くの障害物は、定期的な再計画で滑らかに避ける。XML の定期的な再計画の `Inverter` と `IsPathBlocked` の行を外すと、待たずに迂回路へすぐ乗る動きになる (リカバリーはそのまま使える)
2. **計画が失敗しても止めない**: 定期的な計画は `{candidate_path}` に書き、`CommitPath` が今のゴールへの経路のときだけ `{path}` に反映する。計画が失敗しても直前の経路で追従を続け、止まるかどうかは RPP の判定に任せる。ゴールが変わって新しいゴールへの経路がないときだけ、経路を捨ててリカバリーに入る (古いゴールで到着と報告しないため)
3. **回避行動** (normal のみ): `failure_tolerance` の間待っても空かなければ `FollowPath` が失敗し、リカバリーに入る。1 回のリカバリーの中で次の段階を順に試し、どこかで引き直した経路が通れた時点で走行に戻る (角度・時間・距離・速度は BT の XML と `behavior_server` の設定)
   0. 自己位置が信頼できる (`IsLocalizationHealthy`) とき、ロボットが計画用地図の障害物 (進入禁止の領域) にめり込んでいれば、領域の外の最も近い位置を始点にして計画し、領域から出る (`FindFreeStart`)。めり込んでいなければ、この段階は飛ばす。自己位置が疑わしいときは自己位置の復旧に任せる。この段階を XML から外すと脱出しない
   1. 今のコストマップで引き直す
   2. コストマップをクリアし、作り直されるまで待ってから引き直す
   3. 近くの実在の障害物で今の経路が塞がっているときだけ、ゆっくり短く後退して引き直す
   4. ゆっくり首振り (左、右の順) して、そのたびに引き直す
   5. 待ってから走行に戻る (RPP の待ちからくり返す)

   「通れた」は、引き直した経路を `IsPathBlocked` (RPP と同じ判定。経路の向きへのその場の回転を含む) で確かめる。段階の間に RPP の待ちは挟まない。計画の失敗、塞がった経路、後退・首振りの衝突による中止は、どれも次の段階へ進む
4. **空いたらすぐ戻る**: リカバリーの間、`IsPathClear` が「RPP が衝突判定なしで走り出せるか」を RPP と同等以上の基準で調べる。塞がっていた経路が空いた状態 (コストマップ複数枚、一定時間以上) になったら、回避行動の途中でも中断して走行に戻る。最初から空いている失敗 (スリップなど障害物以外が原因) では中断せず、回避行動を行う

注意:

- **極端な遠回りの経路には乗らない**: 前方が塞がったときに、プランナが逆走して建物の反対側から回り込むような経路を引くことがある。`CommitPath` は、ゴールへ届く計画結果の長さが「始点からゴールまでの直線距離 × `max_detour_ratio` + `max_detour_margin`」を超えると、計画の失敗と同じに扱って乗らず、塞がりが解けるまで待つ (待つ間は上のリカバリーをくり返す)。リカバリー中に棄却しているときは、UI の「いまの活動」に `detour_rejected` が出る (リカバリーの表示に代わる)。通過点で新しいゴールへの最初の計画を棄却すると、RPP の待ちを挟まずに、後退・首振りの段階を毎周期くり返す。上限の基準は、その時点の位置からゴールまでの直線距離なので、採用した遠回りを進んでゴールから離れると上限も広がる (少しずつ延びる遠回りは防がない)。BT の XML の `CommitPath` に `max_detour_ratio="0"` を書くと無効になる (ポートは [mg_navigation の文書](../mg_navigation/doc/bt_plugins.md#commitpath))。基準は直線距離なので、間隔の粗いウェイポイントなど、正常な経路が直線距離より大きく長い区間では棄却して待ち続ける。ウェイポイントは経路に沿って置くか、上限を緩める
- リカバリーの試行回数は実質無制限。障害物が居座ると、「RPP の待ち + 上の段階すべて」の周期で際限なくくり返す。詰まりは診断 `waypoint_sequencer_node: progress` で知らせる。リカバリーは中断・終了のたびに段階 1 から始まる
- 引き直しを首振りより先に置くのは、引き直した経路の確かめで向きの問題 (経路の向きへの回転で当たるか) を判定済みだから。コストマップはセンサの層だけで、向きを変えても見える範囲は変わらない。首振りが効くのは、向きを考慮する Smac Lattice (`global_planner:=smac_lattice`) で経路が引けやすくなる場合で、既定の NavFn では効かない
- 後退は、計画が他の理由 (自己位置のずれ、地図との不一致、ゴールが通れない場所) で失敗し、近くに実在の障害物がないときは行わない。ローカルコストマップはセンサの層だけ (odom 座標) なので、自己位置がずれても実在の障害物の位置は正しい
- 既知の制約: 自己位置がゆっくりずれ (AMCL のずれ、静止中の GNSS の引き込み)、地図に沿って引いた経路が近くの実在の壁に掛かると、リカバリーのたびに後退をくり返す。後退は `behavior_server` がローカルコストマップとフットプリントで先読みして確かめ、致死・未知のセルで止まるので、後方が見えていれば実在の障害物には当たらない。後退の合計の上限や自己位置の状態での抑止は、自己位置飛びの対策 (将来の拡張) で検討する
- 後退・首振りの `cmd_vel` は `collision_monitor` を経由しない。安全は `behavior_server` の衝突の確かめだけに頼る (見えない物と実機で確かめることは [保護の外](../mg_navigation/doc/costmap_and_collision.md#衝突の防止-collision_monitor))
- `CommitPath` は、経路の終点がゴールから `goal_tolerance` (`plan_goal_match_tolerance` と同じ値にする) 以内かで「今のゴールへの経路か」を判定する。この距離より近いゴールが続くと区別できない
- `collision_monitor` のポリゴンが有効なとき、RPP が止めないのに `collision_monitor` が止める状況がありうる。このとき `IsPathClear` (RPP 基準) は空いていると判定するので、「空いたら戻る」が働かない場合がある。`collision_monitor` のポリゴンは、RPP より先に反応しないように作ってある ([衝突の防止](../mg_navigation/doc/costmap_and_collision.md#衝突の防止-collision_monitor))

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

## イベントログ

状態の遷移、走行、アクション、Nav2 のゴールを、`evt=<domain>.<event> key=value ...` の 1 行でログに出す ([doc/monitoring.md](../doc/monitoring.md))。`mg_ui` のイベント一覧と、`make log-events` の時系列に出る。

| イベント | レベル | 内容 |
| :--- | :--- | :--- |
| `sequencer.state_changed` | INFO / WARN (SUSPENDED) / ERROR (ERROR) | FSM の遷移 (`from`、`to`、`index`、`total`) |
| `sequencer.run_started` | INFO | 走行の開始 (一時停止からの再開は含まない) |
| `sequencer.run_finished` | INFO / ERROR | 走行の終了 (`result`: `goal_reached` / `error` / `stopped`) |
| `sequencer.paused` / `sequencer.resumed` | WARN / INFO | 一時停止と再開 (`requesters`) |
| `sequencer.waypoint_reached` | INFO | ウェイポイントに着いた (`result`: `SUCCEEDED` / `PASSED`) |
| `sequencer.manual_goal_finished` | INFO | 手動ゴールが終わった (`result`) |
| `nav.goal_sent` | INFO | Nav2 にゴールを送った (`index`、`x`、`y`、`mode`) |
| `nav.goal_result` | INFO / WARN (CANCELED) / ERROR | ゴールの結果 (`result`、`duration_sec`、`recoveries`) |
| `action.started` / `action.succeeded` / `action.failed` | INFO / INFO / ERROR | 到着時アクションの実行 (`type`、`index`、`duration_sec`、`error`) |
| `node.started` | INFO | 起動 |

アクションは、失敗したら `ActionFailed` を投げる (`actions/base.py`)。`ActionExecutor` が `action.failed` に出し、次のアクションへ進む。サービスの応答が無かったとき (`BaseAction._call_service`) と、汎用の `service` アクションで応答の `success` が false のときも失敗になる。
