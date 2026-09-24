# 同梱シナリオ

`make scenario-test SCENARIO=<名前>` で実行します。所要時間はヘッドレス・GPU 構成での目安で、スタックの起動時間を含みます。

| シナリオ | タグ | 内容 | 走行 | 状態 (確認結果) |
|---|---|---|---|---|
| `example_inline_goals` | example, nav2_goals, smoke | インラインの 2 ゴール。静的障害物を前もって配置し、走行中に人が飛び出す | nav2_goals | PASSED (約 1 分) |
| `example_waypoints_file` | example, nav2_goals | `poses` 形式のファイル (`data/warehouse_poses.yaml`) の 3 ゴール。waypoint index ごとの障害物 | nav2_goals | PASSED (約 2 分) |
| `example_waypoints_sequencer` | example, mg_sequencer | waypoint_sequencer に走行を委ね、通過を status で判定。人の飛び出し | mg_sequencer | PASSED (9 waypoint、約 4 分) |
| `nav_unreachable_goal` | smoke, negative, nav2_goals | 地図の外のゴールへの走行が失敗すること (異常系) | nav2_goals | PASSED (約 35 秒) |
| `collision_persistent_recovery` | collision | 至近距離の障害物が居座ると STOP・リカバリー (BackUp) が動き、除去後に到達する | mg_sequencer | 不安定 (2 回中 1 回 PASSED。詳細は下記) |
| `collision_temporary_block` | collision, smoke | 至近距離の障害物が短時間で消えれば、リカバリーなしで待つだけで再開する | mg_sequencer | PASSED (2 回中 2 回、約 3.5 分) |
| `cone_line_obstacle` | obstacle_detection, nav2_goals | コーンを x=2.0 の直線状に配置 (もとは warehouse.sdf に常設)。通れない隙間を避けて回り込み、接触しないこと | nav2_goals | PASSED (3 回中 3 回、約 1 分) |
| `pedestrian_crossing` | dynamic, nav2_goals | 歩行者 (移動する障害物) が進路を横切る。LiDAR の最小測距 0.15 m 以上を確認 | nav2_goals | 不安定 (3 回中 2 回 PASSED) |

## 既知の問題・不安定なシナリオ

- **`collision_persistent_recovery`**: 障害物 (ロボット前方 0.5 m) が居座っても、ロボットが経路を変えて迂回した場合は
  BackUp が不要になり、monitor (`bt_node` BackUp が発火する) が FAILED を返す。2 回中 1 回で発生。
  リカバリーが必ず起きる配置 (迂回できない障害物) に改めるか、期待値を見直す必要がある。
- **`pedestrian_crossing`**: 同一条件で 3 回実行して 1 回、歩行者への急接近 (0.15 m 未満) と走行失敗が起きた。
  ロボット側の挙動のばらつき。

`smoke` タグ (`make scenario-test-all TAGS=smoke`) は、`example_inline_goals` (正常系)、`nav_unreachable_goal` (異常系)、
`collision_temporary_block` (一時的な障害物) で、合計 5 分程度です。

## 解決済みの問題 (参考)

- `cone_line_obstacle` が失敗していた: 間隔 0.8m のコーンの隙間 (ロボット幅 0.6m では通れない) を、プランナが通れると誤認して
  経路を引き、RPP が衝突と判定して進めず詰まっていた。膨張レイヤーの内接半径がフットプリントの後端 (0.2m) で決まり、
  半幅 (0.3m) より小さかったため。`nav2_params.yaml` の両コストマップに `footprint_padding: 0.1` を追加して解決した。

- 一時的な障害物で、出現から 1 秒足らずで FollowPath が中断されリカバリー (Wait/BackUp) に入っていた。
  原因は `controller_server.failure_tolerance: 0.5`。`5.0` (progress_checker の `movement_time_allowance` と同じ) にして、
  待つだけで再開できるようにした。**`-1` (無期限) は不可**: progress_checker の失敗まで無視され、詰まると永久に停止する。
- world に常設されていたコーン (`warehouse.sdf`) が、スタート付近の進路をふさぎ、他のシナリオの走行を妨げていた。
  `cone_line_obstacle` へ移した。
