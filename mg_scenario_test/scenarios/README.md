# 同梱シナリオ

`make scenario-test SCENARIO=<名前>` で実行します。所要時間はヘッドレス・GPU 構成での目安で、スタックの起動時間を含みます。

| シナリオ | タグ | 内容 | 走行 | 状態 (確認結果) |
|---|---|---|---|---|
| `example_inline_goals` | example, nav2_goals, smoke | インラインの 2 ゴール。静的障害物を前もって配置し、走行中に人が飛び出す | nav2_goals | PASSED (約 1 分) |
| `example_waypoints_file` | example, nav2_goals | `poses` 形式のファイル (`data/warehouse_poses.yaml`) の 3 ゴール。waypoint index ごとの障害物 | nav2_goals | PASSED (約 2 分) |
| `example_waypoints_sequencer` | example, mg_sequencer | waypoint_sequencer に走行を委ね、通過を status で判定。人の飛び出し | mg_sequencer | PASSED (9 waypoint、約 4 分) |
| `nav_unreachable_goal` | smoke, negative, nav2_goals | 地図の外のゴールへの走行が失敗すること (異常系) | nav2_goals | PASSED (約 35 秒) |
| `collision_persistent_recovery` | collision | 至近距離の障害物が居座ると collision_monitor が STOP し、BackUp のリカバリーが動いて、除去後に到達する | mg_sequencer | PASSED (約 4 分) |
| `collision_temporary_block` | collision, known_issue | 至近距離の障害物が短時間で消えれば、リカバリーなしで再開する | mg_sequencer | **FAILED (既知の問題)** |
| `pedestrian_crossing` | dynamic, nav2_goals | 歩行者 (移動する障害物) が進路を横切る。LiDAR の最小測距 0.15 m 以上を確認 | nav2_goals | 不安定 (3 回中 2 回 PASSED) |

## 既知の問題

- **`collision_temporary_block`**: 現在の Nav2 設定では、障害物の出現から約 1 秒以内に controller が
  `Controller patience exceeded` で FollowPath を中断し、collision_monitor の STOP や progress_checker の許容時間
  (5 秒) より先に BackUp などのリカバリーが始まる。そのため「STOP して待つだけで再開」を満たさず、
  monitor (`collision_monitor_action` STOP が発動、`bt_node` BackUp が発火しない) が FAILED を返す。
  挙動を直したら `known_issue` タグを外して `smoke` に戻すこと。
- **`pedestrian_crossing`**: 同一条件で 3 回実行して 1 回、歩行者への急接近 (0.15 m 未満) と走行失敗が起きた。
  ロボット側の挙動のばらつきで、`smoke` には含めていない。

`smoke` タグ (`make scenario-test-all TAGS=smoke`) は、`example_inline_goals` (正常系) と `nav_unreachable_goal` (異常系) で、合計 2 分程度です。
