# MG 固有の拡張

`profiles/mg01.yaml` の `plugins: [mg_scenario_test.plugins]` で読み込まれ、次を登録します
(実装は `mg_scenario_test/plugins.py` と `mg_scenario_test/monitors.py`)。

## 走行ドライバ `mg_sequencer`

走行を `waypoint_sequencer_node` に委ね、`~/status` を監視して waypoint の通過を判定します。
`goal_started` / `goal_reached` / `goal_failed` の `index` は、waypoint リスト上の位置です (`waypoint_index` 属性に waypoint 自身の index も入ります)。

```yaml
run:
  mg_sequencer:
    namespace: waypoint_sequencer_node   # 既定
    waypoints_file: "{world.waypoints}"  # 省略時は sequencer が配信する ~/waypoints トピックから取得
    start_index: 0                       # ここから走行する
    auto_start: true                     # 下記
    countdown_ms: 0                      # ~/start のカウントダウン
    waypoint_timeout_sec: 300            # waypoint 1 つあたりの sim 秒の上限
    idle_stall_sec: 10                   # IDLE のまま進まない場合に ERROR にするまでの秒数
    hooks:                               # waypoint index ごとのフック
      1:
        before: [ ... ]                  # この waypoint へ向かわせる前に同期実行する action
        after:  [ ... ]                  # 通過後に同期実行する action
        countdown_ms: 3000               # この waypoint 向けの ~/start のカウントダウン
```

`auto_start: true` (既定) の動作:

1. 開始時に `~/stop` → `set_next_waypoint_index(start_index)` (反映を確認) → `~/start`。
2. waypoint を通過した後、sequencer が `wait_trigger` で IDLE になっていたら、`hooks.<n>.before` を実行してから `~/start` を呼ぶ。
3. sequencer が IDLE のまま `idle_stall_sec` 進まなければ ERROR (start 忘れの検出)。

`auto_start: false` にすると、start / index 設定はシナリオ側 (`sequencer_start` など) で行います。

**通過の判定**: 今回の走行で一度でも稼働状態 (ON_STARTING / NAVIGATING / ON_ARRIVING / SUSPENDED) を観測してから、
`current_index` が waypoint の index を超えた時点で通過とします (前回の走行の古い status で誤判定しないため)。
sequencer が `ERROR` になったら FAILED です。

## action

| 型 | パラメータ | 内容 |
|---|---|---|
| `sequencer_start` | `countdown_ms`=0, `namespace` | `~/start` を呼ぶ |
| `sequencer_set_index` | `index`, `namespace` | 次に向かう waypoint index を設定 (IDLE / SUSPENDED 時のみ有効。反映を確認し、されなければ ERROR) |
| `sequencer_stop` | `namespace` | `~/stop` を呼ぶ |

## trigger

| 型 | パラメータ | 成立条件 |
|---|---|---|
| `sequencer_state` | `state`, `index`=省略可, `namespace` | sequencer の状態が `state` (`IDLE` / `NAVIGATING` / `ON_ARRIVING` / `GOAL_REACHED` / `SUSPENDED` / `ERROR` 等) になった (`index` 指定時は current_index も一致) |

## monitor

| 型 | パラメータ | 判定 |
|---|---|---|
| `collision_monitor_action` | `action` (`STOP` / `SLOWDOWN` / `APPROACH`), `expect`=occurs (`occurs`/`never`), `topic`=/collision_monitor_state | `nav2_msgs/CollisionMonitorState` (このリポジトリの nav2_pkg 独自) で、collision_monitor がその action を発動した回数から判定 |

## waypoint 形式 `mg`

`mg_waypoint_navigation` の `waypoint.yaml` (version 2.0)。`WaypointsLoader` で読み込み・検証されます。
`nav2_goals` で `waypoints_format: mg` を指定すると、この形式のファイルの姿勢を順に走行します。

**注意**:

- WP の座標へ **直接** 走行すると、シミュレーション上の障害物や地図との差でプランナが経路を作れない場合があります
  (sequencer は通過点として途中で通過判定するため問題になりません)。直接走行するシナリオには
  到達を確認済みのゴール (組み込みの `poses` 形式) を使ってください。
- `reach_tolerance` は到達判定に使われません (`nav2_params.yaml` の `xy_goal_tolerance` が全 waypoint 共通で使われる。
  `mg_waypoint_navigation/doc/waypoint_format.md` 参照)。

## 実機のセンサ故障の注入

コアの `call_set_bool` で、`lidar_publish_controller_node` などの `~/change_publish_state` を呼べば、
センサの配信停止・復旧を再現できます。ただしシミュレーションでは前方 LiDAR (`front_lidar_publish_controller_node`)
が何も配信しないため、意味のあるシナリオにはなりません (確認済み)。
