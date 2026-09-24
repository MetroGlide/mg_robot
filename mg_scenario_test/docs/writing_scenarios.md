# MG-01 向けシナリオの書き方

書式の全項目は [scenario_format.md](../../sim_scenario_test/docs/scenario_format.md)、MG 固有の型は [mg_plugins.md](mg_plugins.md) を参照。
ここでは典型パターンと、つまずきやすい点をまとめます。まず同梱シナリオ ([一覧](../scenarios/README.md)) を真似るのが早道です。

## 基本の骨組み

```yaml
version: "2.0"
name: my_scenario
profile: mg01
world: warehouse
tags: [my_tag]

setup:                       # 走行前: 初期位置へ移動し、自己位置とコストマップを初期化
  - respawn: { pose: { x: 0.0, y: 0.0, yaw: 0.0 }, settle_sec: 3.0 }
run:                         # 走行: Nav2 のゴール、または waypoint_sequencer
  nav2_goals:
    goals:
      - pose: { x: 5.0, y: 2.0, yaw: 0.0 }
expect: [reached_all, { time_limit: { sec: 90 } }]
```

## パターン集

**障害物を走行前に置く・到達後に消す** — ゴールの `before` / `after` (`nav2_goals`)、または waypoint の `hooks` (`mg_sequencer`)。

**走行中に障害物を出す (飛び出し)** — `timeline` で、走行の進み具合をきっかけにする。

```yaml
timeline:
  - name: person_jumps_out
    when: { robot_travelled: { distance: 1.0 } }        # 1m 進んだら
    do:
      - spawn: { obstacle: person, pose: { frame: robot, x: 2.0, yaw: 3.14 } }   # ロボット正面 2m
      - delay: { sec: 3.0 }
      - despawn: { obstacle: person }
```

`at_time` (時間) より `robot_travelled` / `robot_near` / `goal_started` の方が、ロボットの挙動や速度に左右されず再現しやすい。

**移動する障害物 (歩行者)** — `spawn` の後に `move_obstacle` (`speed` と `to`)。実例は `pedestrian_crossing.yaml`。

**異常系 (到達できないことを確認)** — `expect: [{ navigation_fails: { index: 0 } }]`。実例は `nav_unreachable_goal.yaml`。

**リカバリー動作の確認** — `monitors` で BT や collision_monitor を見る。

```yaml
monitors:
  - collision_monitor_action: { action: STOP, expect: occurs }
  - bt_node: { node: BackUp, status: RUNNING, expect: never }      # BackUp が動かないこと
```

**自己位置推定の誘拐 (kidnapped robot)** — `teleport` はロボットをシミュレータ上で動かすだけで自己位置推定に通知しない。
AMCL の復帰処理の確認に使える。

**乱数で揺らす** — `spawn` の `jitter` と、シナリオの `seed`。CLI の `--seed` で上書き、`REPEAT` ごとに +1。

## つまずきやすい点

- **z 座標**: `primitive` の `size.z` は中心の高さ基準。床に置くなら `z` を高さの半分にする。
  ロボットの `respawn` の `z` はプロファイルの `robot_spawn_z` (0.05) が加算される。
- **ゴールの選び方**: 地図上で自由に見えても、シミュレーション内の障害物 (棚、`Person 1` など、地図にない物体) や
  膨張領域のために、プランナが経路を作れない座標がある (例: (8, 0) には到達できない)。新しいゴールは実際に到達できるか
  一度確かめる。到達を確認済みの座標は `scenarios/data/` の waypoint ファイルを参照。
- **狭い通路・回廊のシナリオ**: 回廊が進路の脇にあるだけだと、コストの低い迂回が選ばれて回廊を通らない (通ったことを確認したい場合は、
  回廊の内側からスタートし、後方と両脇を壁で囲んで出口以外の抜け道をなくす)。通れる幅の回廊の内側からのスタートは、Smac Lattice が
  計画に失敗するため、回帰シナリオにできていない ([既知の問題](../scenarios/README.md))。通れない幅の回廊は `narrow_corridor_blocked` を参照。
  壁は `primitive` の box で作り、スタート地点の近くに壁の端を置かない (スタートが膨張領域に入ると `Starting point in lethal space` で計画できない)。
  ゴールは到達確認済みの座標 (`scenarios/data/`) から選ぶ。
- **重いモデル**: Fuel のメッシュモデルは GPU なしだとシミュレーション速度を大きく下げる。壁・バリア類は `primitive` を使う。
- **`required` のタイムライン**: 発火しなかったら ERROR になる。ゴールが早く終わって発火前に走行が終わる場合などは
  `required: false` にするか、`when` の条件を見直す。
- **sequencer と `wait_trigger`**: `wait_trigger` を持つ waypoint で止まる場合、`auto_start: true` なら自動で再開する。
  手動で制御したいときは `auto_start: false` にして `sequencer_start` を使う。
- **動く障害物との接触判定**: `move_obstacle` で動かす障害物は `set_pose` で移動するので、物理的に押し返されず、停止中のロボットを通り抜ける。
  `min_scan_range` はこの通り抜けも検出してしまうので、歩行者には `obstacle_clearance` (ロボット自身の動きで、フットプリントが障害物へ近づいた場合だけを判定) を使う。
- **前方 LiDAR**: シミュレーションでは前方 LiDAR は何も配信せず、データが出るのは `/scan_top_lidar` のみ。
  `min_scan_range` の `topic` に注意。
- **新しいファイルを追加したら** `make build svc=scenario-test` (symlink install は既存のファイルしかリンクしない)。

## デバッグ

1. `make scenario-validate` で書式の誤りを先に潰す (未知のキー、未定義の障害物、ワールド名など)。
2. 実行後の `Summary` と `result.json` の `checks` で、どのチェックが落ちたか確認する。`events` にゴールの到達時刻などがある。
3. `launch.log` にシミュレータ・Nav2・runner の全ログがある。`[action]` `[spawn]` `[timeline]` `[nav2_goals]` などで検索する。
4. 目で見たいときは `make scenario-test SCENARIO=... GUI=1`。Nav2 の挙動は RViz (`make rviz2-navigation`) を併用する。
5. 開発中の反復は `make gazebo-simulation` と `make navigation` を別端末で起動し、`make scenario-test-attach` で繰り返す
   (走行を繰り返すと Nav2 の状態が劣化することがある。結果がおかしいときは起動し直す)。
6. ヘッドレスの結果を rosbag で調べたいときは、`launch.log` の内容と `tools/` の `make bag-*` を併用する。
