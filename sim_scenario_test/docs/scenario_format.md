# シナリオ YAML (version 2.0) リファレンス

読み込み時に dataclass 定義に基づいて厳格に検証されます。未知のキー、型の不一致、必須項目の欠落、
未登録の型、未定義の障害物名は、すべてエラーになります (`scenario_cli.py validate` で事前に確認できます)。
`type` を持つ項目は `{型名: {パラメータ}}` (パラメータなしなら `型名` だけ) と書きます。

## トップレベル

```yaml
version: "2.0"                 # 必須。"2.0" 固定
name: my_scenario              # 必須。結果・JUnit の名前
world: warehouse               # 必須。プロファイルの worlds に定義したワールド名
run: { nav2_goals: {...} }     # 必須。走行ドライバ (下記)
profile: mg01                  # プロファイル名またはパス。CLI の --profile で上書き可
description: >                 # 説明 (任意)
tags: [smoke, collision]       # run-all の --tags で絞り込む (任意)
timeout: 180                   # 走行開始からの sim 秒の上限。超えると走行を打ち切り FAILED (任意)
seed: 0                        # 乱数 (spawn の jitter) の種。--seed で上書き
stack_args: {...}              # プロファイルの stack.args を、このシナリオ用に上書き・追加する launch 引数
obstacles: {...}               # 名前付き障害物の定義
setup: [...]                   # 走行前に順に実行する action
timeline: [...]                # 走行中のイベント
monitors: [...]                # 走行中の常時監視
expect: [...]                  # 判定。省略時は reached_all
teardown: [...]                # 走行後に実行する action (生成した障害物は自動で削除される)
```

## stack_args

プロファイルの `stack.args` (ナビゲーションスタックの launch 引数) を、シナリオごとに上書き・追加します
(`run` / `run-all` のみ。`--attach` では無視)。値は変数展開されます。ロボットが読む waypoint ファイルを
シナリオごとに差し替えるときなどに使います。

```yaml
stack_args:
  waypoints_load_path: "pkg://mg_scenario_test/scenarios/data/through_waypoints.yaml"
```

## obstacles

```yaml
obstacles:
  person:  { model: { type: fuel, uri: "OpenRobotics/models/MaleVisitorOnPhone" } }
  box:     { model: { type: primitive, shape: box, size: { x: 0.6, y: 0.6, z: 1.2 } } }
  pole:    { model: { type: primitive, shape: cylinder, size: { radius: 0.2, length: 1.5 }, static: false } }
  custom:  { model: { type: local, path: "pkg://my_pkg/models/thing.sdf" } }
```

| model.type | 指定 | 備考 |
|---|---|---|
| `fuel` | `uri` (`Owner/models/Name` または URL) | 初回はダウンロードされる。ダウンロード失敗は spawn 後の存在確認で ERROR になる |
| `local` | `path` (`pkg://` と `{変数}` に対応) | SDF ファイル |
| `primitive` | `shape` (`box` / `cylinder` / `sphere`)、`size` | `size`: box は x,y,z、cylinder は radius,length、sphere は radius。**z は中心の高さ** (床に置くなら高さの半分) |

`static` (既定 true) は fuel / primitive に適用。メッシュの大きなモデルはソフトウェアレンダリング環境で
シミュレーション速度を大きく下げるため、壁やバリア類は `primitive` を推奨します。

## 姿勢 (PoseSpec)

```yaml
pose: { frame: map, x: 1.0, y: 2.0, z: 0.0, yaw: 1.57 }   # frame 省略時は map
```

`frame`: `map` / `world` / `robot` (現在のロボット姿勢基準の相対座標)。詳細は architecture.md の「座標系」。

## run (走行ドライバ)

### nav2_goals (組み込み)

Nav2 の NavigateToPose でゴールを順に走行します。`goals` と `waypoints_file` はどちらか一方を指定します。

```yaml
run:
  nav2_goals:
    goals:
      - pose: { x: 5.0, y: 2.0, yaw: 0.0 }
        before: [ ... ]     # このゴールへ向かう前に同期実行する action
        after:  [ ... ]     # 到達後に同期実行する action
    # または
    waypoints_file: "{world.waypoints}"
    waypoints_format: poses  # 組み込みは poses。ロボット固有形式はプラグインが提供
    hooks:                   # waypoints_file 使用時の waypoint index ごとの before / after
      1: { before: [ ... ], after: [ ... ] }
    goal_timeout_sec: 300.0  # ゴール 1 つあたりの sim 秒の上限
    behavior_tree: ""        # 使う BT の XML パス (省略時は Nav2 の既定)
```

`poses` 形式のファイル: `waypoints: [{x: .., y: .., yaw: ..}, ...]`。
ゴールの index は 0 始まりで、trigger (`goal_started` など) から参照できます。

その他のドライバ (例: `mg_sequencer`) はプラグインが提供します。`list-types --profile <name>` で一覧できます。

## action

| 型 | パラメータ | 内容 |
|---|---|---|
| `respawn` | `pose`, `settle_sec`=2, `set_initial_pose`=true, `clear_costmaps`=true, `converge_tolerance`=0.5, `converge_timeout_sec`=10 | ロボットを移動 → `/initialpose` → 待機 → 自己位置が指定位置から `converge_tolerance` [m] 以内に収束するまで待つ (収束しなければ ERROR) → コストマップをクリア |
| `teleport` | `pose`, `entity`=ロボット | エンティティを移動する。**自己位置推定には通知しない** (誘拐ロボット問題の再現に使う) |
| `set_initial_pose` | `pose` | 自己位置推定に初期位置を与える |
| `clear_costmaps` | なし | グローバル・ローカルのコストマップをクリア |
| `spawn` | `obstacle`, `pose`, `jitter`=0 | 障害物を配置する。生成の確認まで行う。`jitter` [m] は x,y の一様乱数 (seed で再現可能) |
| `despawn` | `obstacle` | 障害物を削除する |
| `move_obstacle` | `obstacle`, `to`, `speed`=1.0 [m/s], `rate_hz`=5 | 配置済みの障害物を現在位置から `to` まで等速で直線移動させる |
| `call_set_bool` | `service`, `value` | `std_srvs/SetBool` を呼ぶ (センサの配信停止など故障の注入) |
| `delay` | `sec` | sim 時間で待つ |
| `log` | `message` | ログとイベントに記録する |

## trigger (`timeline[].when` / `until`)

| 型 | パラメータ | 成立条件 |
|---|---|---|
| `at_time` | `sec` | 走行開始から `sec` 秒 (sim 時間) |
| `goal_started` | `index` | `index` 番目のゴールへの走行を開始 (`before` 実行後) |
| `goal_reached` | `index` | `index` 番目のゴールに到達 |
| `goal_failed` | `index` | `index` 番目のゴールへの走行が失敗 |
| `robot_travelled` | `distance` | 走行開始からのロボットの移動距離 (経路長) が `distance` [m] に達した |
| `robot_near` | `x`, `y`, `radius`, `frame`=map (`map`/`world`) | ロボットが指定点から `radius` [m] 以内 |

`robot_travelled` / `robot_near` は速度や RTF に左右されないので、`at_time` より再現性が高くなります。

## timeline

```yaml
timeline:
  - name: person_crosses          # 結果に表示される名前 (任意)
    when: { robot_travelled: { distance: 1.0 } }
    until: { goal_reached: { index: 0 } }   # 成立したら実行中の do を打ち切る (任意)
    required: true                # 一度も発火しなければ ERROR (既定 true)
    do:
      - spawn: { obstacle: person, pose: { frame: robot, x: 2.0, yaw: 3.14 } }
      - delay: { sec: 3.0 }
      - despawn: { obstacle: person }
```

## monitors (走行中の常時監視)

| 型 | パラメータ | 判定 |
|---|---|---|
| `bt_node` | `node`, `status`=RUNNING (`IDLE`/`RUNNING`/`SUCCESS`/`FAILURE`), `expect`=occurs (`occurs`/`never`), `topic`=/behavior_tree_log | BT のノードが指定の状態になった回数で、発生した／しないことを判定 (例: BackUp が動いたか) |
| `min_scan_range` | `min_range`, `topic`=/scan | LiDAR の最小測距値が `min_range` [m] 未満になったら FAILED (接触・急接近の近似検出)。データが来なければ ERROR |
| `obstacle_clearance` | `obstacles` (必須, scenario.obstacles のキー), `min_clearance`=0.0, `speed_threshold`=0.05, `angular_threshold`=0.1, `radius`=省略, `topic`=/odom | **ロボットが動いている間**の、フットプリントと障害物の表面の距離が `min_clearance` [m] 未満なら FAILED。`set_pose` で動かす障害物 (歩行者) は停止中のロボットを通り抜けるため、`min_scan_range` では当てたのがどちらか区別できない。停止中を含む全期間の最小距離は参考値としてメッセージに出る。プロファイルの `robot.footprint` が必要。形状は primitive (cylinder / sphere / box) から取り、fuel / local は `radius` の円として扱う。ロボット姿勢は TF の推定値のため数 cm の誤差がある。障害物を一度も観測しなければ ERROR |
| `no_diagnostic_errors` | `names`=[] (空なら全部), `topic`=/diagnostics | ERROR 以上の診断が出たら FAILED |
| `topic_received` | `topic`, `type` (`String` / `Bool` / `OccupancyGrid`), `data`="" (String の一致), `expect`=occurs, `min_count`=1 | メッセージを `min_count` 回以上受信した／しないことを判定。action の効果の確認 (publish した、地図を再読み込みした等) に使う |
| `max_speed` | `limit` [m/s], `topic`=/odom | オドメトリの並進速度が `limit` を超えたら FAILED |
| `max_stop_duration` | `max_stop_sec`=省略, `min_stop_sec`=0, `speed_threshold`=0.05, `from_goal_started`=0, `until_goal_reached`=0, `topic`=/odom | 区間 (指定ゴールの開始から指定ゴールの到達まで) の連続した停止時間を判定。`max_stop_sec` 超で FAILED (通過点で止まらないこと)、`min_stop_sec` 未満で FAILED (一時停止で止まること) |

ロボット固有の monitor はプラグインが提供します。

## expect (走行後の判定)

| 型 | パラメータ | 内容 |
|---|---|---|
| `reached_all` | なし | すべてのゴールに到達した (省略時の既定) |
| `time_limit` | `sec` | 走行が `sec` 秒 (sim 時間) 以内に終わった |
| `navigation_fails` | `index`=省略可 | 走行が失敗した (到達不能ゴールなどの異常系テスト)。`index` 指定時はそのゴールで失敗 |
| `final_pose_error` | `x`, `y`, `tolerance`=0.5, `frame`=map (`map`/`world`) | 走行後のロボット位置が指定点から `tolerance` [m] 以内 (停止精度) |

`expect:` を書いた場合は既定の `reached_all` は追加されません。必要なら明示してください。

## 変数展開

文字列中の `{world.<キー>}` はプロファイルの `worlds.<world>.<キー>` に、`pkg://<package>/<path>` は
パッケージの share ディレクトリ配下の絶対パスに展開されます。展開できるのは `waypoints_file`、
`local` モデルの `path` などパスを取る項目です。
