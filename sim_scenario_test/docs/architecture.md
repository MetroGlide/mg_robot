# アーキテクチャ

## 層構成

| 層 | 場所 | 役割 |
|---|---|---|
| CLI / 実行 | `cli.py`, `execution.py`, `launch/` | シナリオごとに `ros2 launch` でスタックを起動し直して実行。結果の集約と JUnit |
| ノード | `node.py` | シナリオ 1 本を実行して終了する ROS ノード (`scenario_runner.py`)。結果 JSON を書く |
| エンジン | `engine/` | 実行順序 (`runner.py`)、readiness (`readiness.py`)、タイムライン (`timeline.py`)、結果 (`result.py`) |
| 型の登録簿 | `registry.py`, `spec.py` | action / trigger / expectation / monitor / driver / backend / waypoint 形式を登録し、dataclass に基づいて YAML を厳格に検証 |
| 組み込みの型 | `builtin/`, `drivers/nav2_goals.py` | 標準の action・trigger・expectation・monitor、Nav2 ゴール走行ドライバ |
| ロボット注入 | `profile.py`, `plugins.py` | ロボットプロファイルとプラグインの読み込み |
| シミュレータ | `sim/` | `SimulationBackend` (抽象) と `gazebo_fortress` |

## 実行の流れ (`ScenarioEngine.execute`)

```
readiness ─ setup ─ ┬ driver.run (走行。ゴール到達などを ctx.events に記録)
                    ├ timeline  (trigger が成立したら action を並行実行)
                    └ monitors  (走行中ずっと購読して判定材料を集める)
        ─ teardown ─ 生成した障害物の自動削除 ─ expectation の評価 ─ 結果
```

1. **readiness**: sim 時計・シミュレータのワールド・TF `map→base`・lifecycle manager・指定サービス・ドライバのサーバが
   すべて利用可能になるまで待つ (`profile.readiness.timeout_sec` まで)。ワールド名の不一致はここで分かる。
2. **setup**: `setup:` の action を順に実行する (通常 `respawn`)。
3. **run**: 走行ドライバが走行する。同時にタイムラインと monitor が動く。
4. **teardown**: `teardown:` の action を実行し、シナリオが生成した障害物を残らず削除する。
5. **判定**: `expect:` (省略時は `reached_all`) を評価する。走行できなかった (ERROR) 場合は評価しない。

### タイムライン

各項目は `when` (trigger) が成立した時点で `do` (action 列) を別スレッドで実行する。`until` が成立すると
実行中の `do` を打ち切る。`required: true` (既定) の項目が一度も発火しなかった場合は ERROR とする
(障害物が出ないまま PASSED になる偽陽性を防ぐ)。

### 時間

`delay` や各種タイムアウトは **sim 時間** で測る。シミュレータが遅くても (RTF < 1) タイミングが崩れない。
sim 時計が `sim.clock_stall_sec` (壁時計) 以上進まない場合は、シミュレータ停止として ERROR にする。

## 結果 (PASSED / FAILED / ERROR)

チェックは `execution` (セットアップ・基盤)、タイムライン項目、`monitor:*`、`expect:*`、`teardown`。
1 つでも ERROR があれば全体が ERROR、なければ FAILED があれば FAILED、すべて PASSED なら PASSED。

| 状態 | 意味の例 |
|---|---|
| FAILED | ゴールに到達できない、期待した挙動が起きない (BT ノードが動かない等)、monitor が異常を検出 |
| ERROR | スタックが起動しない、readiness のタイムアウト、spawn したのにエンティティが存在しない、action が失敗、タイムラインが一度も発火しない、sim 時計の停止 |

## 座標系

`PoseSpec.frame`: `map` (既定・Nav2 の地図座標)、`world` (Gazebo のワールド座標)、`robot` (現在のロボット姿勢基準)。
地図原点とワールド原点の関係はプロファイルの `frames.map_in_world` で与える。`robot` は TF `map→base` から解決する。

## 拡張ポイント

| 種類 | 登録デコレータ | 実装するもの |
|---|---|---|
| action | `register_action(name, Spec)` | `fn(ctx, spec, stop_event)`。失敗は `ScenarioError` を送出 |
| trigger | `register_trigger(name, Spec)` | `cls(ctx, spec)` と `poll() -> bool` |
| expectation | `register_expectation(name, Spec)` | `fn(ctx, spec, outcome) -> CheckResult` |
| monitor | `register_monitor(name, Spec)` | `cls(ctx, spec)` と `start()` / `stop()` / `result()` |
| driver | `register_driver(name, Spec)` | `RunDriver` のサブクラス |
| backend | `register_backend(name)` | `SimulationBackend` のサブクラス |
| waypoint 形式 | `register_waypoint_format(name)` | `fn(path) -> List[IndexedPose]` |

作り方は [plugin_development.md](plugin_development.md) を参照。

## 既知の制約

- Gazebo Fortress の `create` / `remove` サービスは要求の受理で成功を返す。生成の確認は `spawn` action が `ign model --list` で行う。
- Humble + Fortress では Python の gz-transport が使えないため、`ign service` CLI を使う。障害物の移動は 5 Hz 程度が上限。
- シナリオごとにスタックを起動し直すので 1 本あたり 30〜60 秒のオーバーヘッドがある。
- 同一マシン・同一 ROS ドメインで別のスタックを起動していると干渉する。
