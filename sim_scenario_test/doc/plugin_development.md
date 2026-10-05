# プラグインの作り方

ロボット固有の走行ドライバ・action・trigger・monitor などを、コアを変更せずに追加する方法です。
実例は `mg_scenario_test/mg_scenario_test/plugins.py` (waypoint_sequencer 用) を参照してください。

## 仕組み

1. プラグインは Python モジュールで、import 時にデコレータで型を **登録簿 (registry)** に登録します。
2. プロファイルの `plugins:` にモジュール名を列挙すると、シナリオの読み込み前に一度だけ import されます
   (`plugins.load_plugins`。import に失敗した場合は握りつぶさずそのまま例外になります)。
3. 登録した型は、コアの型と同じ書式・同じ厳格な検証でシナリオから使えます。

```yaml
# profiles/myrobot.yaml
plugins: [my_pkg.scenario_plugins]
```

## 型ごとの実装

パラメータ (Spec) は **dataclass** で定義します。型注釈に従って YAML が検証され、未知のキーや型の不一致はエラーになります。
使える型: `str` / `int` / `float` / `bool` / `Optional[...]` / `Literal[...]` / `List[...]` / `Dict[...]` /
他の dataclass / `PoseSpec` / `ActionCall` などの呼び出し型。`validate(self, where)` メソッドを定義すると追加検証ができ、
不正なら `ScenarioValidationError` を送出します。

### action

```python
from dataclasses import dataclass
from sim_scenario_test.registry import register_action

@dataclass
class HonkSpec:
    times: int = 1

@register_action("honk", HonkSpec)
def honk(ctx, spec, stop_event):
    """クラクションを鳴らす (1 行目が list-types に表示される)。"""
    for _ in range(spec.times):
        ...  # ctx.node を使って publish / service 呼び出し
```

失敗は `ScenarioError` を送出します (シナリオは ERROR になります)。長い処理は `stop_event` を確認して中断できるようにします。

### trigger

```python
@register_trigger("battery_below", BatterySpec)
class BatteryBelow:
    """バッテリー残量が閾値を下回ったら成立する。"""
    def __init__(self, ctx, spec): ...
    def poll(self) -> bool: ...     # 約 20 Hz で呼ばれる。成立したら True
```

購読はコンストラクタで作り、コールバックが更新した値を `poll` で見るのが定石です。

### monitor

```python
from sim_scenario_test.builtin.monitors import TopicMonitor, occurrence_result

@register_monitor("my_topic_event", MySpec)
class MyMonitor(TopicMonitor):          # 単純なトピック監視なら基底クラスが購読を管理する
    MSG_TYPE = MyMsg
    def __init__(self, ctx, spec):
        super().__init__(ctx, spec.topic)
    def _on_msg(self, msg): ...
    def _evaluate(self):
        return occurrence_result(self._count, "occurs", "my event")
```

`start()` / `stop()` / `result() -> CheckResult` を持つクラスなら何でも monitor になります。
`CheckResult` の `name` はエンジンが設定します。`ScenarioResult` の集計では ERROR > FAILED > PASSED の順です。

### expectation

```python
@register_expectation("arrived_within", Spec)
def arrived_within(ctx, spec, outcome):
    ...
    return CheckResult("", ResultStatus.PASSED, "message")
```

`outcome` は走行ドライバの結果 (`total`, `reached`, `failed_index`, `failure`)。走行中のイベントは
`ctx.events.find("goal_reached", index=0)` などで参照できます。

### 走行ドライバ

`RunDriver` を継承して `run() -> Outcome` を実装し、`register_driver` で登録します。

```python
@register_driver("my_runner", MySpec)
class MyRunner(RunDriver):
    def wait_ready(self, timeout_sec): ...     # 必要なサーバが使えるまで待つ (任意)
    def run(self):
        ctx = self.ctx
        ...
        ctx.events.emit("goal_started", index=i)   # trigger (goal_started 等) の元になる
        ...
        ctx.events.emit("goal_reached", index=i)
        return Outcome(total, reached, failed_index, failure)   # 失敗なら failure に理由
    def close(self): ...
```

守ること:

- `ctx.abort_event` がセットされたら (タイムアウト・タイムライン失敗・sim 時計停止) 速やかに走行を止めて戻る。
- ロボットが期待どおり動かなかった場合は `Outcome.failure` に理由を入れて返す (→ FAILED)。
  セットアップ・基盤の問題は `ScenarioError` を送出する (→ ERROR)。
- `goal_started` / `goal_reached` / `goal_failed` を `ctx.events` に記録する。
- 時間は `ctx.clock.now()` / `ctx.clock.sleep()` (sim 時間) で測る。
- `hooks` のように action を持つ Spec は `List[ActionCall]` を使い、`ctx.run_actions(calls, stop_event)` で実行する。

### waypoint 形式

```python
@register_waypoint_format("myformat")
def load(path) -> List[IndexedPose]:
    """独自形式の waypoint ファイル。"""
```

`nav2_goals.waypoints_format: myformat` で使えます。

### シミュレータバックエンド

`SimulationBackend` を継承し (`is_ready` / `entity_exists` / `set_entity_pose` / `spawn_entity` / `remove_entity`)、
`register_backend("name")` で登録します。姿勢はすべてシミュレータのワールド座標で渡されます。

## `ScenarioContext` (ctx) で使えるもの

| 属性・メソッド | 内容 |
|---|---|
| `ctx.node` | ROS ノード (別スレッドで executor が spin している。購読・サービスクライアントを作れる) |
| `ctx.logger` | ロガー |
| `ctx.clock` | `now()` / `sleep(sec, stop_event)` (sim 時間) |
| `ctx.poses` | `to_map(spec)` / `to_world(spec)` / `robot.get()` (現在のロボット姿勢, map 座標) |
| `ctx.backend` | シミュレータ操作 |
| `ctx.nav2` | `publish_initial_pose` / `clear_costmaps` / `lifecycle_active` |
| `ctx.events` | `emit(name, **attrs)` / `find(name, **attrs)` |
| `ctx.profile`, `ctx.scenario`, `ctx.world_vars` | プロファイル・シナリオ・選択したワールドの変数 |
| `ctx.expand(text, where)` | `{変数}` と `pkg://` を展開 |
| `ctx.run_actions(calls, stop_event)` | action 列を実行 |
| `ctx.abort_event` / `ctx.abort(reason, error=False)` | 走行の中断 |
| `ctx.extensions` | プラグインが状態 (クライアント等) を共有するための辞書 |
| `ctx.rng` | seed で再現できる乱数 |

## テスト

ROS もシミュレータもなしでテストできるように、`ScenarioContext` は依存 (`node`, `backend`, `poses`, `nav2`) を
差し替えられます。`sim_scenario_test/test/fakes.py` に `FakeBackend` / `FakeRobot` / `FakeClock` /
`make_context` があり、`test_monitors.py` や `mg_scenario_test/test/test_sequencer_driver.py` が使用例です。
ROS のモジュールは `test/conftest.py` で `sys.modules` をモックしています。
