# Python / pytest でシナリオを書く

YAML では書きにくい、パラメータの組み合わせ展開や、結果に対する独自の検査は Python で書けます。

## dict で組み立てて実行する

`run_scenario_dict` は、YAML と同じ構造の dict を書き出してシナリオを 1 本実行し (シミュレータとスタックも起動)、
`RunRecord` を返します。

```python
import pytest
from sim_scenario_test.engine.result import ResultStatus
from sim_scenario_test.execution import run_scenario_dict

@pytest.mark.parametrize("goal", [(5.0, 2.0), (10.0, 6.0)])
def test_reaches_goal(goal, tmp_path):
    raw = {
        "version": "2.0",
        "name": f"goal_{goal[0]}_{goal[1]}",
        "profile": "mg01",
        "world": "warehouse",
        "setup": [{"respawn": {"pose": {"x": 0.0, "y": 0.0}}}],
        "run": {"nav2_goals": {"goals": [{"pose": {"x": goal[0], "y": goal[1]}}]}},
        "expect": ["reached_all", {"time_limit": {"sec": 90}}],
    }
    record = run_scenario_dict(raw, str(tmp_path))
    assert record.status == ResultStatus.PASSED, record.message
```

`RunRecord` の主な属性: `status` / `message` (失敗したチェックの要約) / `elapsed_sec` /
`result_dir` (`result.json` と `launch.log` の場所) / `checks`。

既存の YAML を実行するなら `run_scenario(path, out_dir)`、タグで集めるなら `collect_scenarios(paths, tags)`。

## 実行方法

シミュレータを起動するので、通常の単体テストとは分けて実行します。`mg_scenario_test/sim_tests/` が実例です。

```bash
docker compose run --rm -e SCENARIO_SIM=1 -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 scenario-test \
  bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && \
           cd /app && python3 -m pytest mg_scenario_test/sim_tests -v"
```

- ROS 環境を source すると `launch_testing` の pytest プラグインが pytest 9 と衝突するため、
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` でプラグインの自動読み込みを止めます。
- 環境変数 `SCENARIO_SIM` を見てスキップする (`pytest.mark.skipif`) ようにしておくと、`make test` に混ざっても実行されません。
- 1 本ごとに新しくスタックを起動するので、1 本あたり 1 分前後以上かかります。

## エンジンを直接使う

型の追加なしに動作を確かめたいだけなら、`FakeBackend` などを使ってエンジンをシミュレータなしで動かせます
(`sim_scenario_test/test/test_engine.py` が実例)。独自の action や driver を書くときは
[plugin_development.md](plugin_development.md) を参照してください。
