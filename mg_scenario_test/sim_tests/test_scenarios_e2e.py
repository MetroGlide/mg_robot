"""シミュレータを起動する結合テスト (pytest からシナリオを実行する例)。

通常の `make test` では実行されない。実行するには scenario-test コンテナ内で次のようにする
(ROS 環境を source すると launch_testing の pytest プラグインが pytest 9 と衝突するため、
プラグインの自動読み込みを止める):

  docker compose run --rm -e SCENARIO_SIM=1 -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 scenario-test \
    bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && \
             cd /app && python3 -m pytest mg_scenario_test/sim_tests -v"

シナリオごとに新しいシミュレータ・ナビゲーションを起動する (1 本あたり 1 分前後〜)。
"""
from __future__ import annotations

import os
import pathlib

import pytest

from sim_scenario_test.engine.result import ResultStatus
from sim_scenario_test.execution import collect_scenarios, run_scenario, run_scenario_dict

pytestmark = pytest.mark.skipif(
    not os.environ.get("SCENARIO_SIM"),
    reason="set SCENARIO_SIM=1 to run tests that start the simulator")

_SCENARIOS = pathlib.Path(__file__).resolve().parents[1] / "scenarios"


_SMOKE = collect_scenarios([str(_SCENARIOS / "regression")], ["smoke"], ["known_issue"])


@pytest.mark.parametrize("path", _SMOKE, ids=lambda p: os.path.basename(p))
def test_smoke_scenarios(path, tmp_path):
    record = run_scenario(path, str(tmp_path))
    assert record.status == ResultStatus.PASSED, record.message


@pytest.mark.parametrize("goal", [(5.0, 2.0), (10.0, 6.0)])
def test_python_defined_scenario(goal, tmp_path):
    """dict でシナリオを組み立てる例。YAML では書きにくいパラメータ展開を Python で行う。"""
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
