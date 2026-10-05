"""変数展開の単体テスト。"""
from __future__ import annotations

import pytest

from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.template import expand


def test_expand_variables():
    variables = {"world": {"map": "/data/map.yaml", "name": "w"}}
    assert expand("{world.map}", variables, "t") == "/data/map.yaml"
    assert expand("x_{world.name}_y", variables, "t") == "x_w_y"
    assert expand("plain", variables, "t") == "plain"


def test_undefined_variable():
    with pytest.raises(ScenarioValidationError, match="undefined variable"):
        expand("{world.sdf}", {"world": {}}, "t")
