"""dataclass ベースの厳格パーサの単体テスト。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional

import pytest

from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.geometry import PoseSpec
from sim_scenario_test.registry import Registry
from sim_scenario_test.spec import ActionCall, parse_call, parse_spec


@dataclass
class Inner:
    value: int


@dataclass
class Sample:
    name: str
    ratio: float = 1.0
    mode: Literal["a", "b"] = "a"
    limit: Optional[int] = None
    items: List[Inner] = field(default_factory=list)
    table: Dict[int, Inner] = field(default_factory=dict)
    enabled: bool = False


REG = Registry()


def _parse(raw):
    return parse_spec(Sample, raw, "sample", REG)


def test_defaults_and_nesting():
    s = _parse({"name": "x", "ratio": 2, "items": [{"value": 1}], "table": {3: {"value": 4}}})
    assert s.ratio == 2.0 and isinstance(s.ratio, float)
    assert s.items[0].value == 1
    assert s.table[3].value == 4
    assert s.limit is None


@pytest.mark.parametrize("raw, message", [
    ({}, "'name' is required"),
    ({"name": "x", "nmae": 1}, "unknown key"),
    ({"name": "x", "mode": "c"}, "must be one of"),
    ({"name": "x", "limit": True}, "int expected"),
    ({"name": "x", "ratio": "fast"}, "number expected"),
    ({"name": "x", "enabled": 1}, "bool expected"),
    ({"name": "x", "items": {"value": 1}}, "list expected"),
    ({"name": "x", "items": [{"value": 1.5}]}, r"items\[0\]\.value"),
])
def test_errors(raw, message):
    with pytest.raises(ScenarioValidationError, match=message):
        _parse(raw)


def test_pose_frame_literal():
    with pytest.raises(ScenarioValidationError, match="frame"):
        parse_spec(PoseSpec, {"frame": "odom"}, "pose", REG)
    assert parse_spec(PoseSpec, {"x": 1}, "pose", REG).frame == "map"


def test_parse_call_forms():
    reg = Registry()
    reg.register("action", "noop", None, lambda *a: None)
    reg.register("action", "wait", Inner, lambda *a: None)
    assert parse_call(ActionCall, "noop", "a", reg).spec is None
    assert parse_call(ActionCall, {"wait": {"value": 3}}, "a", reg).spec.value == 3
    with pytest.raises(ScenarioValidationError, match="available"):
        parse_call(ActionCall, {"jump": {}}, "a", reg)
    with pytest.raises(ScenarioValidationError, match="takes no parameters"):
        parse_call(ActionCall, {"noop": {"x": 1}}, "a", reg)
    with pytest.raises(ScenarioValidationError, match="expected"):
        parse_call(ActionCall, {"noop": None, "wait": None}, "a", reg)
