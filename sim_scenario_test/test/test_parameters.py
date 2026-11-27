"""record_parameters (実行時のパラメータの記録) のテスト。"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from rcl_interfaces.msg import ParameterType

from fakes import make_context
from sim_scenario_test.engine.runner import ScenarioEngine
from sim_scenario_test.loader import parse_scenario
from sim_scenario_test.parameters import UNAVAILABLE, parameter_value, read_parameters
from sim_scenario_test.registry import DEFAULT_REGISTRY


class _Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


def _node(responses):
    """サービス名 -> 応答 (None ならサービスなし) を返すフェイクのノード。"""
    node = MagicMock()

    def create_client(_srv_type, service):
        client = MagicMock()
        client.srv_name = service
        response = responses.get(service)
        client.wait_for_service.return_value = response is not None
        client.call_async.side_effect = lambda _req: _Future(response)
        return client

    node.create_client.side_effect = create_client
    return node


def _response(*values):
    return SimpleNamespace(values=list(values))


def _value(type_, **fields):
    return SimpleNamespace(type=type_, **fields)


def test_parameter_value_types():
    assert parameter_value(_value(ParameterType.PARAMETER_DOUBLE, double_value=0.2)) == 0.2
    assert parameter_value(_value(ParameterType.PARAMETER_STRING, string_value="a")) == "a"
    assert parameter_value(
        _value(ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=(1.0, 2.0))) == [1.0, 2.0]
    assert parameter_value(_value(ParameterType.PARAMETER_NOT_SET)) is None


def test_read_parameters_and_unavailable_node():
    node = _node({"/amcl/get_parameters": _response(
        _value(ParameterType.PARAMETER_DOUBLE, double_value=0.1),
        _value(ParameterType.PARAMETER_NOT_SET))})
    values = read_parameters(node, {"amcl": ["alpha1", "missing"], "ekf": ["x"]}, 0.1)
    assert values["amcl"] == {"alpha1": 0.1, "missing": None}
    assert values["ekf"]["status"] == UNAVAILABLE


def test_engine_records_parameters_without_changing_status():
    scenario = parse_scenario({
        "version": "2.0", "name": "params_test", "world": "w", "run": {"fake_goals": {}},
    }, "scenario", DEFAULT_REGISTRY)
    ctx = make_context(scenario)
    ctx.profile.record_parameters = {"amcl": ["alpha1"]}
    ctx.node = _node({})
    result = ScenarioEngine(ctx).execute()
    assert result.success, result.checks
    assert result.to_dict()["parameters"]["amcl"]["status"] == UNAVAILABLE
