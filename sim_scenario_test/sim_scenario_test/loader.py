from __future__ import annotations

import dataclasses
from typing import Any, Iterator, Tuple

import yaml

import sim_scenario_test.builtin  # noqa: F401  組み込み型を登録する
from sim_scenario_test.errors import ScenarioValidationError
from sim_scenario_test.plugins import load_plugins
from sim_scenario_test.profile import Profile, load_profile
from sim_scenario_test.registry import DEFAULT_REGISTRY, Registry
from sim_scenario_test.scenario import Scenario
from sim_scenario_test.spec import ActionCall, Call, ExpectationCall, parse_call, parse_spec


def read_yaml(path: str) -> Any:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def load_scenario(path: str, profile_override: str = "") -> Tuple[Scenario, Profile]:
    """シナリオを読み込む。プロファイル記載のプラグインを登録してから検証する。"""
    raw = read_yaml(path)
    if not isinstance(raw, dict):
        raise ScenarioValidationError(f"{path}: mapping expected at top level")
    profile_name = profile_override or raw.get("profile", "")
    if not profile_name:
        raise ScenarioValidationError(
            f"{path}: 'profile' is not set (set it in the scenario or pass a profile)")
    profile = load_profile(profile_name, DEFAULT_REGISTRY)
    load_plugins(profile.plugins)
    scenario = parse_scenario(raw, path, DEFAULT_REGISTRY)
    profile.world_vars(scenario.world, f"{path}.world")
    return scenario, profile


def parse_scenario(raw: Any, where: str, registry: Registry) -> Scenario:
    scenario = parse_spec(Scenario, raw, where, registry)
    if not scenario.expect:
        scenario.expect = [
            parse_call(ExpectationCall, "reached_all", f"{where}.expect", registry)]
    for call in iter_calls(scenario):
        obstacle = getattr(call.spec, "obstacle", None)
        if isinstance(call, ActionCall) and isinstance(obstacle, str) \
                and obstacle not in scenario.obstacles:
            raise ScenarioValidationError(
                f"{call.where}: obstacle '{obstacle}' is not defined in 'obstacles'")
    return scenario


def iter_calls(obj: Any) -> Iterator[Call]:
    """オブジェクト木に含まれる Call を再帰的に列挙する。"""
    if isinstance(obj, Call):
        yield obj
        yield from iter_calls(obj.spec)
    elif dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        for f in dataclasses.fields(obj):
            yield from iter_calls(getattr(obj, f.name))
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            yield from iter_calls(v)
    elif isinstance(obj, dict):
        for v in obj.values():
            yield from iter_calls(v)
