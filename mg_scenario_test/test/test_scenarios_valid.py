"""同梱シナリオ・プロファイルが MG プラグイン込みで検証を通ることを確認する。"""
from __future__ import annotations

import pathlib

import pytest
import yaml

import mg_scenario_test.plugins  # noqa: F401  MG 固有の型を登録する
from sim_scenario_test.loader import parse_scenario
from sim_scenario_test.profile import Profile
from sim_scenario_test.registry import DEFAULT_REGISTRY
from sim_scenario_test.spec import parse_spec

_PKG = pathlib.Path(__file__).resolve().parents[1]


def _profile() -> Profile:
    raw = yaml.safe_load((_PKG / "profiles" / "mg01.yaml").read_text())
    return parse_spec(Profile, raw, "mg01", DEFAULT_REGISTRY)


def test_profile_is_valid():
    profile = _profile()
    assert profile.plugins == ["mg_scenario_test.plugins"]
    assert profile.world_vars("warehouse", "t")["sim_world"] == "warehouse"


def _scenario_files():
    """scenarios/ 配下 (regression/, examples/) のシナリオ。data/ (補助データ) は除く。"""
    return sorted(
        p for p in (_PKG / "scenarios").rglob("*.yaml") if "data" not in p.relative_to(_PKG).parts)


@pytest.mark.parametrize("path", _scenario_files(), ids=lambda p: p.name)
def test_bundled_scenario_is_valid(path):
    raw = yaml.safe_load(path.read_text())
    scenario = parse_scenario(raw, str(path), DEFAULT_REGISTRY)
    assert scenario.profile == "mg01"
    _profile().world_vars(scenario.world, str(path))


def test_mg_types_are_registered():
    assert "mg_sequencer" in DEFAULT_REGISTRY.names("driver")
    assert {"sequencer_start", "sequencer_set_index", "sequencer_stop"} <= set(
        DEFAULT_REGISTRY.names("action"))
    assert "sequencer_state" in DEFAULT_REGISTRY.names("trigger")
    assert "mg" in DEFAULT_REGISTRY.names("waypoint_format")


def test_scenario_names_are_unique_and_match_file_names():
    names = [yaml.safe_load(p.read_text())["name"] for p in _scenario_files()]
    assert len(names) == len(set(names))


def test_regression_scenarios_have_a_tier_and_category_tag():
    for path in _scenario_files():
        if "regression" not in path.parts:
            continue
        tags = yaml.safe_load(path.read_text()).get("tags", [])
        assert len(tags) >= 2, f"{path.name}: add a category tag besides the tier tag"
