"""走り方 (core/conduct.py: 状況 × 程度 → 減速) と、その設定、footprint の余裕のテスト。"""
import copy

import numpy as np
import pytest

from localization_supervisor.core.conduct import (
    Action, ConductConfig, ConductInputs, ConductPlanner, Degree, Situation)
from localization_supervisor.core.config import ConfigError, parse_config
from localization_supervisor.core.grid import OccupancyMap
from localization_supervisor.core.safety import FootprintChecker, SafetyConfig

from test_supervisor_gate import RAW, Sim

FOOTPRINT = [(0.4, 0.25), (0.4, -0.25), (-0.2, -0.3), (-0.2, 0.3)]


def _step(planner, t, situation, since=0.0, clearance=0.5, confirmed=False):
    return planner.update(ConductInputs(t, situation, since, clearance, confirmed))


def test_normal_never_slows_even_without_clearance():
    planner = ConductPlanner(ConductConfig())
    conduct = _step(planner, 0.0, Situation.NORMAL, clearance=None)
    assert conduct.degree == Degree.NONE and conduct.action == Action.NORMAL and conduct.speed_limit_mps == 0.0


def test_suspect_slows_only_when_the_error_is_large_for_the_clearance():
    cfg = ConductConfig()
    narrow = _step(ConductPlanner(cfg), 0.0, Situation.SUSPECT, clearance=0.4)
    assert narrow.degree == Degree.HEAVY and narrow.action == Action.SLOW
    assert narrow.speed_limit_mps == cfg.slow_speed_mps
    wide = _step(ConductPlanner(cfg), 0.0, Situation.SUSPECT, clearance=3.0)
    assert wide.degree == Degree.LIGHT and wide.action == Action.NORMAL


def test_unknown_clearance_is_treated_as_heavy():
    assert _step(ConductPlanner(ConductConfig()), 0.0, Situation.SUSPECT, clearance=None).degree == Degree.HEAVY


def test_error_grows_with_the_distance_since_confirmed():
    cfg = ConductConfig()
    conduct = _step(ConductPlanner(cfg), 0.0, Situation.UNOBSERVED, since=60.0, clearance=1.0)
    assert conduct.possible_error_m == pytest.approx(cfg.error_per_m * 60.0)


def test_suspect_returns_to_normal_only_when_confirmed():
    planner = ConductPlanner(ConductConfig())
    _step(planner, 0.0, Situation.SUSPECT, clearance=0.4)
    assert _step(planner, 1.0, Situation.NORMAL, clearance=0.4).situation == Situation.SUSPECT
    assert _step(planner, 2.0, Situation.NORMAL, clearance=0.4, confirmed=True).situation == Situation.NORMAL


def test_degree_rises_at_once_and_lowers_after_the_hold():
    cfg = ConductConfig(lower_hold_sec=3.0)
    planner = ConductPlanner(cfg)
    assert _step(planner, 0.0, Situation.UNDECIDED, clearance=4.0).degree == Degree.LIGHT
    assert _step(planner, 1.0, Situation.UNDECIDED, clearance=0.4).degree == Degree.HEAVY
    assert _step(planner, 2.0, Situation.UNDECIDED, clearance=4.0).degree == Degree.HEAVY
    assert _step(planner, 5.5, Situation.UNDECIDED, clearance=4.0).degree == Degree.LIGHT


def _raw_with_conduct(**conduct):
    raw = copy.deepcopy(RAW)
    raw['conduct'] = conduct
    return raw


def test_config_overlays_table_cells_and_floors_on_the_defaults():
    config = parse_config(_raw_with_conduct(table={'suspect': {'heavy': 'normal'}}, floors={'suspect': 0.5}))
    assert config.conduct.table['suspect'] == {'light': 'normal', 'medium': 'slow', 'heavy': 'normal'}
    assert config.conduct.table['undecided']['light'] == 'slow'
    assert config.conduct.floors['suspect'] == 0.5 and config.conduct.floors['undecided'] == 1.5
    profiled = parse_config(_raw_with_conduct(), profile='calm',
                            profiles={'calm': {'conduct': {'table': {'undecided': {'light': 'normal'}}}}})
    assert profiled.conduct.table['undecided']['light'] == 'normal'


@pytest.mark.parametrize('conduct', [
    {'table': {'normal': {'light': 'slow'}}},
    {'table': {'suspect': {'light': 'stop'}}},
    {'floors': {'suspect': -1.0}},
    {'medium_ratio': 2.0, 'heavy_ratio': 1.0},
    {'slow_speed_mps': 0.0},
])
def test_config_rejects_bad_conduct(conduct):
    with pytest.raises(ConfigError):
        parse_config(_raw_with_conduct(**conduct))


def test_clearance_is_the_distance_from_the_footprint_to_lethal_cells():
    data = np.zeros((200, 200), dtype=np.int8)
    data[:, 140:] = 100   # x 7 m より先が壁
    occupancy = OccupancyMap(data, 0.05, 0.0, 0.0)
    checker = FootprintChecker(SafetyConfig('ekf', 'planning', FOOTPRINT, depth_range_m=2.0))
    # 前端 (x + 0.4) から壁まで 1.0 m
    assert checker.clearance(occupancy, (5.6, 5.0, 0.0)) == pytest.approx(1.0, abs=0.06)
    # 遠ければ depth_range_m で頭打ち、かかっていれば 0
    assert checker.clearance(occupancy, (2.0, 5.0, 0.0)) == pytest.approx(2.0)
    assert checker.clearance(occupancy, (6.8, 5.0, 0.0)) == 0.0


def test_supervisor_slows_while_the_gate_is_closed_and_not_after_it_confirms():
    """門が閉じると疑い (余裕が分からないので重い) で減速し、照合で決まって確かめれば正常に戻る。"""
    sim = Sim(_raw_with_conduct())
    sim.run(18.0)
    assert sim.results[-1].conduct.situation == Situation.NORMAL
    assert all(r.conduct.speed_limit_mps == 0.0 for r in sim.results[-20:])
    sim.amcl_offset = (1.5, 0.0, 0.0)
    sim.ekf_offset = (0.3, 0.0, 0.0)
    results = sim.run(8.0)
    slowed = [r for r in results if r.conduct.action == Action.SLOW]
    assert slowed and slowed[0].conduct.situation in (Situation.SUSPECT, Situation.UNDECIDED)
    assert results[-1].conduct.situation == Situation.NORMAL and results[-1].conduct.speed_limit_mps == 0.0


def test_supervisor_without_recovery_does_not_slow():
    raw = _raw_with_conduct()
    raw['state_machine'] = dict(raw['state_machine'], recovery_enabled=False)
    sim = Sim(raw)
    sim.run(18.0)
    sim.amcl_offset = (1.5, 0.0, 0.0)
    results = sim.run(8.0)
    assert any(r.conduct.situation != Situation.NORMAL for r in results)
    assert all(r.conduct.speed_limit_mps == 0.0 for r in results)
