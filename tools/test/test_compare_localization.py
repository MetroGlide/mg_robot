import json

import pytest

from tools.scripts import compare_localization as cl
from tools.scripts.loc_dataset import load_dataset, resolve_dataset_path


def _write_run(base, name, run, report):
    d = base / name / f"run_{run}"
    d.mkdir(parents=True)
    (d / "eval_localization.json").write_text(json.dumps(report))


def _report(p95, nees=3.0):
    return {
        "accuracy": {"position_m": {"median": p95 / 2, "p95": p95, "max": p95 * 2},
                     "yaw_rad": {"p95": 0.1}},
        "smoothness": {"jump_position_m": {"p99": 0.05}, "jump_yaw_rad": {"p99": 0.1, "max": 0.5}},
        "amcl": {"latency_s": "n/a (別の時計)"},
        "nees": {"mean": nees, "fraction_within_chi2_95": 0.95},
    }


def test_get_path_handles_missing_and_non_numeric():
    data = {"a": {"b": 1.5, "c": "text", "d": True}}
    assert cl.get_path(data, ("a", "b")) == 1.5
    assert cl.get_path(data, ("a", "c")) is None
    assert cl.get_path(data, ("a", "d")) is None
    assert cl.get_path(data, ("x", "y")) is None


def test_aggregate_and_format_cell():
    assert cl.aggregate([]) is None
    assert cl.aggregate([1.0, 3.0]) == (2.0, 1.0, 3.0)
    assert cl.format_cell((2.0, 2.0, 2.0), 2) == "2.00"
    assert cl.format_cell((2.0, 1.0, 3.0), 1) == "2.0 (1.0–3.0)"
    assert cl.format_cell(None, 2) == "-"


def test_load_runs_and_render(tmp_path):
    _write_run(tmp_path, "baseline", 1, _report(0.4))
    _write_run(tmp_path, "baseline", 2, _report(0.6))
    _write_run(tmp_path, "new", 1, _report(0.2))
    runs = cl.load_runs(str(tmp_path / "baseline"))
    assert len(runs) == 2
    summary = cl.summarize_variant(runs)
    assert summary["位置誤差 p95[m]"] == pytest.approx((0.5, 0.4, 0.6))
    # 遅延が数値でない (別の時計) ときは集計しない
    assert summary["AMCL 遅延 中央値[s]"] is None
    text = cl.render([("baseline", 2, summary),
                      ("new", 1, cl.summarize_variant(cl.load_runs(str(tmp_path / "new"))))])
    assert "| baseline | 2 |" in text
    assert "| new | 1 |" in text
    assert "0.500 (0.400–0.600)" in text


def test_failure_rate_counts_runs_with_large_p95():
    runs = [_report(0.4), _report(9.0), _report(0.5), _report(4.0)]
    assert cl.failure_rate(runs) == pytest.approx(0.5)
    assert cl.failure_rate([]) is None
    assert cl.failure_rate([{"smoothness": {}}]) is None
    summary = cl.summarize_variant(runs)
    assert summary[cl.FAILURE_LABEL][0] == pytest.approx(0.5)
    assert "| 0.50 |" in cl.render([("v", 4, summary)])


def test_dataset_definitions_are_valid():
    for name in ("same_run_043837", "map043837_eval051635", "map043837_eval051635_short",
                 "map043837_eval051635_twist_glitch"):
        variables = load_dataset(resolve_dataset_path(name))
        assert variables["DATASET_NAME"] == name
        assert variables["MAP_YAML"].endswith("map.yaml")
    # 同じ走行では MAP_GT_DIR を使わない。別走行では地図側の SLAM 出力を指す
    assert load_dataset(resolve_dataset_path("same_run_043837"))["MAP_GT_DIR"] == ""
    assert load_dataset(resolve_dataset_path("map043837_eval051635"))["MAP_GT_DIR"] != ""
