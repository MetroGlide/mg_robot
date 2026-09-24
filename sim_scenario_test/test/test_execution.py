"""シナリオ一括実行・結果集約・JUnit 出力の単体テスト (ros2 launch はフェイクに置換)。"""
from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET

import pytest

from sim_scenario_test.engine.result import ResultStatus
from sim_scenario_test.execution import (
    RunRecord,
    collect_scenarios,
    exit_code,
    run_scenario,
    run_scenario_dict,
    write_junit,
)

FAKE_LAUNCH = """
import json, sys, time
args = dict(a.split(":=", 1) for a in sys.argv if ":=" in a)
mode = args["scenario_file"]
print("fake launch", sys.argv[1:])
if mode == "hang":
    time.sleep(60)
elif mode != "crash":
    status = mode
    checks = [] if status == "PASSED" else [
        {"name": "expect:reached_all", "status": status, "message": "boom"}]
    json.dump({"scenario_name": "fake", "status": status, "checks": checks},
              open(args["result_file"], "w"))
"""


@pytest.fixture
def fake_launch(tmp_path):
    script = tmp_path / "fake_launch.py"
    script.write_text(FAKE_LAUNCH)
    return [sys.executable, str(script)]


def _run(fake_launch, tmp_path, mode, **kwargs):
    return run_scenario(mode, str(tmp_path / "out"), launch_prefix=fake_launch, **kwargs)


@pytest.mark.parametrize("mode, status", [
    ("PASSED", ResultStatus.PASSED),
    ("FAILED", ResultStatus.FAILED),
    ("ERROR", ResultStatus.ERROR),
])
def test_status_from_result_file(fake_launch, tmp_path, mode, status):
    rec = _run(fake_launch, tmp_path, mode)
    assert rec.status == status
    assert (tmp_path / "out" / "launch.log").read_text().startswith("fake launch")
    if status != ResultStatus.PASSED:
        assert "boom" in rec.message


def test_missing_result_is_error(fake_launch, tmp_path):
    rec = _run(fake_launch, tmp_path, "crash")
    assert rec.status == ResultStatus.ERROR and "no result" in rec.message


def test_timeout_is_error(fake_launch, tmp_path):
    rec = _run(fake_launch, tmp_path, "hang", timeout_sec=1.0)
    assert rec.status == ResultStatus.ERROR and "timed out" in rec.message


def test_stale_result_is_not_reused(fake_launch, tmp_path):
    out = tmp_path / "out"
    out.mkdir()
    (out / "result.json").write_text(json.dumps(
        {"scenario_name": "old", "status": "PASSED", "checks": []}))
    assert _run(fake_launch, tmp_path, "crash").status == ResultStatus.ERROR


def test_launch_arguments(fake_launch, tmp_path):
    _run(fake_launch, tmp_path, "PASSED", gui=True, profile="p")
    log = (tmp_path / "out" / "launch.log").read_text()
    assert "headless:=false" in log and "profile:=p" in log
    _run(fake_launch, tmp_path, "PASSED", attach=True)
    log = (tmp_path / "out" / "launch.log").read_text()
    assert "scenario.launch.py" in log and "headless" not in log


def _rec(status):
    return RunRecord("s", status, "msg", 1.0, "", [])


def test_exit_code_priority():
    P, F, E = ResultStatus.PASSED, ResultStatus.FAILED, ResultStatus.ERROR
    assert exit_code([_rec(P), _rec(P)]) == 0
    assert exit_code([_rec(P), _rec(F)]) == 1
    assert exit_code([_rec(F), _rec(E)]) == 2
    assert exit_code([]) == 2


def test_junit(tmp_path):
    path = tmp_path / "junit.xml"
    write_junit([_rec(ResultStatus.PASSED), _rec(ResultStatus.FAILED),
                 _rec(ResultStatus.ERROR)], str(path))
    suite = ET.parse(path).getroot()
    assert (suite.get("tests"), suite.get("failures"), suite.get("errors")) == ("3", "1", "1")
    assert suite[1].find("failure") is not None and suite[2].find("error") is not None


def test_collect_scenarios_by_tags(tmp_path):
    (tmp_path / "a.yaml").write_text("tags: [smoke, x]\n")
    (tmp_path / "b.yaml").write_text("tags: [other]\n")
    (tmp_path / "c.txt").write_text("")
    assert len(collect_scenarios([str(tmp_path)], [])) == 2
    assert collect_scenarios([str(tmp_path)], ["smoke"]) == [str(tmp_path / "a.yaml")]
    assert collect_scenarios([str(tmp_path / "b.yaml")], []) == [str(tmp_path / "b.yaml")]


def test_run_scenario_dict_writes_yaml(fake_launch, tmp_path):
    import yaml
    out = tmp_path / "dict_out"
    raw = {"version": "2.0", "name": "d", "tags": ["x"]}
    # フェイクは scenario_file の値でモードを決めるため、書き出したパスを見て PASSED 扱いにする
    fake = tmp_path / "fake2.py"
    fake.write_text(FAKE_LAUNCH.replace('mode = args["scenario_file"]', 'mode = "PASSED"'))
    rec = run_scenario_dict(raw, str(out), launch_prefix=[sys.executable, str(fake)])
    assert rec.status == ResultStatus.PASSED
    assert yaml.safe_load((out / "scenario.yaml").read_text()) == raw
