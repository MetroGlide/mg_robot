import pytest

from tools.scripts import run_localization_queue as q


def test_parse_config_splits_name_and_args():
    assert q.parse_config("a2_sup=--monitor supervisor amcl.alpha2=0.1") == (
        "a2_sup", ["--monitor", "supervisor", "amcl.alpha2=0.1"])


@pytest.mark.parametrize("text", ["--monitor none", "=--monitor none", "x=--runs 3", "x=--run-from 2"])
def test_parse_config_rejects_missing_name_and_run_options(text):
    with pytest.raises(SystemExit):
        q.parse_config(text)


def test_read_conditions_keeps_extra_args_and_filters_labels(tmp_path):
    path = tmp_path / "s.txt"
    path.write_text("# 説明\n\nr4 map20261017r4_eval043837\n"
                    "kidnap map043837_eval051635_short --faults tools/datasets/localization/faults/kidnap.yaml\n")
    conditions = q.read_conditions(str(path), None)
    assert [c.label for c in conditions] == ["r4", "kidnap"]
    assert conditions[1].extra == ["--faults", "tools/datasets/localization/faults/kidnap.yaml"]
    assert conditions[0].duration > 0.0
    assert conditions[0].out_dir.endswith("/eval_loc/map20261017r4_eval043837")
    assert [c.label for c in q.read_conditions(str(path), {"kidnap"})] == ["kidnap"]


def _run(n, until_end=False):
    return {"deviations": {"n": n, "until_end": until_end}}


def test_outcomes_split_only_when_runs_disagree():
    assert not q.outcomes_split([_run(0), _run(0), _run(0)])
    assert not q.outcomes_split([_run(1, True), _run(2, True)])
    assert q.outcomes_split([_run(0), _run(1)])
    # どちらもずれたが、戻ったかどうかが割れている
    assert q.outcomes_split([_run(1, True), _run(1, False)])


def test_deadline_grows_with_duration():
    assert q.deadline_sec(600.0) > q.deadline_sec(200.0) > 200.0
