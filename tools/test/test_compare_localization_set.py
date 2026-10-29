from tools.scripts import compare_localization_set as cs


def _run(p95, deviated):
    return {
        "accuracy": {"position_m": {"p95": p95}, "yaw_rad": {"p95": 0.1}},
        "turns": {"n_judged": 1, "n_deviated": int(deviated), "n_mispicked": 0},
    }


def test_read_set_skips_comments_and_keeps_order(tmp_path):
    path = tmp_path / "s.txt"
    path.write_text("# 説明\n\nb ds_b --runs 2\na ds_a\n")
    assert cs.read_set(str(path)) == [("b", "ds_b"), ("a", "ds_a")]


def test_render_pools_rates_over_conditions():
    entries = [("x", "dx"), ("y", "dy")]
    results = {
        ("x", "base"): [_run(1.0, True), _run(1.0, False)],
        ("y", "base"): [_run(5.0, True), _run(1.0, True)],
        ("x", "new"): [_run(1.0, False)],
        ("y", "new"): [],
    }
    text = cs.render(entries, ["base", "new"], results)
    deviated = text.split(f"## {cs.DEVIATED_LABEL}")[1].split("##")[0]
    # base は 4 試行中 3 試行がずれた。new は試行のある条件だけで数える
    assert "| **全条件** | 0.75 | 0.00 |" in deviated
    assert "| y | 1.00 | - |" in deviated
    failure = text.split(f"## {cs.FAILURE_LABEL}")[1].split("##")[0]
    assert "| **全条件** | 0.25 | 0.00 |" in failure


def test_host_path_maps_container_data_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    assert cs.host_path("/root/ros2_data/a/b") == str(tmp_path / "ros2_data" / "a" / "b")
    assert cs.host_path(str(tmp_path)) == str(tmp_path)
