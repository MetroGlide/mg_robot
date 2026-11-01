from tools.scripts import compare_localization_set as cs


def _run(yaw_p95, deviated, sensitive=False):
    return {
        "accuracy": {"position_m": {"p95": 1.0}, "yaw_rad": {"p95": yaw_p95}},
        "deviations": {"n": int(deviated), "until_end": False, "total_sec": 5.0 * deviated,
                       "sensitivity": [{"yaw_deg": 20.0, "position_m": 3.0, "n": int(deviated or sensitive)},
                                       {"yaw_deg": 45.0, "position_m": 6.0, "n": 0}]},
    }


def test_read_set_skips_comments_and_keeps_order(tmp_path):
    path = tmp_path / "s.txt"
    path.write_text("# 説明\n\nb ds_b --runs 2\na ds_a\n")
    assert cs.read_set(str(path)) == [("b", "ds_b"), ("a", "ds_a")]


def test_render_pools_rates_over_conditions():
    entries = [("x", "dx"), ("y", "dy")]
    results = {
        ("x", "base"): [_run(0.1, True), _run(0.1, False, sensitive=True)],
        ("y", "base"): [_run(0.1, True), _run(0.1, True)],
        ("x", "new"): [_run(0.1, False)],
        ("y", "new"): [],
    }
    text = cs.render(entries, ["base", "new"], results)
    deviated = text.split(f"## {cs.DEVIATED_LABEL}")[1].split("##")[0]
    # base は 4 試行中 3 試行がずれた。new は試行のある条件だけで数える
    assert "| **全条件** | 0.75 | 0.00 |" in deviated
    assert "| y | 1.00 | - |" in deviated
    sensitivity = text.split("## 閾値を変えたとき")[1].split("##")[0]
    assert "| 20° / 3 m | 1.00 | 0.00 |" in sensitivity
    assert "| 45° / 6 m | 0.00 | 0.00 |" in sensitivity
    assert "許容差を超えて悪い条件" not in text


def test_render_marks_cells_worse_than_baseline_beyond_tolerance():
    entries = [("x", "dx"), ("y", "dy")]
    results = {
        ("x", "base"): [_run(0.30, False)],
        ("y", "base"): [_run(0.10, False)],
        # x: 0.30 + max(0.05, 0.2 × 0.30) = 0.36 まで許す。y: 0.10 + 0.05 = 0.15 を超え、ずれも増えた
        ("x", "new"): [_run(0.35, False)],
        ("y", "new"): [_run(0.16, True)],
    }
    text = cs.render(entries, ["base", "new"], results, baseline="base")
    yaw = text.split("## yaw 誤差 p95[rad]")[1].split("##")[0]
    assert "| x | 0.300 | 0.350 |" in yaw
    assert "| y | 0.100 | 0.160 ⚠ |" in yaw
    listing = text.split("許容差を超えて悪い条件")[1]
    assert "- new: y の yaw 誤差 p95[rad]" in listing
    assert f"- new: y の {cs.DEVIATED_LABEL}" in listing
    assert "- new: x" not in listing


def test_host_path_maps_container_data_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    assert cs.host_path("/root/ros2_data/a/b") == str(tmp_path / "ros2_data" / "a" / "b")
    assert cs.host_path(str(tmp_path)) == str(tmp_path)
