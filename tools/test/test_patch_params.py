from tools.scripts.patch_params import patch_lines

MULTI_NODE = """\
amcl:
  ros__parameters:
    use_sim_time: False
    max_particles: 5000   # 上限
    robot_model:
      alpha1: 0.2

controller_server:
  ros__parameters:
    use_sim_time: False
    max_particles: 1
"""


def _patch(text, updates):
    lines, unapplied = patch_lines(text.splitlines(keepends=True), updates)
    return "".join(lines), unapplied


def test_patches_first_match_without_node():
    out, unapplied = _patch(MULTI_NODE, {"use_sim_time": "true"})
    assert unapplied == []
    assert out.count("use_sim_time: true") == 1
    # 最初のノード (amcl) だけが書き換わる
    assert out.index("use_sim_time: true") < out.index("controller_server")


def test_node_qualified_spec_targets_that_node():
    out, unapplied = _patch(MULTI_NODE, {"controller_server:max_particles": "2"})
    assert unapplied == []
    assert "max_particles: 5000   # 上限" in out
    assert "max_particles: 2\n" in out


def test_keeps_comment_and_coerces_float():
    out, _ = _patch(MULTI_NODE, {"amcl:robot_model.alpha1": "1"})
    assert "alpha1: 1.0\n" in out
    out, _ = _patch(MULTI_NODE, {"amcl:max_particles": "2000"})
    assert "max_particles: 2000   # 上限" in out


def test_reports_unapplied_keys():
    _, unapplied = _patch(MULTI_NODE, {"amcl:missing": "1", "nope:use_sim_time": "true"})
    assert unapplied == ["amcl:missing", "nope:use_sim_time"]
