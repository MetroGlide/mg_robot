import json
import math

import numpy as np
import pytest
import yaml

from tools.common import pose_graph as pg


def _transform(easting, northing, zone=54):
    return {"anchor_utm": {"easting": easting, "northing": northing,
                           "zone": zone, "hemisphere": "north"}}


def test_shift_nodes_to_anchor_moves_by_utm_difference():
    nodes = np.array([[0.0, 1.0, 2.0, 0.3], [1.0, 2.0, 3.0, 0.4]])
    moved = pg.shift_nodes_to_anchor(nodes, _transform(110.0, 220.0), _transform(100.0, 200.0))
    # 元の座標 (アンカー B 基準) の (1, 2) は UTM で (111, 222)、アンカー A 基準では (11, 22)
    assert moved[0, 1] == pytest.approx(11.0)
    assert moved[0, 2] == pytest.approx(22.0)
    assert moved[:, 3].tolist() == [0.3, 0.4]
    # 元の配列は変更しない
    assert nodes[0, 1] == 1.0


def test_shift_nodes_rejects_different_zone():
    nodes = np.zeros((1, 4))
    with pytest.raises(ValueError):
        pg.shift_nodes_to_anchor(nodes, _transform(0, 0, zone=54), _transform(0, 0, zone=53))


def test_interpolate_nodes_linear_and_gap_invalid():
    nodes = np.array([
        [0.0, 0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0, 0.0],
        [10.0, 2.0, 0.0, 0.0],
    ])
    poses, valid = pg.interpolate_nodes(nodes, np.array([0.5, 5.0, -1.0, 11.0]))
    assert poses[0, 0] == pytest.approx(0.5)
    assert valid.tolist() == [True, False, False, False]


def test_interpolate_nodes_yaw_across_pi():
    nodes = np.array([[0.0, 0.0, 0.0, math.pi - 0.1], [1.0, 0.0, 0.0, -math.pi + 0.1]])
    poses, valid = pg.interpolate_nodes(nodes, np.array([0.5]))
    assert valid[0]
    assert abs(abs(poses[0, 2]) - math.pi) < 1e-6


def test_antenna_positions_rotate_lever_arm():
    poses = np.array([[1.0, 1.0, math.pi / 2]])
    ant = pg.antenna_positions(poses, (0.26, -0.13))
    assert ant[0, 0] == pytest.approx(1.0 + 0.13)
    assert ant[0, 1] == pytest.approx(1.0 + 0.26)


def test_fit_and_apply_rigid_roundtrip():
    rng = np.random.default_rng(1)
    src = rng.uniform(-5, 5, size=(30, 2))
    theta = 0.4
    t = np.array([1.5, -2.0])
    dst = pg.apply_rigid(src, t, theta)
    fit_t, fit_theta = pg.fit_rigid(src, dst)
    assert fit_theta == pytest.approx(theta, abs=1e-9)
    assert fit_t == pytest.approx(t, abs=1e-9)


def test_load_slam_output(tmp_path):
    graph = {
        "nodes": [
            {"timestamp": 2.0, "x": 2.0, "y": 0.0, "yaw": 0.0},
            {"timestamp": 1.0, "x": 1.0, "y": 0.0, "yaw": 0.0},
        ],
        "sequential_edges": [{"score": 0.9, "dyaw": 0.0}],
    }
    (tmp_path / "pose_graph.json").write_text(json.dumps(graph))
    (tmp_path / "gnss_transform.yaml").write_text(yaml.safe_dump(_transform(1.0, 2.0)))
    nodes, transform, matching = pg.load_slam_output(str(tmp_path))
    assert nodes[:, 0].tolist() == [1.0, 2.0]
    assert transform["anchor_utm"]["zone"] == 54
    assert matching["n"] == 1
