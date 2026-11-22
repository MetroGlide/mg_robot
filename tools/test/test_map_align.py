"""tools/common/map_align.py のテスト。"""

import json
import math

import numpy as np
import pytest
import yaml
from PIL import Image

from tools.common.map_align import (
    apply_offsets,
    assign_group,
    fit_offsets,
    inverse_pose,
    load_wptroj_groups,
    match_cloud,
    select_map,
    transform_points,
    voxel_downsample,
)
from tools.common.scan_map import OccupancyMap

RES = 0.05


def _write_map(tmp_path, occupied: np.ndarray, origin=(0.0, 0.0), name="m"):
    """occupied (行 0 が y 最小) を pgm と map.yaml に書く。"""
    pixels = np.where(occupied, 0, 254).astype(np.uint8)[::-1]
    Image.fromarray(pixels).save(tmp_path / f"{name}.pgm")
    path = tmp_path / f"{name}.yaml"
    with open(path, "w") as f:
        yaml.safe_dump({"image": f"{name}.pgm", "resolution": RES, "origin": [origin[0], origin[1], 0.0],
                        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}, f)
    return str(path)


def _wall_points(length_m: float):
    """向きの違う壁と柱からなる、照合が決まる形 (地図の座標 [m])。"""
    pts = []
    s = np.arange(0.0, length_m, RES)
    pts.append(np.column_stack([s + 2.0, np.full_like(s, 2.0)]))
    pts.append(np.column_stack([np.full_like(s, 2.0), s + 2.0]))
    d = np.arange(0.0, 8.0, RES)
    pts.append(np.column_stack([10.0 + d * 0.8, 8.0 + d * 0.6]))
    for cx, cy in ((8.0, 12.0), (14.0, 5.0), (6.0, 7.0)):
        a = np.linspace(0, 2 * np.pi, 60)
        pts.append(np.column_stack([cx + 0.3 * np.cos(a), cy + 0.3 * np.sin(a)]))
    return np.concatenate(pts)


@pytest.fixture
def grid(tmp_path):
    size = int(25.0 / RES)
    occ = np.zeros((size, size), dtype=bool)
    pts = _wall_points(18.0)
    occ[(pts[:, 1] / RES).astype(int), (pts[:, 0] / RES).astype(int)] = True
    return OccupancyMap(_write_map(tmp_path, occ))


def test_transform_and_inverse_roundtrip():
    pose = np.array([1.0, -2.0, 0.7])
    pts = np.array([[1.0, 0.0], [0.0, 2.0]])
    back = transform_points(inverse_pose(pose), transform_points(pose, pts))
    np.testing.assert_allclose(back, pts, atol=1e-12)
    np.testing.assert_allclose(transform_points(pose, np.array([[1.0, 0.0]]))[0],
                               [1.0 + math.cos(0.7), -2.0 + math.sin(0.7)])


def test_match_cloud_recovers_offset(grid):
    true_pose = np.array([9.0, 9.0, math.radians(20.0)])
    world = _wall_points(18.0)
    cloud = transform_points(inverse_pose(true_pose), world)
    cloud = voxel_downsample(cloud, 0.1, 2000)
    init = true_pose + np.array([-1.2, 0.8, math.radians(-1.5)])
    result = match_cloud(grid, cloud, init, search_xy=2.0)
    assert result.pose[0] == pytest.approx(true_pose[0], abs=0.05)
    assert result.pose[1] == pytest.approx(true_pose[1], abs=0.05)
    assert result.pose[2] == pytest.approx(true_pose[2], abs=math.radians(0.2))
    assert result.inlier_ratio > 0.9
    assert result.ambiguity < 0.8


def test_match_cloud_flags_corridor_as_degenerate(tmp_path):
    size = int(25.0 / RES)
    occ = np.zeros((size, size), dtype=bool)
    occ[int(10.0 / RES), :] = True
    occ[int(14.0 / RES), :] = True
    corridor = OccupancyMap(_write_map(tmp_path, occ, name="corridor"))
    s = np.arange(2.0, 23.0, RES)
    world = np.concatenate([np.column_stack([s, np.full_like(s, 10.0)]), np.column_stack([s, np.full_like(s, 14.0)])])
    pose = np.array([12.0, 12.0, 0.0])
    cloud = voxel_downsample(transform_points(inverse_pose(pose), world), 0.1, 2000)
    result = match_cloud(corridor, cloud, pose + np.array([0.0, 0.3, 0.0]), search_xy=2.0)
    assert result.pose[1] == pytest.approx(12.0, abs=0.05)
    eig = np.linalg.eigvalsh(result.cov[:2, :2])
    assert math.sqrt(eig[1] / eig[0]) > 4.0


def test_select_map_prefers_larger_margin(tmp_path):
    occ = np.zeros((200, 200), dtype=bool)
    a = OccupancyMap(_write_map(tmp_path, occ, origin=(0.0, 0.0), name="a"))
    b = OccupancyMap(_write_map(tmp_path, occ, origin=(5.0, 0.0), name="b"))
    assert select_map([a, b], np.array([9.0, 5.0])) == 1
    assert select_map([a, b], np.array([1.0, 5.0])) == 0
    assert select_map([a, b], np.array([50.0, 5.0])) is None


def test_fit_offsets_rigid_and_translation_only():
    rng = np.random.default_rng(1)
    ref = rng.uniform(-50, 50, size=(30, 2))
    t, theta = np.array([1.5, -0.7]), math.radians(0.4)
    mat = apply_offsets(ref, t, theta)
    ft, fth = fit_offsets(ref, mat)
    np.testing.assert_allclose(ft, t, atol=1e-9)
    assert fth == pytest.approx(theta, abs=1e-12)
    small = ref[:5] * 0.05
    ft2, fth2 = fit_offsets(small, small + [0.3, 0.2])
    np.testing.assert_allclose(ft2, [0.3, 0.2])
    assert fth2 == 0.0


def test_wptroj_groups_and_assignment(tmp_path):
    project = {
        "layer_order": ["top", "bottom", "hidden"],
        "map_layers": [
            {"id": "top", "visible": True, "clip": {"rects": [{"x": 0, "y": 0, "width": 10, "height": 10}]}},
            {"id": "bottom", "visible": True, "clip": {"rects": [{"x": 5, "y": 0, "width": 10, "height": 10}]}},
            {"id": "hidden", "visible": False, "clip": {"rects": [{"x": 0, "y": 0, "width": 100, "height": 100}]}},
        ],
        "layer_groups": {
            "g1": {"name": "upper", "children_ids": ["top"], "visible": True,
                   "pose_offset": {"x": 1.0, "y": -1.0, "yaw": 0.01}},
            "g2": {"name": "lower", "children_ids": ["bottom"], "visible": True},
            "g3": {"name": "off", "children_ids": ["hidden"], "visible": False},
        },
        "export_regions": [{"name": "R1", "rect": {"x": 0, "y": 0, "width": 20, "height": 20}}],
    }
    path = tmp_path / "p.wptroj"
    path.write_text(json.dumps(project))
    groups, regions = load_wptroj_groups(str(path))
    assert {g.name for g in groups} == {"upper", "lower"}
    assert next(g for g in groups if g.name == "upper").offset == (1.0, -1.0, 0.01)
    assert regions == [("R1", (0.0, 0.0, 20.0, 20.0))]
    names = assign_group(groups, np.array([[2.0, 2.0], [7.0, 2.0], [12.0, 2.0], [30.0, 2.0]]))
    assert names == ["upper", "upper", "lower", None]
