import numpy as np
import yaml
from PIL import Image

from tools.common.scan_map import (
    OccupancyMap,
    beam_geometry,
    keep_matched,
    raycast_ranges,
    valid_ranges,
)


def _grid(tmp_path):
    """解像度 0.1 m、原点 (0, 0)、10 m 四方。x = 5.0〜5.1 m に南北の壁を置く。"""
    pixels = np.full((100, 100), 254, dtype=np.uint8)
    pixels[:, 50] = 0
    Image.fromarray(pixels).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({
        "image": "map.pgm", "resolution": 0.1, "origin": [0.0, 0.0, 0.0],
        "occupied_thresh": 0.65, "free_thresh": 0.25, "negate": 0}), encoding="utf-8")
    return OccupancyMap(str(tmp_path / "map.yaml"))


def test_raycast_hits_wall_and_misses_away(tmp_path):
    grid = _grid(tmp_path)
    hit = grid.raycast(np.array([2.0, 5.0]), np.array([0.0, np.pi, np.pi / 4]), 0.05, 30.0)
    assert abs(hit[0] - 3.0) < 0.1
    assert np.isinf(hit[1])
    assert abs(hit[2] - 3.0 * np.sqrt(2.0)) < 0.15


def test_clear_near_removes_occupied_around_path(tmp_path):
    grid = _grid(tmp_path)
    grid.clear_near(np.array([[5.05, 5.0]]), 1.0)
    # 消した所 (壁の y = 4〜6) は光線が抜け、消していない所 (y = 2) には当たる
    assert np.isinf(grid.raycast(np.array([2.0, 5.0]), np.array([0.0]), 0.05, 30.0)[0])
    assert abs(grid.raycast(np.array([2.0, 2.0]), np.array([0.0]), 0.05, 30.0)[0] - 3.0) < 0.1


def test_raycast_ranges_keep_invalid_beams(tmp_path):
    grid = _grid(tmp_path)
    ranges = np.array([1.0, np.inf, 1.0])
    yaws = np.array([0.0, 0.0, np.pi])
    result = raycast_ranges(ranges, np.array([2.0, 5.0]), yaws, grid, 0.05, 30.0, 0.0, np.random.default_rng(0))
    assert abs(result[0] - 3.0) < 0.1
    assert np.isinf(result[1]) and np.isinf(result[2])


def test_keep_matched_drops_points_off_the_map(tmp_path):
    grid = _grid(tmp_path)
    # 1 本目は壁に当たる、2 本目は何もない所 (地図にない物) に当たる
    ranges = np.array([3.0, 1.0])
    result = keep_matched(ranges, np.array([2.0, 5.0]), np.array([0.0, 0.0]), grid, 0.2)
    assert result[0] == 3.0 and np.isinf(result[1])


def test_valid_ranges_and_beam_geometry():
    result = valid_ranges([0.01, 1.0, float("nan"), 50.0], 0.05, 30.0)
    assert np.isinf(result[[0, 2, 3]]).all() and result[1] == 1.0
    xy, yaw = beam_geometry(np.array([1.0, 2.0, np.pi / 2]), np.array([0.5, 0.0, np.pi]), np.array([0.0]))
    assert np.allclose(xy, [1.0, 2.5]) and np.isclose(np.cos(yaw[0]), 0.0) and np.isclose(np.sin(yaw[0]), -1.0)
