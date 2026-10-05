import math

import numpy as np
import pytest
import yaml

from tools.common.map import load_gnss_transform, read_map_list


def test_read_map_list_skips_comments_blank_and_missing(tmp_path):
    (tmp_path / "a.yaml").write_text("", encoding="utf-8")
    (tmp_path / "sub").mkdir()
    (tmp_path / "sub" / "b.yaml").write_text("", encoding="utf-8")
    list_file = tmp_path / "map_list.txt"
    list_file.write_text(
        "a.yaml\n\n# comment\n  sub/b.yaml  \nmissing.yaml\n", encoding="utf-8")

    assert read_map_list(str(list_file)) == [
        str(tmp_path / "a.yaml"), str(tmp_path / "sub" / "b.yaml")]


@pytest.mark.parametrize("rotation", [None, 0.0, 0.35, -1.2])
def test_load_gnss_transform_matches_bridge(tmp_path, rotation):
    data = {"anchor_utm": {"easting": 380000.0, "northing": 3950000.0,
                           "zone": 54, "hemisphere": "north"},
            "rotation_rad": 2.5}
    if rotation is not None:
        data["map_rotation_rad"] = rotation
    path = tmp_path / "gnss_transform.yaml"
    path.write_text(yaml.dump(data), encoding="utf-8")

    (tx, ty, yaw), zone = load_gnss_transform(str(path))

    assert zone == 54
    assert yaw == pytest.approx(rotation or 0.0)

    # slam_gnss_nav_bridge: map = R(map_rotation_rad) * (utm - anchor_utm)
    utm = np.array([380012.3, 3949987.6])
    local = utm - np.array([380000.0, 3950000.0])
    c, s = math.cos(yaw), math.sin(yaw)
    expected = np.array([c * local[0] - s * local[1], s * local[0] + c * local[1]])
    actual = np.array([tx + c * utm[0] - s * utm[1], ty + s * utm[0] + c * utm[1]])
    assert actual == pytest.approx(expected)
