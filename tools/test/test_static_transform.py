import math

import numpy as np
import pytest

from tools.common.geo import chain_static_transforms, quaternion_matrix


def _transform(x, y, z, yaw):
    t = np.eye(4)
    t[:3, :3] = quaternion_matrix(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    t[:3, 3] = (x, y, z)
    return t


def test_quaternion_matrix_yaw():
    r = quaternion_matrix(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    np.testing.assert_allclose(r @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def test_chain_composes_parent_to_child():
    static = {
        "base_link": ("base_footprint", _transform(0.0, 0.0, 0.15, 0.0)),
        "top_frame_link": ("base_link", _transform(0.23, 0.0, 0.45, 0.0)),
        "top_lrf_link": ("top_frame_link", _transform(0.0, 0.0, 0.03, math.pi)),
    }
    t = chain_static_transforms(static, "base_footprint", "top_lrf_link")
    np.testing.assert_allclose(t[:3, 3], [0.23, 0.0, 0.63], atol=1e-12)
    np.testing.assert_allclose(t[:3, :3] @ [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], atol=1e-12)


def test_chain_rotation_applies_to_child_offset():
    static = {
        "a": ("root", _transform(1.0, 0.0, 0.0, math.pi / 2)),
        "b": ("a", _transform(1.0, 0.0, 0.0, 0.0)),
    }
    t = chain_static_transforms(static, "root", "b")
    np.testing.assert_allclose(t[:3, 3], [1.0, 1.0, 0.0], atol=1e-12)


def test_chain_missing_link_exits():
    with pytest.raises(SystemExit):
        chain_static_transforms({"a": ("root", np.eye(4))}, "root", "c")
