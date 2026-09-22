import struct

import numpy as np

from mg_utils.point_cloud import build_point_array

MIN_DEPTH = 0.1
MAX_DEPTH = 10.0


def make_inputs(h=48, w=64, seed=0):
    rng = np.random.default_rng(seed)
    depth_m = rng.uniform(0.0, 12.0, size=(h, w)).astype(np.float32)
    depth_m[0, 0] = np.nan
    depth_m[1, 1] = np.inf
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    u_norm = (u - w / 2) / 50.0
    v_norm = (v - h / 2) / 50.0
    bgr = rng.integers(0, 256, size=(h, w, 3), dtype=np.uint8)
    return depth_m, u_norm, v_norm, bgr


def reference_bytes(depth_m, u_norm, v_norm, bgr=None):
    """従来実装（点ごとに struct でパック）と同じ内容のバイト列。"""
    out = bytearray()
    h, w = depth_m.shape
    for i in range(h):
        for j in range(w):
            z = depth_m[i, j]
            if not (z > MIN_DEPTH and z < MAX_DEPTH and np.isfinite(z)):
                continue
            out += struct.pack('<fff', u_norm[i, j] * z, v_norm[i, j] * z, z)
            if bgr is not None:
                b, g, r = (int(c) for c in bgr[i, j])
                out += struct.pack('<I', (r << 16) | (g << 8) | b)
    return bytes(out)


def test_xyz_matches_reference():
    depth_m, u_norm, v_norm, _ = make_inputs()
    points = build_point_array(depth_m, u_norm, v_norm, MIN_DEPTH, MAX_DEPTH)
    assert points.shape[1] == 3
    assert points.tobytes() == reference_bytes(depth_m, u_norm, v_norm)


def test_xyzrgb_matches_reference():
    depth_m, u_norm, v_norm, bgr = make_inputs(seed=1)
    points = build_point_array(depth_m, u_norm, v_norm, MIN_DEPTH, MAX_DEPTH, bgr)
    assert points.shape[1] == 4
    assert points.tobytes() == reference_bytes(depth_m, u_norm, v_norm, bgr)


def test_no_valid_pixels_returns_none():
    depth_m, u_norm, v_norm, bgr = make_inputs()
    depth_m[:] = 0.0
    assert build_point_array(depth_m, u_norm, v_norm, MIN_DEPTH, MAX_DEPTH) is None
    assert build_point_array(depth_m, u_norm, v_norm, MIN_DEPTH, MAX_DEPTH, bgr) is None


def test_invalid_depth_is_excluded():
    depth_m, u_norm, v_norm, _ = make_inputs()
    points = build_point_array(depth_m, u_norm, v_norm, MIN_DEPTH, MAX_DEPTH)
    assert np.all(np.isfinite(points))
    assert np.all((points[:, 2] > MIN_DEPTH) & (points[:, 2] < MAX_DEPTH))
