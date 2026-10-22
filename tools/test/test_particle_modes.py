import math

import numpy as np
import pytest

from tools.common.particle_modes import cluster_particles


def _blob(rng, center, n, sigma=(0.1, 0.1, 0.05)):
    return np.asarray(center) + rng.normal(0.0, sigma, size=(n, 3))


def test_two_yaw_modes_at_same_place_are_separated():
    rng = np.random.default_rng(0)
    # 同じ位置で向きが 90 度違う 2 つの群れ (角での割れ)
    poses = np.vstack([_blob(rng, [1.0, 2.0, 0.0], 300), _blob(rng, [1.0, 2.0, math.pi / 2], 100)])
    modes = cluster_particles(poses, np.ones(400), cell_m=0.5, cell_yaw_rad=math.radians(15))
    assert len(modes) == 2
    assert modes[0]["weight"] == pytest.approx(0.75)
    assert modes[0]["mean"][2] == pytest.approx(0.0, abs=0.02)
    assert modes[1]["mean"][2] == pytest.approx(math.pi / 2, abs=0.02)
    assert modes[0]["cov"][0, 0] == pytest.approx(0.01, rel=0.3)


def test_yaw_wraps_around_pi():
    rng = np.random.default_rng(1)
    poses = _blob(rng, [0.0, 0.0, math.pi], 200)
    poses[:, 2] = (poses[:, 2] + math.pi) % (2 * math.pi) - math.pi
    modes = cluster_particles(poses, np.ones(200), cell_m=0.5, cell_yaw_rad=math.radians(15))
    assert len(modes) == 1
    assert abs(modes[0]["mean"][2]) == pytest.approx(math.pi, abs=0.02)


def test_weights_decide_order_and_empty_input():
    rng = np.random.default_rng(2)
    poses = np.vstack([_blob(rng, [0.0, 0.0, 0.0], 100), _blob(rng, [5.0, 0.0, 0.0], 100)])
    weights = np.r_[np.full(100, 0.1), np.full(100, 0.9)]
    modes = cluster_particles(poses, weights, cell_m=0.5, cell_yaw_rad=math.radians(15))
    assert modes[0]["mean"][0] == pytest.approx(5.0, abs=0.05)
    assert modes[0]["weight"] == pytest.approx(0.9)
    assert cluster_particles(np.zeros((0, 3)), np.zeros(0), 0.5, 0.3) == []
