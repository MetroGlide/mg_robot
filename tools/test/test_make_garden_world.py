import importlib.util
import os

import numpy as np
import yaml

SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts", "make_garden_world.py")
spec = importlib.util.spec_from_file_location("make_garden_world", SCRIPT)
mg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mg)


def test_distance_to_polyline():
    line = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]])
    d = mg.distance_to_polyline(np.array([[5.0, 2.0], [12.0, 5.0], [-3.0, 4.0]]), line)
    assert np.allclose(d, [2.0, 2.0, 5.0])


def _layout():
    return {
        "seed": 1, "resolution": 0.1, "scan_height_m": 0.5,
        "route": [[0.0, 0.0], [10.0, 0.0], [10.0, 6.0]],
        "hedge": {"inner_m": 1.5, "depth_m": 1.0, "density_per_m2": 5.0,
                  "size_m": [0.05, 0.2], "height_m": [0.2, 1.0]},
        "hedge_gaps": [{"at": [5.0, 2.0], "radius_m": 1.5}],
        "localization_variants": [{"suffix": "", "smear_m": 0.1}],
        "spherical_coordinates": {"latitude_deg": 0.0, "longitude_deg": 0.0},
    }


def test_hedge_elements_stay_in_band_and_out_of_gaps():
    layout = _layout()
    elements = mg.make_hedge_elements(layout, np.random.default_rng(0))
    xy = np.array([[e["x"], e["y"]] for e in elements])
    d = mg.distance_to_polyline(xy, np.array(layout["route"]))
    assert elements and np.all((d >= 1.5) & (d <= 2.5))
    assert np.all(np.hypot(xy[:, 0] - 5.0, xy[:, 1] - 2.0) > 1.5)


def test_outputs_are_generated(tmp_path):
    layout = _layout()
    rng = np.random.default_rng(0)
    elements = mg.make_hedge_elements(layout, rng)
    world = tmp_path / "garden.sdf"
    mg.write_world(str(world), "garden", layout, elements)
    mg.write_maps(str(tmp_path), layout, elements, rng)
    mg.write_waypoints(str(tmp_path / "waypoints.yaml"), np.array(layout["route"]))
    assert "<world name='garden'>" in world.read_text(encoding="utf-8")
    for name in ("planning", "localization"):
        meta = yaml.safe_load((tmp_path / f"{name}.yaml").read_text(encoding="utf-8"))
        assert (tmp_path / meta["image"]).exists()
    waypoints = yaml.safe_load((tmp_path / "waypoints.yaml").read_text(encoding="utf-8"))["waypoints"]
    assert len(waypoints) == 2 and waypoints[-1]["navigation"]["is_through_point"] is False
