import glob
import os

import pytest
import yaml

from localization_supervisor.core.faults import parse_faults
from tools.common.faults import load_faults, parse_faults as parse_replay_faults
from tools.common.offline_faults import to_offline

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FAULT_DEFINITION_DIR = os.path.join(REPO_ROOT, "tools", "datasets", "localization", "faults")
SUPERVISION = os.path.join(REPO_ROOT, "mg_localization_supervisor", "config", "supervision.yaml")


def test_bundled_faults_convert_to_supervision_sources():
    with open(SUPERVISION, encoding="utf-8") as f:
        sources = yaml.safe_load(f)["sources"]
    for path in sorted(glob.glob(os.path.join(FAULT_DEFINITION_DIR, "*.yaml"))):
        replay = [f for f in load_faults(path) if f.type != "odom_scale"]
        for fault in parse_faults(to_offline(replay)):
            assert fault.source in sources, path


def test_conversion_keeps_times_and_values():
    replay = parse_replay_faults({"faults": [
        {"type": "kidnap", "at": 60.0, "offset": {"x": 3.0, "y": 0.0, "yaw": 0.5}},
        {"type": "amcl_drift", "start": 40.0, "end": 100.0, "rate": {"x": 0.05}},
        {"type": "gnss_bias", "start": 10.0, "end": 20.0, "bias": {"x": 2.0, "y": 1.0}},
    ]})
    bias, drift, kidnap = parse_faults(to_offline(replay, {"ekf": 0.5}))
    assert (kidnap.type, kidnap.source, kidnap.start) == ("pose_offset", "amcl", 60.0)
    assert kidnap.params["followers"] == {"ekf": 0.5}
    assert (drift.type, drift.end) == ("pose_drift", 100.0)
    assert (bias.type, bias.source, bias.end, bias.params["offset"]) == (
        "pose_offset", "gnss", 20.0, {"x": 2.0, "y": 1.0})


def test_odom_scale_cannot_be_injected_offline():
    replay = parse_replay_faults({"faults": [{"type": "odom_scale", "start": 1.0, "end": 2.0, "v": 1.0, "w": 1.2}]})
    with pytest.raises(ValueError):
        to_offline(replay)
