import glob
import os

import pytest

from tools.common.faults import load_faults, parse_faults

FAULT_DEFINITION_DIR = os.path.join(
    os.path.dirname(__file__), "..", "datasets", "localization", "faults")


def test_bundled_fault_definitions_are_valid():
    paths = sorted(glob.glob(os.path.join(FAULT_DEFINITION_DIR, "*.yaml")))
    assert paths
    for path in paths:
        assert load_faults(path), path


def test_parse_sorts_and_normalizes():
    faults = parse_faults({"faults": [
        {"type": "gnss_drop", "start": 300, "end": 360},
        {"type": "kidnap", "at": 120, "offset": {"x": 3.0, "y": 0.0, "yaw": 0.5}},
    ]})
    assert [f.type for f in faults] == ["kidnap", "gnss_drop"]
    assert faults[0].is_instant and faults[0].start == faults[0].end == 120.0
    assert faults[0].params["offset"]["x"] == 3.0
    assert not faults[1].is_instant


def test_parse_empty():
    assert parse_faults({}) == []


@pytest.mark.parametrize("raw", [
    {"type": "unknown", "start": 0, "end": 1},
    {"type": "kidnap", "at": 1},
    {"type": "kidnap", "offset": {}},
    {"type": "gnss_drop", "start": 5},
    {"type": "gnss_drop", "start": 5, "end": 5},
    {"type": "odom_scale", "start": 1, "end": 2, "v": 1.0},
])
def test_parse_rejects_invalid(raw):
    with pytest.raises(ValueError):
        parse_faults({"faults": [raw]})
