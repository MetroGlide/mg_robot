"""GazeboFortressBackend のリクエスト生成とエラー処理の単体テスト (ign CLI はモック)。"""
from __future__ import annotations

import subprocess
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import Pose
from sim_scenario_test.sim.gazebo_fortress import GazeboFortressBackend
from sim_scenario_test.sim.model import ModelSpec


@pytest.fixture
def ign(monkeypatch):
    calls = []
    reply = {"returncode": 0, "stdout": "data: true", "stderr": ""}

    def fake_run(cmd, **kwargs):
        calls.append(cmd)
        return SimpleNamespace(**reply)

    monkeypatch.setattr(subprocess, "run", fake_run)
    return SimpleNamespace(calls=calls, reply=reply)


def _backend():
    return GazeboFortressBackend(MagicMock(), "warehouse")


def _req(cmd):
    return cmd[cmd.index("--req") + 1]


def test_fuel_model_is_included_as_static(ign):
    _backend().spawn_entity(
        "person", ModelSpec(type="fuel", uri="OpenRobotics/models/Male Visitor"), Pose(1.0, 2.0))
    cmd = ign.calls[0]
    assert cmd[cmd.index("-s") + 1] == "/world/warehouse/create"
    req = _req(cmd)
    assert "Male%20Visitor" in req
    assert "<static>true</static>" in req
    assert '\\"1.6\\"' in req
    assert "position { x: 1.0 y: 2.0" in req


def test_primitive_dynamic(ign):
    model = ModelSpec(type="primitive", shape="cylinder", size={"radius": 0.3}, static=False)
    _backend().spawn_entity("pole", model, Pose())
    req = _req(ign.calls[0])
    assert "<radius>0.3</radius>" in req and "<static>false</static>" in req


def test_local_sdf_escaping(ign, tmp_path):
    sdf = tmp_path / "m.sdf"
    sdf.write_text('<sdf version="1.6">\n<model name="m"><!-- a\\b --></model></sdf>')
    _backend().spawn_entity("m", ModelSpec(type="local", path=str(sdf)), Pose())
    req = _req(ign.calls[0])
    assert "\n" not in req
    assert 'a\\\\b' in req


def test_service_false_raises(ign):
    ign.reply["stdout"] = "data: false"
    with pytest.raises(ScenarioError, match="returned false"):
        _backend().remove_entity("x")


def test_service_failure_raises(ign):
    ign.reply.update(returncode=1, stderr="boom")
    with pytest.raises(ScenarioError, match="boom"):
        _backend().set_entity_pose("mg", Pose())


def test_is_ready_checks_world_service(ign):
    ign.reply["stdout"] = "/world/other/create\n/world/warehouse/create\n"
    assert _backend().is_ready()
    ign.reply["stdout"] = "/world/other/create\n"
    assert not _backend().is_ready()


def test_missing_local_file_raises(ign):
    with pytest.raises(ScenarioError, match="failed to read SDF"):
        _backend().spawn_entity("m", ModelSpec(type="local", path="/nonexistent.sdf"), Pose())
