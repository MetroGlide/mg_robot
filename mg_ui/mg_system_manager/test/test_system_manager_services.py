import dataclasses

import pytest

from mg_system_manager.config import Settings
from mg_system_manager.docker_ops import ComposeRunner
from sm_fakes import FakeContainer, FakeDockerClient


def test_status_returns_service_to_state(client, containers):
    containers.append(FakeContainer("slam", "running"))
    containers.append(FakeContainer("navigation", "exited"))

    response = client.get("/status")

    assert response.status_code == 200
    assert response.json() == {"slam": "running", "navigation": "exited"}


def test_services_lists_registry(client):
    body = client.get("/services").json()

    keys = [s["key"] for s in body["services"]]
    assert "slam" in keys and "slam-gnss-2d" in keys
    hardware = {s["key"] for s in body["services"] if s["hardware"]}
    assert hardware == {"slam", "navigation", "slam-gnss-2d"}
    assert [layer["id"] for layer in body["layers"]] == [
        "core", "function", "tool"]


def test_start_runs_compose_up_in_host_project_dir(client, compose_calls):
    compose_calls.stdout = "started"

    body = client.post("/slam/start").json()

    assert body == {"success": True, "message": "started"}
    call = compose_calls.calls[0]
    assert call["command"] == ["docker", "compose", "up", "-d", "slam"]
    assert call["cwd"] == "/host/project"
    assert call["env"]["HOME"] == "/host/home"


def test_gpu_compose_file_is_added_like_makefile(
        settings, compose_calls, monkeypatch):
    monkeypatch.setenv("USE_GPU", "nvidia")
    gpu_settings = dataclasses.replace(
        settings, compose_files=Settings.from_env().compose_files)
    runner = ComposeRunner(gpu_settings, client=FakeDockerClient([]))

    runner.up("gazebo-simulation")

    assert compose_calls.calls[0]["command"] == [
        "docker", "compose", "-f", "compose.yaml",
        "-f", "compose.gpu.nvidia.yaml", "up", "-d", "gazebo-simulation"]


def test_start_failure_returns_stderr(client, compose_calls):
    compose_calls.returncode = 1
    compose_calls.stderr = "no such service"

    body = client.post("/navigation/start").json()

    assert body == {"success": False, "message": "no such service"}


def test_restart_runs_compose_restart(client, compose_calls):
    client.post("/foxglove-bridge/restart")

    assert compose_calls.calls[0]["command"] == [
        "docker", "compose", "restart", "foxglove-bridge"]


def test_stop_stops_container(client, containers):
    container = FakeContainer("slam", "running")
    containers.append(container)

    body = client.post("/slam/stop").json()

    assert body["success"] is True
    assert container.status == "exited"


def test_stop_reports_missing_container(client):
    body = client.post("/slam/stop").json()

    assert body == {"success": False, "message": "container not found: slam"}


def test_unknown_service_is_not_routable(client):
    assert client.post("/unknown/start").status_code == 404


@pytest.mark.parametrize("container, expected", [
    (FakeContainer("a", "running"), {"state": "running", "health": "", "exit_code": None, "level": "ok"}),
    (FakeContainer("a", "running", health="healthy"),
     {"state": "running", "health": "healthy", "exit_code": None, "level": "ok"}),
    (FakeContainer("a", "running", health="health: starting"),
     {"state": "running", "health": "starting", "exit_code": None, "level": "warn"}),
    (FakeContainer("a", "running", health="unhealthy"),
     {"state": "running", "health": "unhealthy", "exit_code": None, "level": "error"}),
    (FakeContainer("a", "restarting"), {"state": "restarting", "health": "", "exit_code": None, "level": "warn"}),
    (FakeContainer("a", "exited", exit_code=0), {"state": "exited", "health": "", "exit_code": 0, "level": "off"}),
    (FakeContainer("a", "exited", exit_code=137), {"state": "exited", "health": "", "exit_code": 137, "level": "off"}),
    (FakeContainer("a", "exited", exit_code=130), {"state": "exited", "health": "", "exit_code": 130, "level": "off"}),
    (FakeContainer("a", "exited", exit_code=1), {"state": "exited", "health": "", "exit_code": 1, "level": "error"}),
    (FakeContainer("a", "dead"), {"state": "dead", "health": "", "exit_code": None, "level": "error"}),
    (FakeContainer("a", "created"), {"state": "created", "health": "", "exit_code": None, "level": "off"}),
])
def test_status_detail_level(client, containers, container, expected):
    containers.append(container)

    assert client.get("/status/detail").json() == {"a": expected}


def test_status_detail_prefers_the_running_container(client, containers):
    containers.append(FakeContainer("slam", "exited", exit_code=1))
    containers.append(FakeContainer("slam", "running", oneoff=True))

    detail = client.get("/status/detail").json()["slam"]

    assert detail["state"] == "running" and detail["level"] == "ok"
    assert client.get("/status").json() == {"slam": "running"}
