from dataclasses import replace
from pathlib import Path

import pytest
from mg_system_manager.routers.maps import list_navigation_maps
from sm_fakes import FakeContainer


@pytest.mark.parametrize("path,body,message", [
    ("/map/common/save", {"map_dir": "/tmp/a b"}, "invalid map_dir"),
    ("/map/common/save", {"map_name": "../x"}, "invalid map_name"),
    ("/slam_gnss_2d/map/save", {"output_dir": "/tmp/$(id)"},
     "invalid output_dir"),
    ("/slam_gnss_2d/preview/start", {"slam_map_path": "/a;b"},
     "invalid slam_map_path"),
    ("/slam_gnss_2d/reoptimize/start", {"input_dir": "/a", "bag_path": "x y"},
     "invalid bag_path"),
    ("/rosbag-replay/start", {"file": "/bag; rm -rf /"}, "invalid file path"),
    ("/rosbag-replay/start", {"file": "/bag", "topics": ["/a b"]},
     "invalid topic: /a b"),
])
def test_invalid_input_is_rejected_without_running_commands(
        client, compose_calls, path, body, message):
    response = client.post(path, json=body).json()

    assert response == {"success": False, "message": message}
    assert compose_calls.calls == []


def test_scenario_stack_rejects_unlisted_package(client, compose_calls):
    body = client.post("/scenario-stack/start", json={
        "package": "evil", "file": "a.launch.py"}).json()

    assert body["message"] == "package not allowed: evil"
    assert compose_calls.calls == []


def test_scenario_stack_rejects_path_traversal(client):
    body = client.post("/scenario-stack/start", json={
        "package": "mg_bringup", "file": "../x.launch.py"}).json()

    assert body["message"] == "invalid file"


def test_scenario_stack_rejects_hardware_running(
        client, containers, compose_calls):
    containers.append(FakeContainer("navigation", "running"))

    body = client.post("/scenario-stack/start", json={
        "package": "mg_bringup", "file": "sim.launch.py"}).json()

    assert body["success"] is False
    assert "hardware services are running" in body["message"]
    assert compose_calls.calls == []


def test_scenario_stack_start_passes_stack_env(client, compose_calls):
    body = client.post("/scenario-stack/start", json={
        "package": "mg_bringup",
        "file": "sim.launch.py",
        "args": {"world": "warehouse"},
    }).json()

    assert body["success"] is True
    call = compose_calls.calls[0]
    assert call["command"] == [
        "docker", "compose", "up", "-d", "--force-recreate",
        "scenario-remote-stack"]
    assert call["env"]["STACK_PACKAGE"] == "mg_bringup"
    assert call["env"]["STACK_ARGS"] == "world:=warehouse"
    assert call["timeout"] == 120


def test_scenario_stack_stop_removes_container(client, containers):
    container = FakeContainer("scenario-remote-stack", "running")
    containers.append(container)

    body = client.post("/scenario-stack/stop").json()

    assert body["success"] is True
    assert container.stopped_with == [30]
    assert container.removed is True


def test_scenario_stack_stop_without_container_is_ok(client):
    body = client.post("/scenario-stack/stop").json()

    assert body == {"success": True, "message": "not running"}


def test_scenario_stack_logs(client, containers):
    containers.append(FakeContainer("scenario-remote-stack", "running"))

    assert client.get("/scenario-stack/logs").json() == {
        "logs": "line1\nline2\n"}


def test_rosbag_start_passes_env(client, compose_calls):
    body = client.post("/rosbag-replay/start", json={
        "file": "/data/bag", "topics": ["/scan", "/tf"]}).json()

    assert body["success"] is True
    env = compose_calls.calls[0]["env"]
    assert env["ROSBAG_FILE"] == "/data/bag"
    assert env["ROSBAG_TOPICS"] == "/scan /tf"


def test_rosbag_env_reads_dotenv(client, settings):
    project = Path(settings.project_dir)
    project.mkdir(parents=True)
    (project / ".env").write_text(
        "# comment\nBASE=/data\nROSBAG_FILE=${BASE}/bag\n"
        "ROSBAG_TOPICS=/scan /tf\n", encoding="utf-8")

    assert client.get("/rosbag-replay/env").json() == {
        "file": "/data/bag", "topics": ["/scan", "/tf"]}


def test_list_slam_gnss_2d_maps(client, tmp_path):
    base = tmp_path / "maps"
    (base / "20260101_000000").mkdir(parents=True)
    (base / "file.txt").write_text("x")

    body = client.get(
        "/slam_gnss_2d/maps", params={"base_dir": str(base)}).json()

    assert body == {"success": True, "maps": ["20260101_000000"]}


def _write_map_list(tmp_path, text):
    (tmp_path / "map_list.txt").write_text(text, encoding="utf-8")


def test_list_navigation_maps(tmp_path):
    _write_map_list(
        tmp_path,
        "# 測位用\nlocalization_1.yaml\n\n  planning_1.yaml  \nmissing.yaml\n")
    (tmp_path / "localization_1.yaml").write_text("image: a.pgm")
    (tmp_path / "planning_1.yaml").write_text("image: b.pgm")

    body = list_navigation_maps(str(tmp_path))

    assert body["success"] is True
    assert body["maps"] == [
        {"name": "localization_1.yaml",
         "path": str(tmp_path / "localization_1.yaml"), "missing": False},
        {"name": "planning_1.yaml",
         "path": str(tmp_path / "planning_1.yaml"), "missing": False},
        {"name": "missing.yaml",
         "path": str(tmp_path / "missing.yaml"), "missing": True},
    ]
    assert body["skipped"] == []


def test_list_navigation_maps_skips_paths_outside_map_path(tmp_path):
    _write_map_list(
        tmp_path, "../secret.yaml\n/etc/passwd\nsub/../../x.yaml\na b.yaml\nok.yaml\n")

    body = list_navigation_maps(str(tmp_path))

    assert [m["name"] for m in body["maps"]] == ["ok.yaml"]
    assert body["skipped"] == [
        "../secret.yaml", "/etc/passwd", "sub/../../x.yaml", "a b.yaml"]


def test_list_navigation_maps_without_map_list(tmp_path):
    body = list_navigation_maps(str(tmp_path))

    assert body["success"] is False
    assert "map_list.txt" in body["message"]


def test_list_navigation_maps_without_map_path():
    assert list_navigation_maps("") == {
        "success": False, "message": "MAP_PATH is not set"}


def test_get_navigation_maps_uses_map_path_setting(client, settings, tmp_path):
    _write_map_list(tmp_path, "a.yaml\n")
    client.app.state.settings = replace(settings, map_path=str(tmp_path))

    body = client.get("/navigation/maps").json()

    assert body["success"] is True
    assert [m["name"] for m in body["maps"]] == ["a.yaml"]
