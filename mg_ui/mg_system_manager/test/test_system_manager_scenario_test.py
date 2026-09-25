import json

import pytest
from sm_fakes import FakeContainer


def _scenario_args(call) -> str:
    command = call["command"]
    for i, item in enumerate(command):
        if item == "-e" and command[i + 1].startswith("SCENARIO_ARGS="):
            return command[i + 1].removeprefix("SCENARIO_ARGS=")
    raise AssertionError(f"SCENARIO_ARGS not found: {command}")


def _write_progress(settings, run_id, entries, finished=False, exit_code=None):
    run = settings.scenario_results_dir / run_id
    run.mkdir(parents=True)
    (run / "progress.json").write_text(json.dumps({
        "started_at": "2026-09-25T10:00:00",
        "options": {"attach": False},
        "entries": entries,
        "finished": finished,
        "exit_code": exit_code,
    }))
    return run


def _entry(name, status):
    return {"name": name, "file": f"/s/{name}.yaml", "result_dir": name,
            "status": status, "message": "", "elapsed_sec": 1.0, "attempt": 0}


def test_list_scenarios_reads_yaml(client, settings):
    regression = settings.scenario_results_dir.parent / "scenarios" / "regression"
    (regression / "data").mkdir(parents=True)
    (regression / "b.yaml").write_text(
        "name: b\ndescription: >\n  説明\ntags: [smoke, x]\n")
    (regression / "a.yaml").write_text("name: a\n")
    (regression / "data" / "poses.yaml").write_text("x: 1\n")
    (regression / "broken.yaml").write_text("tags: [\n")

    scenarios = client.get("/scenario/list").json()["scenarios"]

    assert [s["name"] for s in scenarios] == ["a", "b", "broken"]
    assert scenarios[1] == {"name": "b", "group": "regression",
                            "tags": ["smoke", "x"], "description": "説明"}
    assert "error" in scenarios[2]


def test_start_runs_scenario_test_with_cli_args(client, compose_calls, settings):
    body = client.post("/scenario/run/start", json={
        "scenarios": ["nav_basic_goal", "static_avoid_cones"],
        "repeat": 2, "gui": True}).json()

    assert body["success"] is True
    run_id = body["run_id"]
    assert run_id
    call = compose_calls.calls[-1]
    assert call["command"][:4] == ["docker", "compose", "run", "-d"]
    assert call["command"][-1] == "scenario-test"
    dirs = settings.scenario_dirs
    assert _scenario_args(call) == (
        f"run-all nav_basic_goal static_avoid_cones "
        f"--scenario-dir {dirs[0]} --scenario-dir {dirs[1]} "
        f"--results-dir {settings.scenario_results_dir / run_id} "
        f"--repeat 2 --gui")


def test_start_with_robot_passes_remote_env(client, compose_calls):
    body = client.post("/scenario/run/start", json={
        "scenarios": ["nav_basic_goal"], "robot": "192.168.0.10"}).json()

    assert body["success"] is True
    command = compose_calls.calls[-1]["command"]
    assert command[4:10] == [
        "-e", "ROS_DOMAIN_ID=42",
        "-e", "CYCLONEDDS_URI=file:///app/docker/cyclonedds/remote.xml",
        "-e", "MG_REMOTE_PEER=192.168.0.10"]
    assert _scenario_args(compose_calls.calls[-1]).endswith(
        "--remote-stack http://192.168.0.10:8001")


def test_attach_needs_running_environment(client, containers, compose_calls):
    body = client.post("/scenario/run/start", json={
        "scenarios": ["nav_basic_goal"], "attach": True}).json()
    assert body["success"] is False
    assert "attach needs" in body["message"]
    assert compose_calls.calls == []

    containers.append(FakeContainer("scenario-env", "running"))
    body = client.post("/scenario/run/start", json={
        "scenarios": ["nav_basic_goal"], "attach": True, "gui": True}).json()
    assert body["success"] is True
    args = _scenario_args(compose_calls.calls[-1])
    assert args.endswith("--attach")
    assert "--gui" not in args


@pytest.mark.parametrize("service", ["scenario-env", "gazebo-simulation", "navigation"])
def test_normal_run_rejects_conflicting_services(
        client, containers, compose_calls, service):
    containers.append(FakeContainer(service, "running"))

    body = client.post("/scenario/run/start", json={
        "scenarios": ["nav_basic_goal"]}).json()

    assert body["success"] is False
    assert service in body["message"]
    assert compose_calls.calls == []


def test_start_rejects_while_running_and_removes_stopped(
        client, containers, compose_calls):
    old = FakeContainer("scenario-test", "exited", oneoff=True)
    containers.append(old)
    assert client.post("/scenario/run/start", json={
        "scenarios": ["a"]}).json()["success"] is True
    assert old.removed is True

    containers.append(FakeContainer("scenario-test", "running", oneoff=True))
    body = client.post("/scenario/run/start", json={"scenarios": ["a"]}).json()
    assert body == {"success": False,
                    "message": "scenario test is already running"}


@pytest.mark.parametrize("body,message", [
    ({"scenarios": []}, "no scenarios selected"),
    ({"scenarios": ["../x"]}, "invalid scenario name: ../x"),
    ({"scenarios": ["a b"]}, "invalid scenario name: a b"),
    ({"scenarios": ["a"], "repeat": 0}, "repeat must be between 1 and 20"),
    ({"scenarios": ["a"], "robot": "1.2.3.4;id"}, "invalid robot address"),
    ({"scenarios": ["a"], "robot": "1.2.3.4", "attach": True},
     "robot and attach cannot be used together"),
])
def test_start_rejects_invalid_input(client, compose_calls, body, message):
    assert client.post("/scenario/run/start", json=body).json() == {
        "success": False, "message": message}
    assert compose_calls.calls == []


def test_status_reports_latest_run_and_aborted(
        client, containers, settings, runner):
    _write_progress(settings, "20260925_090000", [_entry("a", "PASSED")],
                    finished=True, exit_code=0)
    _write_progress(settings, "20260925_100000",
                    [_entry("a", "PASSED"), _entry("b", "RUNNING")])

    body = client.get("/scenario/status").json()
    assert body["run"]["run_id"] == "20260925_100000"
    assert body["run"]["state"] == "aborted"

    containers.append(FakeContainer("scenario-test", "running", oneoff=True))
    runner.get_status(fresh=True)
    body = client.get("/scenario/status").json()
    assert body["services"]["scenario-test"] == "running"
    assert body["run"]["state"] == "running"
    assert [e["status"] for e in body["run"]["entries"]] == ["PASSED", "RUNNING"]


def test_runs_are_listed_newest_first_with_counts(client, settings):
    _write_progress(settings, "20260925_090000",
                    [_entry("a", "PASSED"), _entry("b", "FAILED")],
                    finished=True, exit_code=1)
    _write_progress(settings, "20260925_100000", [_entry("a", "PENDING")])
    (settings.scenario_results_dir / "no_progress").mkdir()

    runs = client.get("/scenario/runs").json()["runs"]

    assert [r["run_id"] for r in runs] == ["20260925_100000", "20260925_090000"]
    assert runs[0]["state"] == "aborted"
    assert runs[1]["counts"] == {"PASSED": 1, "FAILED": 1}
    assert (runs[1]["state"], runs[1]["exit_code"], runs[1]["total"]) == (
        "finished", 1, 2)


def test_result_and_log_of_an_entry(client, settings):
    run = _write_progress(settings, "20260925_100000", [_entry("a", "FAILED")],
                          finished=True, exit_code=1)
    (run / "a").mkdir()
    (run / "a" / "result.json").write_text(json.dumps({"status": "FAILED"}))
    (run / "a" / "launch.log").write_text(
        "\n".join(f"line{i}" for i in range(10)))

    assert client.get(
        "/scenario/runs/20260925_100000/a/result").json() == {
            "status": "FAILED"}
    log = client.get(
        "/scenario/runs/20260925_100000/a/log?tail=3").json()["log"]
    assert log == "line7\nline8\nline9"
    assert client.get(
        "/scenario/runs/20260925_100000/a/log?name=stack").status_code == 404
    assert client.get(
        "/scenario/runs/20260925_100000/a/log?name=../x").status_code == 422
    assert client.get("/scenario/runs/nope/a/result").status_code == 404


def test_result_rejects_path_traversal(client, settings):
    _write_progress(settings, "r1", [], finished=True)
    (settings.scenario_results_dir.parent / "secret.json").write_text("{}")

    assert client.get("/scenario/runs/r1/..%2F..%2Fsecret/result"
                      ).status_code == 404
    assert client.get("/scenario/runs/..%2Fr1/x/result").status_code == 404


def test_env_start_passes_profile_and_gui(client, compose_calls):
    body = client.post("/scenario/env/start", json={"gui": True}).json()

    assert body["success"] is True
    call = compose_calls.calls[-1]
    assert call["command"] == [
        "docker", "compose", "up", "-d", "--force-recreate", "scenario-env"]
    assert call["env"]["SCENARIO_ENV_ARGS"] == (
        "profile:=mg01 world:=warehouse headless:=false")


def test_env_start_rejects_conflicts(client, containers, compose_calls):
    containers.append(FakeContainer("gazebo-simulation", "running"))

    body = client.post("/scenario/env/start", json={}).json()

    assert body["success"] is False
    assert "gazebo-simulation" in body["message"]
    assert compose_calls.calls == []
