from sm_fakes import FakeContainer


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


def test_start_failure_returns_stderr(client, compose_calls):
    compose_calls.returncode = 1
    compose_calls.stderr = "no such service"

    body = client.post("/navigation/start").json()

    assert body == {"success": False, "message": "no such service"}


def test_restart_runs_compose_restart(client, compose_calls):
    client.post("/diagnostics/restart")

    assert compose_calls.calls[0]["command"] == [
        "docker", "compose", "restart", "diagnostics"]


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
