from mg_system_manager.config import Settings
from sm_fakes import FakeContainer


def test_other_project_containers_are_ignored(client, containers):
    containers.append(FakeContainer("slam", "running", project="other"))

    assert client.get("/status").json() == {}
    assert client.post("/slam/stop").json()["success"] is False


def test_status_prefers_running_container_of_same_service(client, containers):
    containers.append(FakeContainer("scenario-test", "exited"))
    containers.append(FakeContainer("scenario-test", "running", oneoff=True))

    assert client.get("/status").json() == {"scenario-test": "running"}


def test_stop_targets_running_container_not_stale_one(client, containers):
    stale = FakeContainer("scenario-test", "exited")
    live = FakeContainer("scenario-test", "running", oneoff=True)
    containers.extend([stale, live])

    body = client.post("/scenario-test/stop").json()

    assert body["success"] is True
    assert live.status == "exited"
    assert stale.stopped_with == []


def test_stop_stops_every_running_container_of_service(client, containers):
    first = FakeContainer("scenario-test", "running")
    second = FakeContainer("scenario-test", "running", oneoff=True)
    containers.extend([first, second])

    client.post("/scenario-test/stop")

    assert first.status == "exited" and second.status == "exited"


def test_default_compose_project_matches_compose_naming(monkeypatch):
    monkeypatch.delenv("COMPOSE_PROJECT_NAME", raising=False)
    monkeypatch.setenv("HOST_PROJECT_DIR", "/home/user/My Robot.v2")

    assert Settings.from_env().compose_project == "myrobotv2"


def test_compose_project_name_env_takes_precedence(monkeypatch):
    monkeypatch.setenv("COMPOSE_PROJECT_NAME", "mg")

    assert Settings.from_env().compose_project == "mg"
