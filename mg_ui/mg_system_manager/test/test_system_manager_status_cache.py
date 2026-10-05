from mg_system_manager import docker_ops
from mg_system_manager.docker_ops import ComposeRunner
from sm_fakes import FakeContainer, FakeDockerClient


def _runner(settings, containers):
    client = FakeDockerClient(containers)
    return ComposeRunner(settings, client=client), client


def test_status_uses_list_api_without_inspecting_each_container(
        settings, containers):
    containers.append(FakeContainer("slam", "running"))
    runner, client = _runner(settings, containers)

    assert runner.get_status() == {"slam": "running"}
    assert client.api.calls == 1


def test_status_is_cached_for_a_short_time(settings, containers, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(docker_ops.time, "monotonic", lambda: now[0])
    runner, client = _runner(settings, containers)

    runner.get_status()
    now[0] += 0.5
    runner.get_status()
    assert client.api.calls == 1

    now[0] += 0.6
    runner.get_status()
    assert client.api.calls == 2


def test_fresh_status_bypasses_the_cache(settings, containers):
    runner, client = _runner(settings, containers)
    runner.get_status()

    containers.append(FakeContainer("slam", "running"))

    assert runner.get_status() == {}
    assert runner.get_status(fresh=True) == {"slam": "running"}


def test_status_is_refreshed_after_an_operation(
        settings, containers, compose_calls):
    runner, client = _runner(settings, containers)
    runner.get_status()

    runner.up("slam")
    containers.append(FakeContainer("slam", "running"))

    assert runner.get_status() == {"slam": "running"}


def test_status_prefers_running_and_ignores_other_projects(
        settings, containers):
    containers.extend([
        FakeContainer("scenario-test", "exited"),
        FakeContainer("scenario-test", "running", oneoff=True),
        FakeContainer("slam", "running", project="other"),
    ])
    runner, _ = _runner(settings, containers)

    assert runner.get_status() == {"scenario-test": "running"}


def test_stop_is_reflected_in_the_next_status(settings, containers):
    containers.append(FakeContainer("slam", "running"))
    runner, _ = _runner(settings, containers)
    assert runner.get_status() == {"slam": "running"}

    runner.stop("slam")

    assert runner.get_status() == {"slam": "exited"}
