import threading

from sm_fakes import FakeContainer


def _post_in_thread(client, path, results, key):
    results[key] = client.post(path).json()


def test_second_operation_on_same_service_is_rejected(
        client, runner, compose_calls):
    with runner.operation("slam"):
        body = _run_in_other_thread(client, "/slam/start")

    assert body == {
        "success": False,
        "message": "another operation is in progress: slam"}
    assert compose_calls.calls == []


def test_lock_is_released_after_operation(client, compose_calls):
    assert client.post("/slam/start").json()["success"] is True
    assert client.post("/slam/start").json()["success"] is True
    assert len(compose_calls.calls) == 2


def test_lock_is_released_when_command_fails(client, compose_calls):
    compose_calls.returncode = 1
    client.post("/slam/start")

    compose_calls.returncode = 0
    assert client.post("/slam/start").json()["success"] is True


def test_different_services_do_not_block_each_other(
        client, runner, compose_calls):
    with runner.operation("rviz2"):
        body = _run_in_other_thread(client, "/diagnostics/start")

    assert body["success"] is True


def test_hardware_start_is_blocked_while_scenario_stack_is_starting(
        client, runner, compose_calls):
    with runner.operation("scenario-remote-stack", exclusive=True):
        body = _run_in_other_thread(client, "/navigation/start")

    assert body["success"] is False
    assert "another operation is in progress" in body["message"]
    assert compose_calls.calls == []


def test_non_hardware_service_is_not_blocked_by_scenario_stack(
        client, runner, compose_calls):
    with runner.operation("scenario-remote-stack", exclusive=True):
        body = _run_in_other_thread(client, "/rviz2/start")

    assert body["success"] is True


def test_stop_is_rejected_while_starting_same_service(
        client, runner, containers):
    containers.append(FakeContainer("slam", "running"))

    with runner.operation("slam", exclusive=True):
        body = _run_in_other_thread(client, "/slam/stop")

    assert body["success"] is False
    assert containers[0].status == "running"


def test_scenario_stack_start_reentrant_in_same_request(client, compose_calls):
    body = client.post("/scenario-stack/start", json={
        "package": "mg_bringup", "file": "sim.launch.py"}).json()

    assert body["success"] is True


def _run_in_other_thread(client, path):
    """RLock は同じスレッドからの入れ子を許すため、別スレッドから呼び出す。"""
    results = {}
    thread = threading.Thread(
        target=_post_in_thread, args=(client, path, results, "r"))
    thread.start()
    thread.join()
    return results["r"]
