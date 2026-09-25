import asyncio

from mg_system_manager import log_hub
from mg_system_manager.log_hub import LogHub, Subscriber
from sm_fakes import FakeContainer, FakeDockerClient
from mg_system_manager.docker_ops import ComposeRunner


class FakeProcess:
    def __init__(self, lines: list[bytes], limit: int) -> None:
        self.stdout = asyncio.StreamReader(limit=limit)
        for line in lines:
            self.stdout.feed_data(line)
        self.returncode = None
        self.terminated = False

    def finish(self) -> None:
        self.stdout.feed_eof()

    async def wait(self) -> int:
        if self.returncode is None:
            await self.stdout.read()  # EOF まで待つ
            self.returncode = 0
        return self.returncode

    def terminate(self) -> None:
        self.terminated = True
        self.returncode = -15
        self.stdout.feed_eof()


def _make_hub(settings, containers, monkeypatch, lines=()):
    processes: list[FakeProcess] = []

    async def fake_exec(*args, **kwargs):
        process = FakeProcess(list(lines), kwargs["limit"])
        processes.append(process)
        return process

    monkeypatch.setattr(log_hub.asyncio, "create_subprocess_exec", fake_exec)
    runner = ComposeRunner(settings, client=FakeDockerClient(containers))
    return LogHub(runner), processes


def _run(coro):
    return asyncio.run(coro)


def test_subscriber_drops_oldest_entries_when_full():
    async def scenario():
        subscriber = Subscriber(max_entries=3)
        for i in range(5):
            subscriber.push({"service": "slam", "line": str(i)})
        return subscriber.drain()

    entries, dropped = _run(scenario())

    assert [e["line"] for e in entries] == ["2", "3", "4"]
    assert dropped == 2


def test_subscriber_drain_resets_state():
    async def scenario():
        subscriber = Subscriber(max_entries=3)
        subscriber.push({"service": "slam", "line": "a"})
        subscriber.drain()
        return subscriber.drain(), subscriber.ready.is_set()

    (entries, dropped), ready = _run(scenario())

    assert entries == [] and dropped == 0 and ready is False


def test_two_subscribers_share_one_docker_logs_process(
        settings, containers, monkeypatch):
    containers.append(FakeContainer("slam"))
    hub, processes = _make_hub(
        settings, containers, monkeypatch, lines=[b"hello\n"])

    async def scenario():
        first, second = Subscriber(10), Subscriber(10)
        hub.set_services(first, {"slam"})
        hub.set_services(second, {"slam"})
        await asyncio.wait_for(first.ready.wait(), 1)
        await asyncio.wait_for(second.ready.wait(), 1)
        result = first.drain()[0], second.drain()[0]
        await hub.close()
        return result

    first, second = _run(scenario())

    assert len(processes) == 1
    assert first == second == [{"service": "slam", "line": "hello"}]


def test_process_is_terminated_when_last_subscriber_leaves(
        settings, containers, monkeypatch):
    containers.append(FakeContainer("slam"))
    hub, processes = _make_hub(settings, containers, monkeypatch)

    async def scenario():
        subscriber = Subscriber(10)
        hub.set_services(subscriber, {"slam"})
        await asyncio.sleep(0.05)
        hub.remove_subscriber(subscriber)
        await asyncio.sleep(0.05)
        await hub.close()

    _run(scenario())

    assert len(processes) == 1
    assert processes[0].terminated is True


def test_changing_services_stops_removed_ones(
        settings, containers, monkeypatch):
    containers.extend([FakeContainer("slam"), FakeContainer("navigation")])
    hub, processes = _make_hub(settings, containers, monkeypatch)

    async def scenario():
        subscriber = Subscriber(10)
        hub.set_services(subscriber, {"slam", "navigation"})
        await asyncio.sleep(0.05)
        hub.set_services(subscriber, {"slam"})
        await asyncio.sleep(0.05)
        await hub.close()

    _run(scenario())

    assert len(processes) == 2
    assert sum(p.terminated for p in processes) == 2  # 1 つは切替、もう 1 つは close


def test_missing_container_error_is_reported_once(
        settings, containers, monkeypatch):
    hub, _ = _make_hub(settings, containers, monkeypatch)
    monkeypatch.setattr(log_hub, "RETRY_INTERVAL_S", 0.01)

    async def scenario():
        subscriber = Subscriber(10)
        hub.set_services(subscriber, {"slam"})
        await asyncio.sleep(0.1)
        result = subscriber.drain()[0]
        await hub.close()
        return result

    entries = _run(scenario())

    assert entries == [{"service": "slam", "error": "container not found"}]


def test_websocket_sends_lines_in_batches(client, containers, monkeypatch):
    containers.append(FakeContainer("slam"))

    async def fake_exec(*args, **kwargs):
        return FakeProcess([b"a\n", b"b\n"], kwargs["limit"])

    monkeypatch.setattr(log_hub.asyncio, "create_subprocess_exec", fake_exec)

    with client.websocket_connect("/logs/stream") as ws:
        ws.send_text('{"services": ["slam", "not-a-service"]}')
        message = ws.receive_json()

    assert message == {
        "entries": [
            {"service": "slam", "line": "a"},
            {"service": "slam", "line": "b"},
        ],
        "dropped": 0,
    }


def test_long_line_is_truncated_and_does_not_break_stream(
        settings, containers, monkeypatch):
    containers.append(FakeContainer("slam"))
    long_line = b"x" * 200_000 + b"\n"
    hub, _ = _make_hub(
        settings, containers, monkeypatch, lines=[long_line, b"next\n"])

    async def scenario():
        subscriber = Subscriber(10)
        hub.set_services(subscriber, {"slam"})
        await asyncio.sleep(0.1)
        result = subscriber.drain()[0]
        await hub.close()
        return result

    entries = _run(scenario())

    assert len(entries[0]["line"]) == log_hub.MAX_LINE_CHARS
    assert entries[1]["line"] == "next"
