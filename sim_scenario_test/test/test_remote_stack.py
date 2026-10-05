"""リモートスタッククライアントと、run_scenario の remote_stack 連携の単体テスト。"""
from __future__ import annotations

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from sim_scenario_test.engine.result import ResultStatus
from sim_scenario_test.execution import is_infrastructure_error, run_scenario
from sim_scenario_test.remote_stack import RemoteStackClient, RemoteStackError, StackLaunch

LAUNCH = StackLaunch("pkg", "launch/nav.launch.py", {"map_path": "/m.yaml"})


class _Handler(BaseHTTPRequestHandler):
    calls = []
    fail_start = False

    def _reply(self, body):
        data = json.dumps(body).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length))
        type(self).calls.append((self.path, body))
        if self.path.endswith("/start") and type(self).fail_start:
            self._reply({"success": False, "message": "busy"})
        else:
            self._reply({"success": True, "message": ""})

    def do_GET(self):
        type(self).calls.append((self.path, None))
        self._reply({"logs": "stack log"})

    def log_message(self, *args):
        pass


@pytest.fixture
def server():
    _Handler.calls = []
    _Handler.fail_start = False
    httpd = HTTPServer(("127.0.0.1", 0), _Handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{httpd.server_port}"
    httpd.shutdown()


def test_client_start_stop_logs(server):
    client = RemoteStackClient(server)
    client.start(LAUNCH)
    assert client.logs() == "stack log"
    client.stop()
    assert _Handler.calls == [
        ("/scenario-stack/start",
         {"package": "pkg", "file": "launch/nav.launch.py", "args": {"map_path": "/m.yaml"}}),
        ("/scenario-stack/logs", None),
        ("/scenario-stack/stop", {}),
    ]


def test_client_start_failure_raises(server):
    _Handler.fail_start = True
    with pytest.raises(RemoteStackError, match="busy"):
        RemoteStackClient(server).start(LAUNCH)


def test_client_unreachable_raises():
    with pytest.raises(RemoteStackError):
        RemoteStackClient("http://127.0.0.1:9", timeout_sec=1.0).stop()


@pytest.fixture
def fake_launch(tmp_path):
    script = tmp_path / "fake_launch.py"
    script.write_text(
        "import json, sys\n"
        "args = dict(a.split(':=', 1) for a in sys.argv if ':=' in a)\n"
        "print('fake launch', sys.argv[1:])\n"
        "json.dump({'scenario_name': 'fake', 'status': 'PASSED', 'checks': []},"
        " open(args['result_file'], 'w'))\n")
    return [sys.executable, str(script)]


def _run(fake_launch, tmp_path, url, **kwargs):
    return run_scenario(
        "s.yaml", str(tmp_path / "out"), launch_prefix=fake_launch,
        remote_stack=RemoteStackClient(url, timeout_sec=2.0),
        stack_resolver=lambda scenario_file, profile: LAUNCH, **kwargs)


def test_run_starts_and_stops_remote_stack(server, fake_launch, tmp_path):
    rec = _run(fake_launch, tmp_path, server)
    assert rec.status == ResultStatus.PASSED
    assert [path for path, _ in _Handler.calls] == [
        "/scenario-stack/start", "/scenario-stack/logs", "/scenario-stack/stop"]
    assert "launch_stack:=false" in (tmp_path / "out" / "launch.log").read_text()
    assert (tmp_path / "out" / "stack.log").read_text() == "stack log"


def test_start_failure_is_retryable_error(server, fake_launch, tmp_path):
    _Handler.fail_start = True
    rec = _run(fake_launch, tmp_path, server)
    assert rec.status == ResultStatus.ERROR and is_infrastructure_error(rec)
    assert [path for path, _ in _Handler.calls] == ["/scenario-stack/start"]
    assert not (tmp_path / "out" / "launch.log").exists()


def test_stack_is_stopped_even_when_launch_crashes(server, tmp_path):
    crash = tmp_path / "crash.py"
    crash.write_text("raise SystemExit(1)\n")
    rec = _run([sys.executable, str(crash)], tmp_path, server)
    assert rec.status == ResultStatus.ERROR
    assert _Handler.calls[-1][0] == "/scenario-stack/stop"
