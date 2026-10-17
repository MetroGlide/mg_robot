import os
import signal
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from driver_watchdog_core import (  # noqa: E402
    ProcessTerminator,
    RestartPolicy,
)


def test_restart_policy_waits_for_stale_time_then_cools_down():
    p = RestartPolicy(restart_after_sec=3.0, cooldown_sec=10.0)
    assert not p.should_restart(0.0, 2.9)
    assert p.should_restart(1.0, 3.0)
    assert not p.should_restart(5.0, 7.0)  # クールダウン中
    assert not p.should_restart(10.9, 12.0)
    assert p.should_restart(11.0, 13.0)


def _make_proc(root, pid, args):
    d = root / str(pid)
    d.mkdir()
    (d / "cmdline").write_bytes(b"\0".join(a.encode() for a in args) + b"\0")


@pytest.fixture
def fake_proc(tmp_path):
    _make_proc(tmp_path, 101, [
        "/root/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node",
        "--ros-args", "-r", "__node:=top_rplidar_node"])
    _make_proc(tmp_path, 102, [
        "/root/ros2_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node",
        "--ros-args", "-r", "__node:=front_rplidar_node"])
    _make_proc(tmp_path, 103, ["ros2", "launch", "mg_drivers", "bringup.launch.py"])
    (tmp_path / "self").mkdir()
    (tmp_path / "meminfo").write_text("x")
    return tmp_path


class _Killer:
    def __init__(self):
        self.calls = []

    def __call__(self, pid, sig):
        self.calls.append((pid, sig))


def test_find_pids_matches_exact_node_name(fake_proc):
    t = ProcessTerminator(3.0, proc_root=str(fake_proc), kill=_Killer())
    assert t.find_pids("top_rplidar_node") == [101]
    assert t.find_pids("rplidar_node") == []
    assert t.find_pids("missing_node") == []


def test_find_pids_skips_process_that_disappeared(fake_proc):
    (fake_proc / "104").mkdir()  # cmdline が無い (列挙のあとに終了した)
    t = ProcessTerminator(3.0, proc_root=str(fake_proc), kill=_Killer())
    assert t.find_pids("top_rplidar_node") == [101]


def test_find_pids_excludes_own_process(fake_proc):
    _make_proc(fake_proc, os.getpid(), ["python3", "-r", "__node:=top_rplidar_node"])
    t = ProcessTerminator(3.0, proc_root=str(fake_proc), kill=_Killer())
    assert t.find_pids("top_rplidar_node") == [101]


def test_terminate_sends_sigterm_only_to_target(fake_proc):
    killer = _Killer()
    t = ProcessTerminator(3.0, proc_root=str(fake_proc), kill=killer)
    assert t.terminate("top_rplidar_node", now=10.0) == [101]
    assert killer.calls == [(101, signal.SIGTERM)]


def test_escalate_kills_only_after_timeout_if_still_alive(fake_proc):
    killer = _Killer()
    t = ProcessTerminator(3.0, proc_root=str(fake_proc), kill=killer)
    t.terminate("top_rplidar_node", now=10.0)

    assert t.escalate(12.0) == []
    assert t.escalate(13.0) == [101]
    assert killer.calls == [(101, signal.SIGTERM), (101, signal.SIGKILL)]
    assert t.escalate(14.0) == []  # 待ちから外れている


def test_escalate_does_not_kill_process_that_exited(fake_proc):
    killer = _Killer()
    t = ProcessTerminator(3.0, proc_root=str(fake_proc), kill=killer)
    t.terminate("top_rplidar_node", now=10.0)
    (fake_proc / "101" / "cmdline").unlink()
    (fake_proc / "101").rmdir()

    assert t.escalate(20.0) == []
    assert killer.calls == [(101, signal.SIGTERM)]


def test_escalate_does_not_kill_reused_pid_of_another_node(fake_proc):
    killer = _Killer()
    t = ProcessTerminator(3.0, proc_root=str(fake_proc), kill=killer)
    t.terminate("top_rplidar_node", now=10.0)
    # 同じ PID を別のノードが使っている
    _make_proc_args = ["x", "-r", "__node:=other_node"]
    (fake_proc / "101" / "cmdline").write_bytes(
        b"\0".join(a.encode() for a in _make_proc_args) + b"\0")

    assert t.escalate(20.0) == []
    assert killer.calls == [(101, signal.SIGTERM)]
