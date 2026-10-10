import os
import signal
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from driver_watchdog_core import (  # noqa: E402
    ProcessTerminator,
    RestartPolicy,
    SourceMonitor,
    TickGuard,
)


def _monitor(**kwargs):
    params = dict(stale_timeout_sec=1.0, recover_hold_sec=2.0, start_time=0.0)
    params.update(kwargs)
    return SourceMonitor(**params)


def test_healthy_source_is_not_faulted():
    m = _monitor()
    for t in range(0, 10):
        m.on_message(t * 0.1)
        assert not m.update(t * 0.1)


def test_stale_source_becomes_faulted():
    m = _monitor()
    m.on_message(0.0)
    assert not m.update(0.9)
    assert m.update(1.1)
    assert m.stale_sec(1.1) == pytest.approx(1.1)


def test_startup_grace_delays_fault_without_first_message():
    m = _monitor(startup_grace_sec=30.0)
    assert not m.update(20.0)
    assert m.stale_sec(20.0) == 0.0
    # 猶予が終わってからさらに stale_timeout_sec たっても来なければ異常
    assert not m.update(30.5)
    assert m.update(31.5)


def test_no_first_message_without_grace_is_faulted_after_timeout():
    m = _monitor()
    assert not m.update(0.5)
    assert m.update(1.5)


def test_armed_on_first_message_ignores_source_that_never_published():
    m = _monitor(armed_on_first_message=True)
    assert not m.update(100.0)
    assert m.stale_sec(100.0) == 0.0
    m.on_message(100.0)
    assert not m.update(100.5)
    assert m.update(102.0)


def test_recovery_requires_hold_time():
    m = _monitor()
    m.on_message(0.0)
    assert m.update(2.0)

    m.on_message(2.0)
    assert m.update(2.1)  # 戻ったが、まだ保持時間に達しない
    m.on_message(3.0)
    assert m.update(3.0)
    m.on_message(4.0)
    assert not m.update(4.2)  # 2.1 から 2 秒以上正常が続いた


def test_fault_during_hold_restarts_the_hold():
    m = _monitor()
    m.on_message(0.0)
    assert m.update(2.0)
    m.on_message(2.0)
    assert m.update(2.5)  # 保持の開始 (2.5)
    assert m.update(4.0)  # 再び途絶 (最後の受信は 2.0)。保持は振り出しに戻る
    m.on_message(4.0)
    assert m.update(4.1)  # 保持の開始 (4.1)
    m.on_message(5.0)
    assert m.update(5.0)
    m.on_message(6.0)
    assert not m.update(6.2)  # 4.1 から 2 秒以上正常が続いた


def test_not_ok_message_is_fault_even_if_fresh():
    m = _monitor()
    m.on_message(0.0, ok=False)
    assert m.update(0.1)
    assert m.disconnected
    m.on_message(0.2, ok=True)
    assert m.update(0.3)
    m.on_message(2.3, ok=True)
    assert not m.update(2.4)


def test_forgive_moves_last_receive_forward():
    m = _monitor()
    m.on_message(0.0)
    m.forgive(5.0)
    assert not m.update(5.5)
    assert m.stale_sec(5.5) == pytest.approx(0.5)


def test_forgive_does_not_arm_a_source_without_messages():
    m = _monitor(armed_on_first_message=True)
    m.forgive(5.0)
    assert not m.received
    assert not m.update(100.0)


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


def test_tick_guard_detects_large_gap():
    g = TickGuard(max_gap_sec=2.0)
    assert not g.check(0.0)
    assert not g.check(0.5)
    assert g.check(3.0)
    assert not g.check(3.5)
