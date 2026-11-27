"""MmiLink を仮想シリアル (pty) で確かめる。実機の MMI は不要。"""
import os
import sys
import time
import tty

import pytest
import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from mmi_protocol import HEARTBEAT  # noqa: E402
from mmi_serial import MmiLink  # noqa: E402

SW1 = bytes.fromhex("23610100")


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def pty_pair():
    """(MMI 側の fd, PC 側のデバイスパス)"""
    master, slave = os.openpty()
    tty.setraw(slave)
    path = os.ttyname(slave)
    yield master, path
    os.close(master)
    os.close(slave)


def _read_until(link, count, timeout=1.0):
    switches, ignored = [], []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and len(switches) + len(ignored) < count:
        s, i = link.read_switches()
        switches += s
        ignored += i
        time.sleep(0.01)
    return switches, ignored


def test_reads_a_press_after_the_ignore_window(pty_pair):
    master, path = pty_pair
    clock = FakeClock()
    link = MmiLink(path, 115200, ignore_after_open_sec=1.0, clock=clock)
    link.open()
    clock.now += 1.5
    os.write(master, SW1[:2])
    os.write(master, SW1[2:])
    assert _read_until(link, 1) == ([1], [])
    link.close()


def test_press_buffered_before_open_is_dropped(pty_pair):
    master, path = pty_pair
    os.write(master, SW1)
    clock = FakeClock()
    link = MmiLink(path, 115200, ignore_after_open_sec=0.0, clock=clock)
    link.open()
    clock.now += 1.0
    assert _read_until(link, 1, timeout=0.2) == ([], [])
    link.close()


def test_press_right_after_open_is_ignored(pty_pair):
    master, path = pty_pair
    clock = FakeClock()
    link = MmiLink(path, 115200, ignore_after_open_sec=1.0, clock=clock)
    link.open()
    clock.now += 0.5
    os.write(master, SW1)
    assert _read_until(link, 1) == ([], [1])
    link.close()


def test_write_reaches_the_device(pty_pair):
    master, path = pty_pair
    link = MmiLink(path, 115200, ignore_after_open_sec=0.0)
    link.open()
    link.write(HEARTBEAT)
    received = b""
    deadline = time.monotonic() + 1.0
    while len(received) < len(HEARTBEAT) and time.monotonic() < deadline:
        received += os.read(master, 64)
    assert received == HEARTBEAT
    link.close()


def test_open_fails_for_a_missing_device(tmp_path):
    link = MmiLink(str(tmp_path / "missing"), 115200, ignore_after_open_sec=0.0)
    with pytest.raises(serial.SerialException):
        link.open()
    assert not link.connected
