"""MMI の USB CDC ポートの開閉と読み書き。ROS に依存しない。

ポートを開いた直後は、OS の受信バッファに残っていた古い押下 (ノードが止まっていた間の押下) を
捨てる。さらに ignore_after_open_sec の間に届いた押下も捨て、押したつもりのない時に
走り出さないようにする。
"""
import time
from typing import Callable, List, Optional, Tuple

import serial

from mmi_protocol import SwitchEventParser


class MmiLink:
    """読み書きで失敗したら serial.SerialException か OSError を投げる (呼び出し側が close する)。"""

    def __init__(self, device: str, baudrate: int, ignore_after_open_sec: float,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._device = device
        self._baudrate = baudrate
        self._ignore_after_open = ignore_after_open_sec
        self._clock = clock
        self._serial: Optional[serial.Serial] = None
        self._opened_at = 0.0
        self._parser = SwitchEventParser()

    @property
    def device(self) -> str:
        return self._device

    @property
    def connected(self) -> bool:
        return self._serial is not None

    def open(self) -> None:
        port = serial.Serial(self._device, self._baudrate, timeout=0, write_timeout=0.2)
        port.reset_input_buffer()
        self._serial = port
        self._opened_at = self._clock()
        self._parser.reset()

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
        self._serial = None

    def read_switches(self) -> Tuple[List[int], List[int]]:
        """届いた押下を (使う押下, 開いた直後なので捨てた押下) で返す。スイッチの番号は 1-3。"""
        if self._serial is None:
            return [], []
        waiting = self._serial.in_waiting
        if waiting <= 0:
            return [], []
        switches = self._parser.feed(self._serial.read(waiting))
        if self._clock() - self._opened_at < self._ignore_after_open:
            return [], switches
        return switches, []

    def write(self, packet: bytes) -> None:
        if self._serial is not None:
            self._serial.write(packet)
