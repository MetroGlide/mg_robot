"""ドライバの監視ノード (driver_watchdog_node.py) の、プロセスの再起動に関する判定。ROS に依存しない。

トピックの途絶の判定 (ActivityMonitor) と周期の遅れの検知 (TickGuard) は、
mg_utils.monitoring.activity の共通のものを使う。

- RestartPolicy: 途絶が続くドライバを、いつ終了させるか (クールダウン付き)
- ProcessTerminator: ROS ノード名からプロセスを探して終了させる
"""
import os
import signal
from typing import Callable, Dict, List, Optional


class RestartPolicy:
    """途絶が restart_after_sec 以上続いたら再起動を求める。直前の再起動から cooldown_sec は求めない"""

    def __init__(self, restart_after_sec: float, cooldown_sec: float) -> None:
        self._restart_after = restart_after_sec
        self._cooldown = cooldown_sec
        self._last_restart: Optional[float] = None

    def should_restart(self, now: float, stale_sec: float) -> bool:
        if stale_sec < self._restart_after:
            return False
        if self._last_restart is not None and now - self._last_restart < self._cooldown:
            return False
        self._last_restart = now
        return True


class ProcessTerminator:
    """ROS ノード名 (launch が渡す `__node:=<名前>`) からプロセスを探して終了させる。

    まず SIGTERM を送る。kill_timeout_sec たっても残っていれば SIGKILL を送る。
    自分と同じ PID 名前空間 (同じコンテナ) のプロセスだけが対象になる。
    """

    def __init__(
        self,
        kill_timeout_sec: float,
        proc_root: str = '/proc',
        kill: Callable[[int, int], None] = os.kill,
    ) -> None:
        self._kill_timeout = kill_timeout_sec
        self._proc_root = proc_root
        self._kill = kill
        # pid -> (ノード名, SIGKILL を送る時刻)
        self._pending: Dict[int, tuple] = {}

    def find_pids(self, node_name: str) -> List[int]:
        target = f'__node:={node_name}'.encode()
        own_pid = os.getpid()
        pids = []
        for entry in os.listdir(self._proc_root):
            if not entry.isdigit() or int(entry) == own_pid:
                continue
            try:
                with open(os.path.join(self._proc_root, entry, 'cmdline'), 'rb') as f:
                    args = f.read().split(b'\0')
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                # 列挙のあとに終了したプロセス
                continue
            if target in args:
                pids.append(int(entry))
        return pids

    def terminate(self, node_name: str, now: float) -> List[int]:
        pids = self.find_pids(node_name)
        for pid in pids:
            self._kill(pid, signal.SIGTERM)
            self._pending[pid] = (node_name, now + self._kill_timeout)
        return pids

    def escalate(self, now: float) -> List[int]:
        """SIGTERM のあとも残っているプロセスに SIGKILL を送る。終了したものは待ちから外す"""
        killed = []
        for pid, (node_name, deadline) in list(self._pending.items()):
            if pid not in self.find_pids(node_name):
                del self._pending[pid]
            elif now >= deadline:
                self._kill(pid, signal.SIGKILL)
                del self._pending[pid]
                killed.append(pid)
        return killed
