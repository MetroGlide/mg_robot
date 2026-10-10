"""ドライバの監視ノード (driver_watchdog_node.py) の判定ロジック。ROS に依存しない。

- SourceMonitor: トピックの途絶を見て、異常の有無を判定する
- RestartPolicy: 途絶が続くドライバを、いつ終了させるか (クールダウン付き)
- ProcessTerminator: ROS ノード名からプロセスを探して終了させる
- TickGuard: 監視ノード自身の周期の遅れを検知する
"""
import os
import signal
from typing import Callable, Dict, List, Optional


class SourceMonitor:
    """1 つのトピックの鮮度を見て、異常 (faulted) を判定する。

    - 最後の受信から stale_timeout_sec を超えたら異常にする
    - 異常から戻るには、recover_hold_sec のあいだ正常が続く必要がある (ばたつきを防ぐ)
    - 起動直後は startup_grace_sec のあいだ、初回のメッセージが無くても異常にしない
    - armed_on_first_message が true のときは、初回のメッセージを受け取るまで監視しない
    - ok=False のメッセージ (接続状態が false など) は、途絶と同じく異常にする
    """

    def __init__(
        self,
        stale_timeout_sec: float,
        recover_hold_sec: float,
        start_time: float,
        startup_grace_sec: float = 0.0,
        armed_on_first_message: bool = False,
    ) -> None:
        self._stale_timeout = stale_timeout_sec
        self._recover_hold = recover_hold_sec
        self._start_time = start_time
        self._startup_grace = startup_grace_sec
        self._armed_on_first = armed_on_first_message
        self._last_rx: Optional[float] = None
        self._ok = True
        self._faulted = False
        self._good_since: Optional[float] = None

    @property
    def faulted(self) -> bool:
        return self._faulted

    @property
    def disconnected(self) -> bool:
        """最後のメッセージが ok=False だった"""
        return not self._ok

    @property
    def received(self) -> bool:
        return self._last_rx is not None

    def on_message(self, now: float, ok: bool = True) -> None:
        self._last_rx = now
        self._ok = ok

    def forgive(self, now: float) -> None:
        """監視側の遅れで、受信の記録が古く見えているとき、受信時刻を今に進める"""
        if self._last_rx is not None:
            self._last_rx = max(self._last_rx, now)

    def stale_sec(self, now: float) -> float:
        """途絶している時間。猶予の間と、監視前は 0"""
        if self._last_rx is None:
            if self._armed_on_first:
                return 0.0
            return max(0.0, now - (self._start_time + self._startup_grace))
        return max(0.0, now - self._last_rx)

    def update(self, now: float) -> bool:
        """判定を更新し、異常かどうかを返す"""
        bad = self.stale_sec(now) > self._stale_timeout or not self._ok
        if bad:
            self._faulted = True
            self._good_since = None
        elif self._faulted:
            if self._good_since is None:
                self._good_since = now
            if now - self._good_since >= self._recover_hold:
                self._faulted = False
                self._good_since = None
        return self._faulted


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


class TickGuard:
    """周期処理の間隔が max_gap_sec を超えたら、遅れたとみなす (CPU の詰まりなど)"""

    def __init__(self, max_gap_sec: float) -> None:
        self._max_gap = max_gap_sec
        self._last: Optional[float] = None

    def check(self, now: float) -> bool:
        lagged = self._last is not None and now - self._last > self._max_gap
        self._last = now
        return lagged
