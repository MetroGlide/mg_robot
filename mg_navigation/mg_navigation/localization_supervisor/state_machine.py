"""自己位置の監督の状態遷移 (ROS に依存しない)。

判定 (Checks) を 1 秒程度ごとに受け取り、状態と、ノードが行う行動 (Action) を返す。

  NORMAL ─ 異常の疑い ─► SUSPECT ─ 確認 ─► ISOLATED (AMCL を EKF から切り離す)
     ▲                     │ 異常が消える            │ 一定時間待つ
     └─────────────────────┘                          ▼
     ▲                                          RECOVERING (EKF の姿勢で AMCL を初期化し、収束を確認)
     └──────────── 収束を確認 (AMCL を戻す) ◄──────────┤ 時間切れが続く
                                                        ▼
                                                    DEGRADED (復旧できない。一定時間ごとに再試行)
"""
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import List, Optional


class State(IntEnum):
    """mg_msgs/LocalizationStatus の state と対応する。"""
    NORMAL = 0
    SUSPECT = 1
    ISOLATED = 2
    RECOVERING = 3
    DEGRADED = 4


class Action(Enum):
    ISOLATE = 'isolate'                  # AMCL の出力を EKF に入れない
    REINIT = 'reinit'                    # 姿勢を選んで AMCL と EKF を初期化し直す
    ATTACH = 'attach'                    # AMCL の出力を EKF に戻す
    NOTIFY_DEGRADED = 'notify_degraded'  # 復旧できないことを知らせる


@dataclass
class MachineConfig:
    # 各判定が続けて異常のとき、SUSPECT / ISOLATED にする回数
    gnss_suspect_ticks: int = 3
    gnss_isolate_ticks: int = 6
    scan_suspect_ticks: int = 5
    scan_isolate_ticks: int = 10
    diff_suspect_ticks: int = 5
    diff_isolate_ticks: int = 10
    # AMCL の飛びが見つかった後、他の判定がこの回数続けば ISOLATED にする
    corroborate_ticks: int = 2
    # 異常がこの回数続けてなければ SUSPECT から NORMAL に戻る
    clear_ticks: int = 5
    # 切り離してから、初期化し直すまで待つ時間 [s] (EKF がオドメトリと GNSS だけで落ち着く時間)
    isolate_hold_sec: float = 3.0
    # 初期化し直した後、AMCL を戻すまでに正常を確認する回数と、諦めるまでの時間 [s]
    recover_ok_ticks: int = 5
    recover_timeout_sec: float = 30.0
    max_reinit_attempts: int = 3
    # DEGRADED から再試行する間隔 [s]
    degraded_retry_sec: float = 60.0


@dataclass
class Checks:
    """1 回分の判定。True が異常、False が正常、None が判定できない。"""
    gnss: Optional[bool] = None   # RTK などの精度の良い GNSS と AMCL の位置が合わない
    jump: Optional[bool] = None   # AMCL の推定が、オドメトリの示す動きから飛んだ
    scan: Optional[bool] = None   # EKF の姿勢でスキャンが地図に合わない
    diff: Optional[bool] = None   # AMCL と EKF の位置が離れている


@dataclass
class Decision:
    state: State
    actions: List[Action]
    event: str = ''


class SupervisorMachine:
    def __init__(self, config: Optional[MachineConfig] = None) -> None:
        self.config = config or MachineConfig()
        self.state = State.NORMAL
        self.attached = True
        self._reset_counters()
        self._isolated_at = 0.0
        self._reinit_at = 0.0
        self._degraded_at = 0.0
        self._attempts = 0

    def _reset_counters(self) -> None:
        self._gnss_run = 0
        self._scan_run = 0
        self._diff_run = 0
        self._jump_seen = False
        self._clear_run = 0
        self._ok_run = 0

    @staticmethod
    def _run(count: int, value: Optional[bool]) -> int:
        """連続して異常だった回数。正常で 0 に戻り、判定できないときは変えない。"""
        if value is True:
            return count + 1
        if value is False:
            return 0
        return count

    def _update_runs(self, checks: Checks) -> None:
        self._gnss_run = self._run(self._gnss_run, checks.gnss)
        self._scan_run = self._run(self._scan_run, checks.scan)
        self._diff_run = self._run(self._diff_run, checks.diff)
        if checks.jump:
            self._jump_seen = True

    def step(self, now: float, checks: Checks) -> Decision:
        cfg = self.config
        self._update_runs(checks)
        anomaly = any(v is True for v in (checks.gnss, checks.jump, checks.scan, checks.diff))
        known = any(v is not None for v in (checks.gnss, checks.jump, checks.scan, checks.diff))

        if self.state == State.NORMAL:
            self._jump_seen = bool(checks.jump)
            reason = self._suspect_reason(checks)
            if reason:
                self.state = State.SUSPECT
                self._clear_run = 0
                return Decision(self.state, [], f'suspect: {reason}')
            return Decision(self.state, [])

        if self.state == State.SUSPECT:
            reason = self._isolate_reason()
            if reason:
                self.state = State.ISOLATED
                self.attached = False
                self._isolated_at = now
                return Decision(self.state, [Action.ISOLATE], f'isolated: {reason}')
            if known and not anomaly:
                self._clear_run += 1
                if self._clear_run >= cfg.clear_ticks:
                    self.state = State.NORMAL
                    self._reset_counters()
                    return Decision(self.state, [], 'cleared')
            elif anomaly:
                self._clear_run = 0
            return Decision(self.state, [])

        if self.state == State.ISOLATED:
            if now - self._isolated_at >= cfg.isolate_hold_sec:
                return self._start_recovery(now, 'reinit')
            return Decision(self.state, [])

        if self.state == State.RECOVERING:
            # 初期化直後は AMCL が飛ぶので、飛びの判定は使わない
            bad = any(v is True for v in (checks.gnss, checks.scan, checks.diff))
            seen = any(v is not None for v in (checks.gnss, checks.scan, checks.diff))
            if bad:
                self._ok_run = 0
            elif seen:
                self._ok_run += 1
            if self._ok_run >= cfg.recover_ok_ticks:
                self.state = State.NORMAL
                self.attached = True
                self._reset_counters()
                return Decision(self.state, [Action.ATTACH], 'recovered')
            if now - self._reinit_at >= cfg.recover_timeout_sec:
                if self._attempts < cfg.max_reinit_attempts:
                    self._attempts += 1
                    self._reinit_at = now
                    self._ok_run = 0
                    return Decision(self.state, [Action.REINIT], f'reinit retry {self._attempts}')
                self.state = State.DEGRADED
                self._degraded_at = now
                return Decision(self.state, [Action.NOTIFY_DEGRADED], 'degraded')
            return Decision(self.state, [])

        # DEGRADED
        if now - self._degraded_at >= cfg.degraded_retry_sec:
            return self._start_recovery(now, 'retry from degraded')
        return Decision(self.state, [])

    def _suspect_reason(self, checks: Checks) -> str:
        cfg = self.config
        if checks.jump:
            return 'amcl_jump'
        if self._gnss_run >= cfg.gnss_suspect_ticks:
            return 'amcl_gnss'
        if self._scan_run >= cfg.scan_suspect_ticks:
            return 'scan_map'
        if self._diff_run >= cfg.diff_suspect_ticks:
            return 'amcl_ekf'
        return ''

    def _isolate_reason(self) -> str:
        cfg = self.config
        if self._gnss_run >= cfg.gnss_isolate_ticks:
            return 'amcl_gnss'
        if self._scan_run >= cfg.scan_isolate_ticks:
            return 'scan_map'
        if self._diff_run >= cfg.diff_isolate_ticks:
            return 'amcl_ekf'
        if self._jump_seen and max(self._gnss_run, self._scan_run, self._diff_run) >= cfg.corroborate_ticks:
            return 'amcl_jump'
        return ''

    def _start_recovery(self, now: float, event: str) -> Decision:
        self.state = State.RECOVERING
        self._reinit_at = now
        self._attempts = 1
        self._ok_run = 0
        return Decision(self.state, [Action.REINIT], event)
