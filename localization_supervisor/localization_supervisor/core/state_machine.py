"""自己位置の監督の状態遷移 (ROS に依存しない)。

総合の結果 (FusionResult) と、復旧の収束の確認 (converged) を周期ごとに受け取り、状態と行動 (Action) を返す。

  recovery_enabled: false (通知だけ)
    NORMAL ─ 疑い ─► SUSPECT ─ 確認 ─► DEGRADED ─ 異常がなくなる (clear_ticks 回) ─► NORMAL
                       └─ 異常がなくなる (clear_ticks 回) ─► NORMAL

  recovery_enabled: true
    NORMAL ─ 疑い ─► SUSPECT ─ 確認 ─► ISOLATED (blame に応じて切り離す)
                                         │ isolate_hold_sec 待つ
                                         ▼
    NORMAL ◄─ 収束を確認 (recover_ok_ticks 回) ─ RECOVERING (再初期化)
                                         │ 時間切れが max_reinit_attempts 回
                                         ▼
                                       DEGRADED (degraded_retry_sec ごとに RECOVERING へ)
"""
import enum
from dataclasses import dataclass, field
from typing import List, Optional

from .interfaces import FusionResult


class State(enum.IntEnum):
    """localization_supervisor_msgs/LocalizationStatus の state と対応する。"""
    NORMAL = 0
    SUSPECT = 1
    ISOLATED = 2
    RECOVERING = 3
    DEGRADED = 4


class Action(str, enum.Enum):
    ISOLATE = 'isolate'                  # blame の情報源を切り離す
    REINIT = 'reinit'                    # 候補を選んで推定器を初期化し直す
    RESTORE = 'restore'                  # 切り離したものを戻す
    NOTIFY_DEGRADED = 'notify_degraded'  # 異常を確認した (通知だけ、または復旧できない)


@dataclass
class MachineConfig:
    recovery_enabled: bool = False
    # recovery_enabled のとき、切り離してから戻すまで、走行を止めてほしいと要求するか
    request_hold: bool = True
    # 異常がなくなった周期がこの回数続けば、SUSPECT・DEGRADED (通知だけ) から NORMAL に戻る
    clear_ticks: int = 5
    # 切り離してから、初期化し直すまで待つ時間 [s]
    isolate_hold_sec: float = 3.0
    # 初期化し直した後、戻すまでに収束を確認する回数と、諦めるまでの時間 [s]
    recover_ok_ticks: int = 5
    recover_timeout_sec: float = 30.0
    max_reinit_attempts: int = 3
    # DEGRADED から再試行する間隔 [s] (recovery_enabled のとき)
    degraded_retry_sec: float = 60.0


@dataclass
class Decision:
    state: State
    actions: List[Action] = field(default_factory=list)
    event: str = ''
    blame: str = ''


class StateMachine:
    def __init__(self, config: Optional[MachineConfig] = None) -> None:
        self.config = config or MachineConfig()
        self.state = State.NORMAL
        self.blame = ''
        self._clear_run = 0
        self._ok_run = 0
        self._isolated_at = 0.0
        self._reinit_at = 0.0
        self._degraded_at = 0.0
        self._attempts = 0

    @property
    def hold_requested(self) -> bool:
        """走行を止めてほしい状態か (切り離してから、戻すまで)。"""
        return (self.config.recovery_enabled and self.config.request_hold
                and self.state in (State.ISOLATED, State.RECOVERING, State.DEGRADED))

    def step(self, now: float, fusion: FusionResult, converged: Optional[bool] = None) -> Decision:
        cfg = self.config
        if self.state == State.NORMAL:
            if fusion.suspect:
                self.state = State.SUSPECT
                self._clear_run = 0
                return Decision(self.state, [], f'suspect: {fusion.reason}', fusion.blame)
            return Decision(self.state)

        if self.state == State.SUSPECT:
            if fusion.confirmed:
                self.blame = fusion.blame or 'unknown'
                if not cfg.recovery_enabled:
                    self.state = State.DEGRADED
                    self._clear_run = 0
                    return Decision(self.state, [Action.NOTIFY_DEGRADED], f'alert: {fusion.reason}', self.blame)
                self.state = State.ISOLATED
                self._isolated_at = now
                return Decision(self.state, [Action.ISOLATE], f'isolated: {fusion.reason}', self.blame)
            if self._cleared(fusion):
                return self._to_normal([], 'cleared')
            return Decision(self.state, [], '', fusion.blame)

        if self.state == State.ISOLATED:
            if now - self._isolated_at >= cfg.isolate_hold_sec:
                return self._start_recovery(now, 'reinit')
            return Decision(self.state, [], '', self.blame)

        if self.state == State.RECOVERING:
            if converged is False:
                self._ok_run = 0
            elif converged is True:
                self._ok_run += 1
            if self._ok_run >= cfg.recover_ok_ticks:
                return self._to_normal([Action.RESTORE], 'recovered')
            if now - self._reinit_at >= cfg.recover_timeout_sec:
                if self._attempts < cfg.max_reinit_attempts:
                    self._attempts += 1
                    self._reinit_at = now
                    self._ok_run = 0
                    return Decision(self.state, [Action.REINIT], f'reinit retry {self._attempts}', self.blame)
                self.state = State.DEGRADED
                self._degraded_at = now
                return Decision(self.state, [Action.NOTIFY_DEGRADED], 'degraded', self.blame)
            return Decision(self.state, [], '', self.blame)

        # DEGRADED
        if not cfg.recovery_enabled:
            if self._cleared(fusion):
                return self._to_normal([Action.RESTORE], 'cleared')
            return Decision(self.state, [], '', self.blame)
        if now - self._degraded_at >= cfg.degraded_retry_sec:
            return self._start_recovery(now, 'retry from degraded')
        return Decision(self.state, [], '', self.blame)

    def _cleared(self, fusion: FusionResult) -> bool:
        if fusion.cleared:
            self._clear_run += 1
        elif fusion.anomaly:
            self._clear_run = 0
        return self._clear_run >= self.config.clear_ticks

    def _to_normal(self, actions: List[Action], event: str) -> Decision:
        self.state = State.NORMAL
        self.blame = ''
        self._clear_run = 0
        return Decision(self.state, actions, event)

    def _start_recovery(self, now: float, event: str) -> Decision:
        self.state = State.RECOVERING
        self._reinit_at = now
        self._attempts = 1
        self._ok_run = 0
        return Decision(self.state, [Action.REINIT], event, self.blame)
