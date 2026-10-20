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

    RECOVERING (まだ初期化し直していない) と DEGRADED で、異常がなくなった周期が clear_ticks 回続けば、
    初期化せずに NORMAL へ戻す (一時的な外れ値だった場合に、走行を止め続けない)

    blame が isolate_only の情報源のときは、ISOLATED のまま、止めず、再初期化もせず、異常がなくなる
    (clear_ticks 回) のを待って NORMAL に戻る。別の情報源が blame で異常を確認したら、その blame の対処に移る
    (blame が決めきれない間は、切り離したまま待つ)
"""
import enum
from dataclasses import dataclass, field
from typing import List, Optional

from .interfaces import FusionResult

# 疑わしい情報源を決めきれないときの blame
UNKNOWN = 'unknown'


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
    # blame がこれらの情報源のときは、切り離す (信頼度を下げる) だけにして、走行を止めず、再初期化もしない。
    # 異常がなくなれば戻る。別の blame で異常を確認したら、その blame の対処に移る
    isolate_only: List[str] = field(default_factory=list)


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
        self._reinit_done = False
        # isolate_only の blame を切り離しているだけの間 (止めず、再初期化もしない)
        self._soft = False

    def note_reinit(self) -> None:
        """推定器を初期化し直す指示を出した。これ以降は、収束を確認して戻す (それまでは、異常が消えても戻せる)。"""
        self._reinit_done = True

    @property
    def isolating(self) -> bool:
        """blame の情報源を切り離している状態か (切り離してから、戻すまで)。"""
        return self.config.recovery_enabled and self.state in (State.ISOLATED, State.RECOVERING, State.DEGRADED)

    @property
    def hold_requested(self) -> bool:
        """走行を止めてほしい状態か (切り離してから、戻すまで。isolate_only の blame は止めない)。"""
        return self.isolating and self.config.request_hold and not self._soft

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
                self.blame = fusion.blame or UNKNOWN
                if not cfg.recovery_enabled:
                    self.state = State.DEGRADED
                    self._clear_run = 0
                    return Decision(self.state, [Action.NOTIFY_DEGRADED], f'alert: {fusion.reason}', self.blame)
                self.state = State.ISOLATED
                self._isolated_at = now
                self._soft = self.blame in cfg.isolate_only
                return Decision(self.state, [Action.ISOLATE], f'isolated: {fusion.reason}', self.blame)
            if self._cleared(fusion):
                return self._to_normal([], 'cleared')
            return Decision(self.state, [], '', fusion.blame)

        if self.state == State.ISOLATED:
            if self._soft:
                return self._step_soft(now, fusion)
            if self._narrowed(fusion):
                return self._to_soft(fusion)
            if now - self._isolated_at >= cfg.isolate_hold_sec:
                return self._start_recovery(now, 'reinit')
            return Decision(self.state, [], '', self.blame)

        if self.state == State.RECOVERING:
            # まだ初期化し直していないうちに、異常がなくなった (一時的な外れ値だった) なら、初期化せずに戻す。
            # 初期化し直した後は、推定器が収束したかで戻す (停止中は AMCL が更新されないので、異常の有無は決められない)
            if not self._reinit_done and self._narrowed(fusion):
                return self._to_soft(fusion)
            if not self._reinit_done and self._cleared(fusion):
                return self._to_normal([Action.RESTORE], 'cleared')
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
                    self._reinit_done = False
                    return Decision(self.state, [Action.REINIT], f'reinit retry {self._attempts}', self.blame)
                self.state = State.DEGRADED
                self._degraded_at = now
                self._clear_run = 0
                return Decision(self.state, [Action.NOTIFY_DEGRADED], 'degraded', self.blame)
            return Decision(self.state, [], '', self.blame)

        # DEGRADED
        if self._cleared(fusion):
            return self._to_normal([Action.RESTORE], 'cleared')
        if cfg.recovery_enabled and now - self._degraded_at >= cfg.degraded_retry_sec:
            return self._start_recovery(now, 'retry from degraded')
        return Decision(self.state, [], '', self.blame)

    def _narrowed(self, fusion: FusionResult) -> bool:
        """決めきれずに止めたが、まだ再初期化していないうちに、blame が isolate_only の情報源だと分かった
        (正しいと示す判定が、後から判定できるようになった場合など)。"""
        return self.config.recovery_enabled and fusion.confirmed and fusion.blame in self.config.isolate_only

    def _to_soft(self, fusion: FusionResult) -> Decision:
        self.state = State.ISOLATED
        self._soft = True
        self.blame = fusion.blame
        self._clear_run = 0
        return Decision(self.state, [Action.ISOLATE], f'isolated: narrowed to {self.blame}', self.blame)

    def _step_soft(self, now: float, fusion: FusionResult) -> Decision:
        """isolate_only の blame を切り離している間。異常がなくなれば戻る。別の情報源が blame で異常を確認したら、
        その blame の対処 (切り離し・停止・再初期化) に移る。決めきれない (unknown) 間は、今の状態を保つ
        (正しいと示す判定が、停止中などで判定できないだけのことがある。本当にずれていれば blame が決まる)。"""
        if (fusion.confirmed and fusion.blame and fusion.blame != UNKNOWN
                and fusion.blame not in self.config.isolate_only):
            self._soft = False
            self.blame = fusion.blame
            self._isolated_at = now
            return Decision(self.state, [Action.ISOLATE], f'isolated: escalate to {self.blame}', self.blame)
        if self._cleared(fusion):
            return self._to_normal([Action.RESTORE], 'cleared')
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
        self._soft = False
        self._clear_run = 0
        return Decision(self.state, actions, event)

    def _start_recovery(self, now: float, event: str) -> Decision:
        self.state = State.RECOVERING
        self._reinit_at = now
        self._attempts = 1
        self._ok_run = 0
        self._clear_run = 0
        self._reinit_done = False
        return Decision(self.state, [Action.REINIT], event, self.blame)
