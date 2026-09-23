from __future__ import annotations

import enum
from typing import Optional

ACTIVE_STATES = frozenset({"ON_STARTING", "NAVIGATING", "ON_ARRIVING", "SUSPENDED"})


class Progress(enum.Enum):
    WAITING = "WAITING"
    REACHED = "REACHED"
    ERROR = "ERROR"
    STALLED = "STALLED"


class SequencerProgressTracker:
    """waypoint_sequencer の status 列から waypoint 通過を判定する純粋ロジック。

    シナリオ開始前に残っている古い status (前回走行の GOAL_REACHED 等) で
    誤って「通過」と判定しないよう、今回の走行で一度でも稼働状態
    (ACTIVE_STATES) を観測するまでは通過と見なさない。
    """

    def __init__(self, idle_stall_sec: float):
        self._idle_stall_sec = idle_stall_sec
        self._armed = False
        self._state: Optional[str] = None
        self._index: Optional[int] = None
        self._idle_since: Optional[float] = None

    @property
    def state(self) -> Optional[str]:
        return self._state

    @property
    def index(self) -> Optional[int]:
        return self._index

    def reset_run(self) -> None:
        self._armed = False

    def update(self, state: str, index: int, now: float) -> None:
        if state in ACTIVE_STATES:
            self._armed = True
        if state == "IDLE":
            if self._state != "IDLE" or self._idle_since is None:
                self._idle_since = now
        else:
            self._idle_since = None
        self._state = state
        self._index = index

    def evaluate(self, expected_index: int, wait_started_at: float, now: float) -> Progress:
        if self._state is None:
            return Progress.WAITING
        if self._state == "ERROR":
            return Progress.ERROR
        if self._armed and self._index is not None and self._index > expected_index:
            return Progress.REACHED
        if self._state == "IDLE" and self._idle_since is not None:
            idle_from = max(self._idle_since, wait_started_at)
            if now - idle_from >= self._idle_stall_sec:
                return Progress.STALLED
        return Progress.WAITING
