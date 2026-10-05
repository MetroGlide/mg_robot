from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext
    from sim_scenario_test.engine.result import Outcome


class RunDriver(ABC):
    """シナリオの「走行」を担うドライバ。

    run() は走行完了までブロックし、ctx.abort_event がセットされたら速やかに
    走行を止めて戻る。走行中は ctx.events に以下を記録する:
      goal_started(index) / goal_reached(index) / goal_failed(index)
    インフラ・セットアップの失敗は ScenarioError を送出する。
    """

    def __init__(self, ctx: "ScenarioContext", spec: Any):
        self.ctx = ctx
        self.spec = spec

    def wait_ready(self, timeout_sec: float) -> None:
        """走行に必要なサーバ等が利用可能になるまで待つ。"""

    @abstractmethod
    def run(self) -> "Outcome":
        ...

    def close(self) -> None:
        """リソースを解放する。"""
