from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


class ResultStatus(enum.Enum):
    PASSED = "PASSED"
    FAILED = "FAILED"
    ERROR = "ERROR"


EXIT_CODES = {
    ResultStatus.PASSED: 0,
    ResultStatus.FAILED: 1,
    ResultStatus.ERROR: 2,
}


@dataclass
class CheckResult:
    name: str
    status: ResultStatus
    message: str = ""


@dataclass
class Outcome:
    """走行ドライバの結果。失敗は FAILED/ERROR の判定材料として expectation が評価する。"""
    total: int = 0
    reached: int = 0
    failed_index: int = -1
    failure: Optional[str] = None


@dataclass
class ScenarioResult:
    name: str
    status: ResultStatus
    checks: List[CheckResult]
    outcome: Outcome
    elapsed_wall_sec: float = 0.0
    elapsed_sim_sec: float = 0.0
    events: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def success(self) -> bool:
        return self.status == ResultStatus.PASSED

    def to_dict(self) -> Dict[str, Any]:
        return {
            "scenario_name": self.name,
            "status": self.status.value,
            "elapsed_wall_sec": round(self.elapsed_wall_sec, 2),
            "elapsed_sim_sec": round(self.elapsed_sim_sec, 2),
            "goals": {
                "reached": self.outcome.reached,
                "total": self.outcome.total,
                "failed_index": self.outcome.failed_index,
                "failure": self.outcome.failure,
            },
            "checks": [
                {"name": c.name, "status": c.status.value, "message": c.message}
                for c in self.checks
            ],
            "events": self.events,
        }


def aggregate(checks: List[CheckResult]) -> ResultStatus:
    statuses = {c.status for c in checks}
    if ResultStatus.ERROR in statuses:
        return ResultStatus.ERROR
    if ResultStatus.FAILED in statuses:
        return ResultStatus.FAILED
    return ResultStatus.PASSED
