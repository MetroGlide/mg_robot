from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, List, Optional

from sim_scenario_test.engine.result import CheckResult, ResultStatus
from sim_scenario_test.errors import ScenarioError

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext
    from sim_scenario_test.scenario import TimelineEntry
    from sim_scenario_test.spec import TriggerCall

_LOOP_SEC = 0.05


def instantiate_trigger(ctx: "ScenarioContext", call: "TriggerCall") -> Any:
    return call.entry.impl(ctx, call.spec)


@dataclass
class _Item:
    label: str
    entry: "TimelineEntry"
    when: Any
    until: Optional[Any]
    stop: threading.Event = field(default_factory=threading.Event)
    fired_at: Optional[float] = None
    finished: bool = False
    interrupted: bool = False
    thread: Optional[threading.Thread] = None


class Timeline:
    """タイムライン項目の発火判定と実行、シナリオ全体のタイムアウト監視を行う。"""

    def __init__(self, ctx: "ScenarioContext", entries: "List[TimelineEntry]"):
        self._ctx = ctx
        self._items = [
            _Item(
                label=entry.name or f"timeline[{i}]",
                entry=entry,
                when=instantiate_trigger(ctx, entry.when),
                until=instantiate_trigger(ctx, entry.until) if entry.until else None,
            )
            for i, entry in enumerate(entries)
        ]
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.errors: List[str] = []

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        for item in self._items:
            item.stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        for item in self._items:
            if item.thread is not None:
                item.thread.join(timeout=10.0)

    def checks(self) -> List[CheckResult]:
        results = []
        for item in self._items:
            if item.fired_at is None and item.entry.required:
                results.append(CheckResult(
                    item.label, ResultStatus.ERROR,
                    "never triggered (set required: false if this is intended)"))
            elif item.fired_at is not None:
                note = "interrupted" if item.interrupted else "completed"
                results.append(CheckResult(
                    item.label, ResultStatus.PASSED,
                    f"fired at t={item.fired_at:.1f}s, {note}"))
        with self._lock:
            results.extend(
                CheckResult("timeline", ResultStatus.ERROR, e) for e in self.errors)
        return results

    def _fail(self, message: str) -> None:
        with self._lock:
            self.errors.append(message)
        self._ctx.abort(f"timeline error: {message}")

    def _loop(self) -> None:
        timeout = self._ctx.scenario.timeout
        stall_sec = self._ctx.profile.sim.clock_stall_sec
        last_sim = self._ctx.clock.now()
        last_change = time.monotonic()
        while not self._stop.is_set():
            now_sim = self._ctx.clock.now()
            if now_sim != last_sim:
                last_sim = now_sim
                last_change = time.monotonic()
            elif time.monotonic() - last_change > stall_sec:
                self._ctx.abort(
                    f"simulation clock stalled for {stall_sec:.0f}s (wall)", error=True)
            if timeout is not None and self._ctx.elapsed() > timeout:
                self._ctx.abort(f"scenario timeout ({timeout:.0f}s sim time)")
            for item in self._items:
                try:
                    self._step(item)
                except ScenarioError as e:
                    self._fail(f"{item.label}: {e}")
            time.sleep(_LOOP_SEC)

    def _step(self, item: _Item) -> None:
        if item.fired_at is None:
            if item.when.poll():
                item.fired_at = self._ctx.elapsed()
                self._ctx.logger.info(f"[timeline] {item.label} fired")
                self._ctx.events.emit("timeline_fired", entry=item.label)
                item.thread = threading.Thread(
                    target=self._run_item, args=(item,), daemon=True)
                item.thread.start()
        elif not item.finished and item.until is not None and not item.stop.is_set():
            if item.until.poll():
                self._ctx.logger.info(f"[timeline] {item.label} interrupted by until")
                item.interrupted = True
                item.stop.set()

    def _run_item(self, item: _Item) -> None:
        try:
            if not self._ctx.run_actions(item.entry.do, item.stop):
                item.interrupted = True
        except ScenarioError as e:
            self._fail(f"{item.label}: {e}")
        finally:
            item.finished = True
