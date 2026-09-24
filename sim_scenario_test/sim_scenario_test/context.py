from __future__ import annotations

import random
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.template import expand

if TYPE_CHECKING:
    import rclpy.node
    from sim_scenario_test.nav2 import Nav2Interface
    from sim_scenario_test.poses import PoseResolver
    from sim_scenario_test.profile import Profile
    from sim_scenario_test.registry import Registry
    from sim_scenario_test.scenario import Scenario
    from sim_scenario_test.sim.base import SimulationBackend
    from sim_scenario_test.spec import ActionCall

_POLL_SEC = 0.02


class Clock:
    """時刻源。ROS ノードの時計 (use_sim_time 時はシミュレーション時間) を使う。"""

    def __init__(self, node: "rclpy.node.Node"):
        self._clock = node.get_clock()

    def now(self) -> float:
        return self._clock.now().nanoseconds * 1e-9

    def sleep(self, sec: float, stop_event: Optional[threading.Event] = None) -> bool:
        """sec 秒待つ。stop_event で中断された場合は False を返す。"""
        deadline = self.now() + sec
        while self.now() < deadline:
            if stop_event is not None and stop_event.is_set():
                return False
            time.sleep(_POLL_SEC)
        return True


@dataclass
class Event:
    name: str
    time: float
    attrs: Dict[str, Any] = field(default_factory=dict)


class EventLog:
    """走行中に発生したイベント (goal_started / goal_reached 等) の記録。"""

    def __init__(self, clock: Clock):
        self._clock = clock
        self._lock = threading.Lock()
        self._events: List[Event] = []

    def emit(self, name: str, **attrs: Any) -> None:
        with self._lock:
            self._events.append(Event(name, self._clock.now(), attrs))

    def find(self, name: str, **attrs: Any) -> Optional[Event]:
        with self._lock:
            for event in self._events:
                if event.name == name and all(
                        event.attrs.get(k) == v for k, v in attrs.items()):
                    return event
        return None

    def to_list(self, t0: float) -> List[Dict[str, Any]]:
        with self._lock:
            return [
                {"name": e.name, "t": round(e.time - t0, 3), **e.attrs}
                for e in self._events
            ]


class ScenarioContext:
    """シナリオ実行中に action / trigger / driver 等が共有する実行環境。"""

    def __init__(
        self,
        node: "rclpy.node.Node",
        registry: "Registry",
        scenario: "Scenario",
        profile: "Profile",
        clock: Clock,
        backend: "SimulationBackend",
        poses: "PoseResolver",
        nav2: "Nav2Interface",
        seed: Optional[int] = None,
    ):
        self.node = node
        self.registry = registry
        self.scenario = scenario
        self.profile = profile
        self.clock = clock
        self.backend = backend
        self.poses = poses
        self.nav2 = nav2
        self.events = EventLog(clock)
        self.logger = node.get_logger()
        self.world_vars = profile.world_vars(scenario.world, "scenario.world")
        self.abort_event = threading.Event()
        self.abort_reason: Optional[str] = None
        # true の場合、中断理由を基盤側の失敗 (ERROR) として扱う
        self.abort_is_error = False
        self.run_start_time: Optional[float] = None
        # プラグインが実行中の状態 (クライアント等) を保持するための領域
        self.extensions: Dict[str, Any] = {}
        self.rng = random.Random(scenario.seed if seed is None else seed)
        # spawn / move した障害物の最新のワールド座標姿勢
        self.entity_poses: Dict[str, Any] = {}
        self._spawned: List[str] = []
        self._lock = threading.Lock()

    @property
    def variables(self) -> Dict[str, Any]:
        return {"world": self.world_vars, "profile": {"name": self.profile.name}}

    def expand(self, text: str, where: str) -> str:
        return expand(text, self.variables, where)

    def elapsed(self) -> float:
        if self.run_start_time is None:
            return 0.0
        return self.clock.now() - self.run_start_time

    def abort(self, reason: str, error: bool = False) -> None:
        with self._lock:
            if self.abort_reason is None:
                self.abort_reason = reason
                self.abort_is_error = error
        self.abort_event.set()

    def run_actions(
        self, calls: "List[ActionCall]", stop_event: Optional[threading.Event] = None
    ) -> bool:
        """action を順に実行する。stop_event で中断された場合は False を返す。"""
        stop = stop_event or threading.Event()
        for call in calls:
            if stop.is_set():
                return False
            self.logger.info(f"[action] {call.name}")
            try:
                call.entry.impl(self, call.spec, stop)
            except ScenarioError as e:
                raise ScenarioError(f"{call.where}: {e}") from e
        return not stop.is_set()

    def track_spawned(self, name: str) -> None:
        with self._lock:
            if name not in self._spawned:
                self._spawned.append(name)

    def untrack_spawned(self, name: str) -> None:
        with self._lock:
            if name in self._spawned:
                self._spawned.remove(name)

    def spawned(self) -> List[str]:
        with self._lock:
            return list(self._spawned)
