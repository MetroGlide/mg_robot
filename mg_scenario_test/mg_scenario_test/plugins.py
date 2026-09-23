"""MG-01 固有の sim_scenario_test 拡張。

プロファイル (profiles/mg01.yaml) の plugins に列挙され、読み込み時に以下を登録する。
  driver          : mg_sequencer
  action          : sequencer_start / sequencer_set_index / sequencer_stop
  trigger         : sequencer_state
  waypoint_format : mg (mg_waypoint_navigation の waypoint.yaml)
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, List, Optional

from mg_waypoint_navigation.waypoint import WaypointsLoader
from sim_scenario_test.drivers.base import RunDriver
from sim_scenario_test.engine.result import Outcome
from sim_scenario_test.errors import ScenarioError
from sim_scenario_test.geometry import PoseSpec, yaw_from_quaternion
from sim_scenario_test.registry import (
    register_action,
    register_driver,
    register_trigger,
    register_waypoint_format,
)
from sim_scenario_test.spec import ActionCall
from sim_scenario_test.waypoints import IndexedPose

from mg_scenario_test.sequencer_client import SequencerClient
from mg_scenario_test.sequencer_tracker import Progress

if TYPE_CHECKING:
    from sim_scenario_test.context import ScenarioContext

DEFAULT_NAMESPACE = "waypoint_sequencer_node"


def _pose_spec_from_stamped(pose_stamped) -> PoseSpec:
    p = pose_stamped.pose.position
    q = pose_stamped.pose.orientation
    return PoseSpec(frame="map", x=p.x, y=p.y, z=p.z,
                    yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w))


@register_waypoint_format("mg")
def load_mg_waypoints(path: str) -> List[IndexedPose]:
    """mg_waypoint_navigation の waypoint.yaml (version 2.0)。"""
    return [
        IndexedPose(wp.index, _pose_spec_from_stamped(wp.pose))
        for wp in WaypointsLoader(path).load().get_all()
    ]


def sequencer_for(
    ctx: "ScenarioContext", namespace: str, idle_stall_sec: float = 10.0
) -> SequencerClient:
    """ネームスペースごとに 1 つの SequencerClient を共有する。"""
    key = f"mg_sequencer:{namespace}"
    if key not in ctx.extensions:
        ctx.extensions[key] = SequencerClient(ctx.node, namespace, idle_stall_sec)
    return ctx.extensions[key]


@dataclass
class WaypointHooks:
    """waypoint ごとに同期実行する action。"""
    before: List[ActionCall] = field(default_factory=list)
    after: List[ActionCall] = field(default_factory=list)
    # この waypoint へ向かわせる際の start のカウントダウン (省略時は driver の既定値)
    countdown_ms: Optional[int] = None


@dataclass
class MgSequencerSpec:
    namespace: str = DEFAULT_NAMESPACE
    # 省略時は sequencer が配信する ~/waypoints トピックから取得する
    waypoints_file: str = ""
    start_index: int = 0
    # true の場合、開始時と sequencer が IDLE で待機している waypoint で自動的に start を呼ぶ
    auto_start: bool = True
    countdown_ms: int = 0
    # waypoint 1 つあたりの制限 (sim 時間の秒)
    waypoint_timeout_sec: float = 300.0
    # この秒数 IDLE のまま進まない場合は ERROR とする (start 忘れの検出)
    idle_stall_sec: float = 10.0
    hooks: Dict[int, WaypointHooks] = field(default_factory=dict)


@register_driver("mg_sequencer", MgSequencerSpec)
class MgSequencerDriver(RunDriver):
    """走行を waypoint_sequencer に委ね、status を監視して waypoint の通過を判定する。

    goal_* イベントの index は開始位置からではなく waypoint リスト上の位置を表す。
    """

    def __init__(self, ctx: "ScenarioContext", spec: MgSequencerSpec):
        super().__init__(ctx, spec)
        self._client = sequencer_for(ctx, spec.namespace, spec.idle_stall_sec)

    def wait_ready(self, timeout_sec: float) -> None:
        if not self._client.wait_for_service(timeout_sec):
            raise ScenarioError(f"/{self.spec.namespace}/start not available")

    def _load_waypoints(self) -> List[IndexedPose]:
        if self.spec.waypoints_file:
            path = self.ctx.expand(self.spec.waypoints_file, "run.mg_sequencer.waypoints_file")
            return load_mg_waypoints(path)
        infos = self._client.fetch_waypoints()
        if infos is None:
            raise ScenarioError(
                f"/{self.spec.namespace}/waypoints not received and waypoints_file not set")
        return [IndexedPose(w.index, _pose_spec_from_stamped(w.pose)) for w in infos]

    def run(self) -> Outcome:
        ctx = self.ctx
        spec = self.spec
        waypoints = self._load_waypoints()
        total = len(waypoints)
        if not 0 <= spec.start_index < total:
            raise ScenarioError(f"start_index {spec.start_index} out of range (0..{total - 1})")
        unknown = sorted(set(spec.hooks) - {wp.index for wp in waypoints})
        if unknown:
            raise ScenarioError(f"hooks refer to unknown waypoint index {unknown}")

        if spec.auto_start:
            self._client.stop()
            self._client.set_next_index(spec.start_index)
        self._client.begin_run()

        reached = 0
        for seq in range(spec.start_index, total):
            wp = waypoints[seq]
            hooks = spec.hooks.get(wp.index, WaypointHooks())
            if ctx.abort_event.is_set():
                return self._aborted(total, reached, seq)
            ctx.logger.info(f"[mg_sequencer] waypoint {wp.index} ({seq + 1}/{total})")
            ctx.run_actions(hooks.before, ctx.abort_event)
            if spec.auto_start and (seq == spec.start_index or self._client.state == "IDLE"):
                countdown = hooks.countdown_ms if hooks.countdown_ms is not None \
                    else spec.countdown_ms
                self._client.start(countdown)
            ctx.events.emit("goal_started", index=seq, waypoint_index=wp.index)

            progress = self._client.wait_passed(
                seq, spec.waypoint_timeout_sec, ctx.abort_event, ctx.clock.now)
            if progress == Progress.REACHED:
                ctx.events.emit("goal_reached", index=seq, waypoint_index=wp.index)
                reached += 1
                ctx.run_actions(hooks.after, ctx.abort_event)
                continue
            if progress == Progress.STALLED:
                raise ScenarioError(
                    f"sequencer stayed IDLE before passing waypoint {wp.index} "
                    "(set auto_start: true or add sequencer_start)")
            if progress == Progress.ERROR:
                reason = f"sequencer entered ERROR at waypoint {wp.index}"
            elif ctx.abort_event.is_set():
                return self._aborted(total, reached, seq)
            else:
                reason = (f"waypoint {wp.index} not passed within "
                          f"{spec.waypoint_timeout_sec:.0f}s")
                self._client.stop()
            ctx.events.emit("goal_failed", index=seq, waypoint_index=wp.index, reason=reason)
            return Outcome(total, reached, seq, reason)
        return Outcome(total, reached)

    def _aborted(self, total: int, reached: int, seq: int) -> Outcome:
        self._client.stop()
        return Outcome(total, reached, seq, self.ctx.abort_reason)


@dataclass
class SequencerStartSpec:
    countdown_ms: int = 0
    namespace: str = DEFAULT_NAMESPACE


@register_action("sequencer_start", SequencerStartSpec)
def sequencer_start(
    ctx: "ScenarioContext", spec: SequencerStartSpec, stop: threading.Event
) -> None:
    """waypoint_sequencer の ~/start を呼ぶ。"""
    sequencer_for(ctx, spec.namespace).start(spec.countdown_ms)


@dataclass
class SequencerSetIndexSpec:
    index: int
    namespace: str = DEFAULT_NAMESPACE


@register_action("sequencer_set_index", SequencerSetIndexSpec)
def sequencer_set_index(
    ctx: "ScenarioContext", spec: SequencerSetIndexSpec, stop: threading.Event
) -> None:
    """次に向かう waypoint index を設定する (IDLE / SUSPENDED 時のみ有効)。"""
    sequencer_for(ctx, spec.namespace).set_next_index(spec.index)


@dataclass
class SequencerStopSpec:
    namespace: str = DEFAULT_NAMESPACE


@register_action("sequencer_stop", SequencerStopSpec)
def sequencer_stop(
    ctx: "ScenarioContext", spec: SequencerStopSpec, stop: threading.Event
) -> None:
    """waypoint_sequencer の ~/stop を呼び IDLE に戻す。"""
    sequencer_for(ctx, spec.namespace).stop()


@dataclass
class SequencerStateSpec:
    state: str
    # 指定した場合は current_index も一致したときのみ成立する
    index: Optional[int] = None
    namespace: str = DEFAULT_NAMESPACE


@register_trigger("sequencer_state", SequencerStateSpec)
class SequencerState:
    """waypoint_sequencer の状態が state になったら成立する。"""

    def __init__(self, ctx: "ScenarioContext", spec: SequencerStateSpec):
        self._client = sequencer_for(ctx, spec.namespace)
        self._spec = spec

    def poll(self) -> bool:
        if self._client.state != self._spec.state:
            return False
        return self._spec.index is None or self._client.index == self._spec.index
