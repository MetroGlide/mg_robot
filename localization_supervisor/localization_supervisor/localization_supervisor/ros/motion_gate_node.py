"""門のノード: 推定器の姿勢を融合に入れる前に確かめる (doc/motion_gate.md)。

中核 (core/motion_gate.py) を周期 (period_sec) ごとに回し、門の開閉をゲートの調停ノードに伝え、置き直し・
一時停止の要求・状態・診断・イベントを出す。照合は重いので、周期の処理と別のスレッドで回す。

  パラメータ
    config_file : 設定ファイル (必須)
  入力
    設定の sources.<名前>.input のアダプタが購読するトピックと TF、manual_pose_topic (手動の初期姿勢)
  出力
    gate_service (SetBool)、resetters の送り先、hold_output、output.status_topic (LocalizationStatus)、
    診断 output.diagnostic_name、イベント (events.sink)
"""
import threading
import time
from typing import Optional, Tuple

import rclpy
from diagnostic_updater import Updater
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_supervisor_msgs.msg import LocalizationStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

from ..core.config import ConfigError
from ..core.interfaces import Candidate
from ..core.map_match import MapMatcher, MatchDecision
from ..core.motion_gate import GateStep, MatchRequest, MotionGate
from ..core.motion_gate_config import load_gate_config
from ..core.snapshot import SampleStore
from ..core.status import StatusView, gate_status_view
from .adapters import build_adapters
from .adapters.standard import pose_from_msg
from .diagnostics import LocalizationTask
from .event_sinks import build_event_sink
from .hold_outputs import build_hold_output
from .own_messages import OwnMessages
from .resetters import build_resetters
from .status import to_message
from .supervisor_node import _QOS, TfContext


class MotionGateNode(Node):
    def __init__(self) -> None:
        super().__init__('motion_gate_node')
        config_file = self.declare_parameter('config_file', '').value
        if not config_file:
            raise ConfigError('parameter config_file is required')
        config = load_gate_config(config_file)
        self._config = config
        self._gate = MotionGate(config.gate)
        self._matcher = MapMatcher(config.gate.map_match)
        self._store = SampleStore(config.gate.kinds, horizon_sec=config.horizon_sec,
                                  max_extrapolation_sec=config.max_extrapolation_sec)
        self._tf = TfContext(self)
        for adapter in build_adapters(config.sources, self._store, self._tf).values():
            for sub in adapter.subscriptions():
                self.create_subscription(sub.msg_type, sub.topic, sub.handler, _QOS[sub.qos])
        own = OwnMessages()
        self._resetters = {role: build_resetters(items, self, own) for role, items in config.resetters.items()}
        self._hold = build_hold_output(config.hold_output, self)
        self._gate_client = self.create_client(SetBool, config.gate_service)
        self._gate_applied: Optional[bool] = None
        self._gate_pending = False
        if config.manual_pose_topic:
            self.create_subscription(PoseWithCovarianceStamped, config.manual_pose_topic, self._on_manual, 10)

        self._lock = threading.Lock()
        self._result: Optional[Tuple[MatchRequest, MatchDecision]] = None
        self._manual: Optional[tuple] = None
        self._match_ms = 0.0
        self._last_event = ''

        output = config.output
        self._status_pub = self.create_publisher(LocalizationStatus, output.status_topic, 10)
        self._events = build_event_sink(config.event_sink, self.get_logger())
        self._view: Optional[StatusView] = None
        self._updater = Updater(self)
        self._updater.setHardwareID(output.hardware_id)
        self._updater.add(LocalizationTask(output.diagnostic_name, lambda: self._view))
        self.create_timer(config.period_sec, self._on_timer)
        self._emit('node.started', 'info', name=self.get_name(), config=config_file,
                   closers=','.join(c.name for c in config.gate.closers))

    def _emit(self, name: str, level: str, **fields) -> None:
        self._events.emit(f'{self._config.event_prefix}.{name}', level, fields)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_manual(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._manual = pose_from_msg(msg.pose.pose)

    def _match(self, request: MatchRequest) -> None:
        started = time.monotonic()
        decision = self._matcher.decide(request.field_for, request.points, request.candidates)
        with self._lock:
            self._match_ms = (time.monotonic() - started) * 1000.0
            self._result = (request, decision)

    def _on_timer(self) -> None:
        now = self._now()
        if now == 0.0:
            return
        with self._lock:
            result, self._result = self._result, None
            manual, self._manual = self._manual, None
            match_ms = self._match_ms
        if manual is not None:
            self._gate.manual_pose(manual)
        step = self._gate.step(self._store.snapshot(now), result)
        for event in step.events:
            fields = dict(event.fields)
            if event.name == 'map_match':
                fields['duration_ms'] = round(match_ms, 1)
            self._emit(event.name, event.level, **fields)
            self._last_event = event.name
        if step.reset is not None:
            candidate = Candidate(step.reset.pose, step.reset.cov, 'motion_gate')
            roles = ('estimator', 'fused') if step.reset.targets == 'all' else ('fused',)
            for role in roles:
                for resetter in self._resetters[role]:
                    resetter.reset(candidate)
        self._set_gate(step.gate_open)
        if self._hold is not None:
            self._hold.apply(step.hold, step.hold_reason)
        if step.request is not None:
            threading.Thread(target=self._match, args=(step.request,), daemon=True).start()
        self._publish_status(step)

    def _publish_status(self, step: GateStep) -> None:
        view = gate_status_view(step, self._config.gate, self._last_event)
        self._view = view
        self._status_pub.publish(to_message(view, self.get_clock().now().to_msg()))

    def _set_gate(self, attach: bool) -> None:
        """変わったとき・まだ届いていないときだけ呼ぶ (サービスがなくても、次の周期で再び試す)。"""
        if attach == self._gate_applied or self._gate_pending or not self._gate_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = attach
        self._gate_pending = True
        self._gate_client.call_async(request).add_done_callback(lambda f, want=attach: self._on_gate_done(f, want))

    def _on_gate_done(self, future, attach: bool) -> None:
        self._gate_pending = False
        response = future.result()
        if response is not None and response.success:
            self._gate_applied = attach


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
